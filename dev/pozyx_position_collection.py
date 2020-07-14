#!/usr/bin/env python3
import pypozyx
from time import time_ns, sleep
from datetime import datetime


class PositionCollector(object):

    def __init__(self,anchors):
        super().__init__()

        # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        self.algorithm = pypozyx.PozyxConstants.POSITIONING_ALGORITHM_TRACKING    
        self.dimension = pypozyx.PozyxConstants.DIMENSION_3D

        self.start_time = time_ns()
        self.time_previous = time_ns()
        self.current_time = self.time_previous

        self.pozyxs = self.findPozyxSerial()
        self.findNeighbors()
        self.setAnchorsManual(anchors)
        print('Initalization complete.')

    def findPozyxSerial(self):
        """
        Automatically checks this computer's serial ports for any pozyx 
        devices connected through USB.
        """
        # Detect Pozyx device on serial ports, get serial port address.
        pozyx_devices = list()
        for port in pypozyx.get_serial_ports():
            if pypozyx.is_pozyx_port(port):
                pozyx_devices.append(port.device)

        
        if not pozyx_devices:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()
        else:
            print("Pozyx device(s) detected on serial port(s): " + str(pozyx_devices)+ ".")
        
        # Initialize connection to serial port.
        pozyx_serials = list()
        for serial_port in pozyx_devices:
            pozyx = pypozyx.PozyxSerial(serial_port)
            pozyx_serials.append(pozyx)
            
        return pozyx_serials

    def findNeighbors(self):
        """ 
        Automatically discovers any pozyx anchor or tag devices within UWB range.

        Returns:
            device_list: [pypozyx.DeviceList] list containing the device IDs
        """
        for pozyx in self.pozyxs:
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)
            status = pozyx.clearDevices()
            status = pozyx.doDiscoveryAll()

            device_list_size = pypozyx.SingleRegister()
            status = pozyx.getDeviceListSize(device_list_size)

            device_list = pypozyx.DeviceList(list_size = device_list_size.value)
            status = pozyx.getDeviceIds(device_list)

            id_string = "Device " + str(hex(who_am_i.id))+" has discovered the following other devices: "
            for id in device_list.data:
                id_string += str(hex(id)) + ", "
                print(id_string)
            return device_list

    def setAnchorsManual(self,anchors):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for pozyx in self.pozyxs:
            status = pozyx.clearDevices()
            for anchor in anchors:
                status &= pozyx.addDevice(anchor)
            if len(anchors) > 4:
                status &= pozyx.setSelectionOfAnchors(pypozyx.PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors))

            return status

    def createDataFile(self,filename = None):
        """
        Creates a new csv file with the following default file name,
        
        YYYY_MM_DD_HH_MM_SS_agmp.csv

        unless overwritten by the user. Also creates the header.

        Args:
            filename: [string](optional) custom filename
        Returns:
            file: file object
        """

        # Generate the filename string
        if filename is None:
            now = datetime.now()
            filename = now.strftime("%Y_%m_%d_%H_%M_%S")
            filename = filename + "_positioning.csv"

        header_device = ","
        header_data = "Timestamp (ns),"
        for pozyx in self.pozyxs:
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)

            # Generate the header string
            self.number_data_columns = 0
            header_device += (str(hex(who_am_i.id)) + ",")*3
            header_data += "Pos_x (mm), Pos_y (mm), Pos_z (mm),"
            self.number_data_columns += 3
          
        header_device += "\n"
        header_data += "\n"
            
        # Create the file and insert the header.
        file = open(filename,"a")
        file.truncate(0)
        file.write(header_device)
        file.write(header_data)
        print(header_device[:-1])
        print(header_data[:-1])
        return file

    def record(self, duration):
        """
        Prints data to screen and saves to a file for a given duration.

        Args:
            duration: [float] Amount of time to stream data, in seconds.
        """


        response = input("Ready to stream data? (y/n)")
        if response == "n" or response == "no":
            quit()

        file = self.createDataFile()

        self.start_time = time_ns()

        while (time_ns() - self.start_time)*(10**-9) < duration:

            data_string = self.getData()

            # Write to file
            print(data_string[:-1])
            file.write(data_string)
                

        # Close file
        file.flush()
        file.close()

    def stream(self, duration):
        """
        Prints data to screen for a given duration.

        Args:
            duration: [float] Amount of time to stream data, in seconds.
        """

        response = input("Ready to stream data? (y/n)")
        if response == "n" or response == "no":
            quit()

        self.start_time = time_ns()
        while (time_ns() - self.start_time)*(10**-9) < duration:
            data_string = self.getData()
            # Write to file
            print(data_string[:-1])

    def getData(self):
        
        position = pypozyx.Coordinates()
        data_string = ""
        
        for idx_pozyx, pozyx in enumerate(self.pozyxs):
            
            status = pozyx.doPositioning(position, self.dimension, self.algorithm)

            if status is pypozyx.POZYX_SUCCESS:
                data_string += str(time_ns()) + ","
                data_string += idx_pozyx*","*3
                data_string += str(position.x) + ","
                data_string += str(position.y) + ","
                data_string += str(position.z) + ","
                data_string += "\n"


        return data_string

            
if __name__ == "__main__":

    # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    anchors = [pypozyx.DeviceCoordinates(0x6f4a, 1, pypozyx.Coordinates(3272, -2122, 1831)),
               pypozyx.DeviceCoordinates(0x6f58, 1, pypozyx.Coordinates(-106, 2871, 1620)),
               pypozyx.DeviceCoordinates(0x6f5f, 1, pypozyx.Coordinates(5758, 1901, 2120)),
               pypozyx.DeviceCoordinates(0x6f60, 1, pypozyx.Coordinates(-3667, 176, 1820)),
               pypozyx.DeviceCoordinates(0x6f61, 1, pypozyx.Coordinates(150, -2091, 472))]

    pc = PositionCollector(anchors)
    pc.record(180)
    