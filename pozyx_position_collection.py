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

            data_string += str(time_ns() - self.start_time) + ","
            data_string += idx_pozyx*","
            data_string += str(position.x) + ","
            data_string += str(position.y) + ","
            data_string += str(position.z) + ","
            data_string += "\n"


        return data_string

            
if __name__ == "__main__":

    # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    anchors = [pypozyx.DeviceCoordinates(0xA001, 1, pypozyx.Coordinates(0, 0, 2790)),
               pypozyx.DeviceCoordinates(0xA002, 1, pypozyx.Coordinates(10490, 0, 2790)),
               pypozyx.DeviceCoordinates(0xA003, 1, pypozyx.Coordinates(-405, 6000, 2790)),
               pypozyx.DeviceCoordinates(0xA004, 1, pypozyx.Coordinates(10490, 6500, 2790))]

    pc = PositionCollector(anchors)
    pc.stream(30)
    