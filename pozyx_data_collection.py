#!/usr/bin/env python3
import pypozyx
from time import time_ns, sleep
from datetime import datetime
"""
This code is currently applicable for a device (such as a raspberry pi)
which has many pozyx devices connected to it through USB. This code will 
automatically detect all the pozyx devices connected to the master device, 
as well as any other pozyx devices that are active within UWB range.

This code can then be used to collect IMU data from the connected devices,
and range data between the connected devices and all other active pozyx 
devices in UWB range.
"""
class PozyxDataCollector(object):
    
    def __init__(self):
        super().__init__()

        # Settings
        self.record_accel = True
        self.record_gyro = True
        self.record_mag = True
        self.record_pres = True
        self.record_quat = False
        self.record_range = True
        self.allow_self_ranging = False

        # Internal variables
        self.start_time = time_ns()
        self.time_previous = time_ns()
        self.current_time = self.time_previous
        self.number_data_columns = 0
        
        

        self.pozyxs, self.ids = self.findPozyxSerial()

        print('Initalization complete.')

    def findPozyxSerial(self):
        """
        Automatically checks this computer's serial ports for any pozyx 
        devices connected through USB.

        Returns:
            pozyxs: [list] of [pypozyx.PozyxSerial] containing one
                PozyxSerial() object per connected pozyx device.
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
        pozyx_ids = list()
        self.device_lists = list()
        for serial_port in pozyx_devices:
            pozyx = pypozyx.PozyxSerial(serial_port)
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)
            if status == pypozyx.POZYX_FAILURE:
                print("ERROR: Failed to obtain device ID.")

            pozyx_serials.append(pozyx)
            pozyx_ids.append(who_am_i.id)
            self.device_lists.append(pypozyx.DeviceList(list_size = 1))
            
        return pozyx_serials, pozyx_ids

    def getHeader(self):
        header_device = ","
        header_data = "Timestamp (ns),"
        for idx_pozyx, pozyx in enumerate(self.pozyxs):
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)

            # Generate the header string
            self.number_data_columns = 0
            if self.record_accel:
                header_device += (str(hex(who_am_i.id)) + ",")*3
                header_data += "Accel_x (mg), Accel_y (mg), Accel_z (mg),"
                self.number_data_columns += 3

            if self.record_gyro:
                header_device += (str(hex(who_am_i.id)) + ",")*3
                header_data += "Gyro_x (deg/s), Gyro_y (deg/s), Gyro_z (deg/s),"
                self.number_data_columns += 3

            if self.record_mag:
                header_device += (str(hex(who_am_i.id)) + ",")*3
                header_data += "Mag_x (uT), Mag_y (uT), Mag_z (uT),"
                self.number_data_columns += 3

            if self.record_quat:
                header_device += (str(hex(who_am_i.id)) + ",")*4
                header_data += "Quat_w, Quat_x, Quat_y, Quat_z,"
                self.number_data_columns += 4

            if self.record_pres:
                header_device += (str(hex(who_am_i.id)) + ",")*1
                header_data += "Pressure (Pa),"
                self.number_data_columns += 1

            if self.record_range:
                device_list = self.device_lists[idx_pozyx]
                for id in device_list.data:
                    header_device += (str(hex(who_am_i.id)) + ",")*2
                    header_data += 'Range to ' + str(hex(id)) + ' (mm),'
                    header_data += 'RSS to ' + str(hex(id)) + ' (dB),'
                    self.number_data_columns += 2

        header_device += "\n"
        header_data += "\n"
        return header_device + header_data

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
            if self.record_accel:
                filename += "a"

            if self.record_gyro:
                filename += "g"

            if self.record_mag:
                filename += "m"

            if self.record_pres:
                filename += "p"

            if self.record_quat:
                filename += "q"

            if self.record_range:
                filename += "r"

            filename = filename + ".csv"
            
        # Create the file and insert the header.
        header = self.getHeader()
        file = open(filename,"a")
        file.truncate(0)
        file.write(header)
        return file

    

    def findNeighbors(self):
        """ 
        Automatically discovers any pozyx anchor or tag devices within UWB range.

        Returns:
            device_lists: [list] of [pypozyx.DeviceList] list containing the device IDs
        """
        device_lists = list()
        for pozyx in self.pozyxs:
            # Get ID of current pozyx device
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)

            # Discover other pozyx devices
            status = pozyx.clearDevices()
            status = pozyx.doDiscoveryAll()
            device_list_size = pypozyx.SingleRegister()
            status = pozyx.getDeviceListSize(device_list_size)
            device_list = pypozyx.DeviceList(list_size = device_list_size.value)
            status = pozyx.getDeviceIds(device_list)

            # Add to list of lists
            device_lists.append(device_list)

            # Print device list 
            id_string = "Device " + str(hex(who_am_i.id))+" has discovered the following other devices: "
            for id in device_list.data:
                id_string += str(hex(id)) + ", "
                
            print(id_string)

        if not self.allow_self_ranging:
            print("However, self-ranging is currently deactivated. " \
                + "No range measurements between devices connected to the same computer ")
        return device_lists


    def getData(self):
        """ 
        Collects the data from the PozyxSerial device(s) as soon as a new data point is available."
        """

        # Containers for storing the data 
        accel_data = pypozyx.Acceleration()
        gyro_data = pypozyx.AngularVelocity()
        mag_data = pypozyx.Magnetic()
        quat_data = pypozyx.Quaternion()
        pres_data = pypozyx.Pressure()
        device_range = pypozyx.DeviceRange()

        data_string = ""
        for idx_pozyx, pozyx in enumerate(self.pozyxs):
            # Wait for new data to become available
            # If taking range measurements, dont wait at all, as doRanging takes lots of time.
            if not self.record_range:
                interrupt_register = pypozyx.SingleRegister()
                pozyx.waitForFlagSafe(pypozyx.PozyxBitmasks.INT_MASK_IMU, 0.1, interrupt_register)

            device_list = self.device_lists[idx_pozyx]
            for idx, id in enumerate(device_list.data):
                if id in self.ids and not self.allow_self_ranging:
                    pass
                else:
                    if self.record_range:
                        status_range = pozyx.doRanging(id, device_range)
                        if status_range == pypozyx.POZYX_FAILURE:
                            # Try ranging a second time, sometimes this occurs if multiple devices are used.
                            status_range = pozyx.doRanging(id, device_range)

                    # Create datastring
                    data_string += str(time_ns()) + ","
                    data_string += ","*self.number_data_columns*idx_pozyx
                    if self.record_accel:
                        status = pozyx.getAcceleration_mg(accel_data)
                        data_string += str(accel_data.x) + "," 
                        data_string += str(accel_data.y) + "," 
                        data_string += str(accel_data.z) + ","

                    if self.record_gyro:
                        status = pozyx.getAngularVelocity_dps(gyro_data)
                        data_string += str(gyro_data.x) + "," 
                        data_string += str(gyro_data.y) + "," 
                        data_string += str(gyro_data.z) + ","

                    if self.record_mag:
                        status = pozyx.getMagnetic_uT(mag_data)
                        data_string += str(mag_data.x) + "," 
                        data_string += str(mag_data.y) + "," 
                        data_string += str(mag_data.z) + ","

                    if self.record_quat:
                        status = pozyx.getQuaternion(quat_data)
                        data_string += str(quat_data.w) + "," 
                        data_string += str(quat_data.x) + "," 
                        data_string += str(quat_data.y) + ","
                        data_string += str(quat_data.z) + ","

                    if self.record_pres:
                        status = pozyx.getPressure_Pa(pres_data)
                        data_string += str(pres_data.value) + ","

                    if self.record_range:
                        data_string += " ,"*2*idx 
                        if status_range == pypozyx.POZYX_SUCCESS:
                            data_string += str(device_range.distance) + ","
                            data_string += str(device_range.RSS) + ","
                        else:
                            data_string += " ,"*2
                        data_string += " ,"*2*(len(device_list.data)- 1 - idx)

                    data_string += "\n"

        return data_string

    def record(self, duration):
        """
        Prints data to screen and saves to a file for a given duration.

        Args:
            duration: [float] Amount of time to stream data, in seconds.
        """

        # Discover any other pozyx devices within range
        if self.record_range:
            self.device_lists = self.findNeighbors()

        response = input("Ready to record data? (y/n)")
        if response == "y" or response == "yes":
            pass
        else :
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
        if self.record_range:
            self.device_lists = self.findNeighbors()

        response = input("Ready to stream data? (y/n)")
        if response == "y" or response == "yes":
            pass
        else :
            quit()

        print(self.getHeader())

        self.start_time = time_ns()
        while (time_ns() - self.start_time)*(10**-9) < duration:
            data_string = self.getData()
            # Write to file
            print(data_string[:-1])

if __name__ == "__main__":
    dc = PozyxDataCollector()
    dc.stream(30)       # To stream data to screen without saving to file
  #  dc.record(10)      # To stream data to screen and save to a file

    