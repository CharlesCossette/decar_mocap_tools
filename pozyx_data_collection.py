#!/usr/bin/env python3
import pypozyx
from time import time_ns, sleep
from datetime import datetime

class DataCollector(object):
    
    def __init__(self):
        super().__init__()
        self.start_time = time_ns()
        self.time_previous = time_ns()
        self.current_time = self.time_previous
        self.timeout_s = 0.1
        self.number_data_columns = 0

        self.record_accel = True
        self.record_gyro = True
        self.record_mag = True
        self.record_pres = True
        self.record_quat = True
        self.record_range = True

        self.device_list = pypozyx.DeviceList(list_size = 1)
        
        self.pozyxs = self.findPozyxSerial()

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

        header_device = ","
        header_data = "Timestamp (ns),"
        for pozyx in self.pozyxs:
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
                for id in self.device_list.data:
                    header_device += (str(hex(who_am_i.id)) + ",")*2
                    header_data += 'Range to ' + str(hex(id)) + ' (mm),'
                    header_data += 'RSS to ' + str(hex(id)) + ' (dB),'
                    self.number_data_columns += 2
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

        # Discover any other pozyx devices within range
        if self.record_range:
            self.device_list = self.findNeighbors()


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
        if self.record_range:
            self.device_list = self.findNeighbors()
        response = input("Ready to stream data? (y/n)")
        if response == "n" or response == "no":
            quit()

        self.start_time = time_ns()
        while (time_ns() - self.start_time)*(10**-9) < duration:
            data_string = self.getData()
            # Write to file
            print(data_string[:-1])

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
            # If taking measurements, dont wait at all, as doRanging takes lots of time.
            if not self.record_range:
                interrupt_register = pypozyx.SingleRegister()
                pozyx.waitForFlagSafe(pypozyx.PozyxBitmasks.INT_MASK_IMU, self.timeout_s, interrupt_register)

            
            for idx, id in enumerate(self.device_list.data):

                # Get the data
                if self.record_range:
                    status_range = pozyx.doRanging(id, device_range)
            
                status = pozyx.getAcceleration_mg(accel_data)
                status = pozyx.getAngularVelocity_dps(gyro_data)
                status = pozyx.getMagnetic_uT(mag_data)
                status = pozyx.getNormalizedQuaternion(quat_data)
                status = pozyx.getPressure_Pa(pres_data)

                # Create datastring
                data_string += str(time_ns() - self.start_time) + ","
                data_string += ","*self.number_data_columns*idx_pozyx
                if self.record_accel:
                    data_string += str(accel_data.x) + "," 
                    data_string += str(accel_data.y) + "," 
                    data_string += str(accel_data.z) + ","
                if self.record_gyro:
                    data_string += str(gyro_data.x) + "," 
                    data_string += str(gyro_data.y) + "," 
                    data_string += str(gyro_data.z) + ","
                if self.record_mag:
                    data_string += str(mag_data.x) + "," 
                    data_string += str(mag_data.y) + "," 
                    data_string += str(mag_data.z) + ","
                if self.record_quat:
                    data_string += str(quat_data.w) + "," 
                    data_string += str(quat_data.x) + "," 
                    data_string += str(quat_data.y) + ","
                    data_string += str(quat_data.z) + ","
                if self.record_pres:
                    data_string += str(pres_data.value) + ","
                if self.record_range:
                    data_string += " ,"*2*idx 
                    if status_range == pypozyx.POZYX_SUCCESS:
                        data_string += str(device_range.distance) + ","
                        data_string += str(device_range.RSS) + ","
                    else:
                        data_string += " ,"*2
                    data_string += " ,"*2*(len(self.device_list.data)- 1 - idx)

                data_string += "\n"

        return data_string

if __name__ == "__main__":
    dc = DataCollector()
    dc.record_quat = False
    dc.record_range = False
    dc.stream(30)
    #device_list = dc.findNeighbors()

    