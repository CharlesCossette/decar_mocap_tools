#!/usr/bin/env python3
import pypozyx
from data_collector import DataSource, DataCollector
from time import time_ns, sleep
from datetime import datetime
"""
This code is currently applicable for a computer (such as a raspberry pi)
which has many pozyx devices connected to it through USB. This code will 
automatically detect all the pozyx devices connected to the computer, 
as well as any other pozyx devices that are active within UWB range.

This code can then be used to collect IMU data from the connected devices,
and range data between the connected devices and all other active pozyx 
devices in UWB range.
"""

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

class PozyxIMU(DataSource):
    
    def __init__(self,pozyx):
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
        #self.start_time = time_ns()
        #self.time_previous = time_ns()
        #self.current_time = self.time_previous
        #self.number_data_columns = 0
        self.pozyxs, self.ids = self.findPozyxSerial()
        self.device_lists = self.findNeighbors()
        print('Initalization complete.')

    def getHeader(self):
        header_fields = list()
        header_fields.append('Timestamp (ns)')
        for idx_pozyx, pozyx in enumerate(self.pozyxs):
            who_am_i = pypozyx.NetworkID()
            status = pozyx.getNetworkId(who_am_i)
            self.number_data_columns = 1
            # Generate the header string
            if self.record_accel:
                header_fields.append(str(hex(who_am_i.id)) + " Accel_x (mg)")
                header_fields.append(str(hex(who_am_i.id)) + " Accel_y (mg)")
                header_fields.append(str(hex(who_am_i.id)) + " Accel_z (mg)")
                self.number_data_columns += 3

            if self.record_gyro:
                header_fields.append(str(hex(who_am_i.id)) + " Gyro_x (deg/s)")
                header_fields.append(str(hex(who_am_i.id)) + " Gyro_y (deg/s)")
                header_fields.append(str(hex(who_am_i.id)) + " Gyro_z (deg/s)")
                self.number_data_columns += 3

            if self.record_mag:
                header_fields.append(str(hex(who_am_i.id)) + " Mag_x (uT)")
                header_fields.append(str(hex(who_am_i.id)) + " Mag_y (uT)")
                header_fields.append(str(hex(who_am_i.id)) + " Mag_z (uT)")
                self.number_data_columns += 3
                
            if self.record_quat:
                header_fields.append(str(hex(who_am_i.id)) + " Quat_w")
                header_fields.append(str(hex(who_am_i.id)) + " Quat_x")
                header_fields.append(str(hex(who_am_i.id)) + " Quat_y")
                header_fields.append(str(hex(who_am_i.id)) + " Quat_z")
                self.number_data_columns += 4

            if self.record_pres:
                header_fields.append(str(hex(who_am_i.id)) + " Pressure (Pa)")
                self.number_data_columns += 1

            if self.record_range:
                device_list = self.device_lists[idx_pozyx]
                for id in device_list.data:
                    header_fields.append(str(hex(who_am_i.id))\
                        + 'Range to ' + str(hex(id)) + ' (mm)')
                    header_fields.append(str(hex(who_am_i.id))\
                        + 'RSS to ' + str(hex(id)) + ' (dB)')
                    self.number_data_columns += 2

        return header_fields

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
        data_values = list()

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
                            # Try ranging a second time, sometimes this occurs 
                            # if multiple devices are used.
                            status_range = pozyx.doRanging(id, device_range)

                    # Create datastring
                    
                    data_values.append(time_ns())
                    data_values += [""]*self.number_data_columns*idx_pozyx
                    if self.record_accel:
                        status = pozyx.getAcceleration_mg(accel_data)
                        data_values.append(accel_data.x)
                        data_values.append(accel_data.y)
                        data_values.append(accel_data.z)

                    if self.record_gyro:
                        status = pozyx.getAngularVelocity_dps(gyro_data)
                        data_values.append(gyro_data.x)
                        data_values.append(gyro_data.y)
                        data_values.append(gyro_data.z)

                    if self.record_mag:
                        status = pozyx.getMagnetic_uT(mag_data)
                        data_values.append(mag_data.x)
                        data_values.append(mag_data.y)
                        data_values.append(mag_data.z)

                    if self.record_quat:
                        status = pozyx.getQuaternion(quat_data)
                        data_values.append(quat_data.w)
                        data_values.append(quat_data.x)
                        data_values.append(quat_data.y)
                        data_values.append(quat_data.z)

                    if self.record_pres:
                        status = pozyx.getPressure_Pa(pres_data)
                        data_values.append(pres_data.value)

                    if self.record_range:
                        data_values += [" "]*2*idx 
                        if status_range == pypozyx.POZYX_SUCCESS:
                            data_values.append(device_range.distance)
                            data_values.append(device_range.RSS)
                        else:
                            data_values += [" "]*2
                        data_values += [" "]*2*(len(device_list.data)- 1 - idx)

                    data_values += ["\n"]

        return data_values

class PozyxRange(DataSource):
    def __init__(self,pozyx_serial,allow_self_ranging = True)
        super.__init__()
        self.pozyx = pozyx_serial
        self.allow_self_ranging = allow_self_ranging
        self._neighbor_to_range = 0
        self.device_list = self.findNeighbors()

    def findNeighbors(self):
        """ 
        Automatically discovers any pozyx anchor or tag devices within UWB range.

        Returns:
            device_lists: [list] of [pypozyx.DeviceList] list containing the device IDs
        """
        device_lists = list()
        # Get ID of current pozyx device
        who_am_i = pypozyx.NetworkID()
        status = self.pozyx.getNetworkId(who_am_i)

        # Discover other pozyx devices
        status = self.pozyx.clearDevices()
        status = self.pozyx.doDiscoveryAll()
        device_list_size = pypozyx.SingleRegister()
        status = self.pozyx.getDeviceListSize(device_list_size)
        device_list = pypozyx.DeviceList(list_size = device_list_size.value)
        status = self.pozyx.getDeviceIds(device_list)

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

class PozyxPosition(DataSource):
    pass

if __name__ == "__main__":
    pozyx_source = PozyxIMU()
    pozyx_source
    dc = DataCollector(pozyx_source)
    dc.record(10)      # To stream data to screen and save to a file

    