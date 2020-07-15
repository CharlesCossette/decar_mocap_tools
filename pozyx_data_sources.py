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

def findPozyxSerial():
    """
    Automatically checks this computer's serial ports for any pozyx 
    devices connected through USB.

    Returns:
        pozyxs: [list] of [pypozyx.PozyxSerial] containing one
            PozyxSerial() object per connected pozyx device.
        pozyx_ids: [list] of IDs corresponding to the above PozyxSerial() list
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
        print("Pozyx device(s) detected on serial port(s): "\
             + str(pozyx_devices)+ ".")
    
    # Initialize connection to serial port.
    pozyx_serials = list()
    pozyx_ids = list()
    for serial_port in pozyx_devices:
        pozyx = pypozyx.PozyxSerial(serial_port)
        who_am_i = pypozyx.NetworkID()
        status = pozyx.getNetworkId(who_am_i)
        if status == pypozyx.POZYX_FAILURE:
            print("ERROR: Failed to obtain device ID.")

        pozyx_serials.append(pozyx)
        pozyx_ids.append(who_am_i.id)
        
    return pozyx_serials, pozyx_ids

class PozyxImuSource(DataSource):
    """
    Create a Pozyx IMU source object which can read and return sensor from the
    pozyx device. Certain readings can be excluded using, as an example,

    source = PozyxImuSource(mag = False)
    source = PozyxImuSource(mag = False, pres = False)
    source = PozyxImuSource(gyro = False)

    This class does NOT record range measurements. See PozyxRangeSource()
    """
    def __init__(self, pozyx, accel = True, gyro = True, mag = True,
        pres = True, quat = False):
        super().__init__()

        # Settings
        self.record_accel = accel
        self.record_gyro = gyro
        self.record_mag = mag
        self.record_pres = pres
        self.record_quat = quat

        # Internal variables
        self.pozyx = pozyx
        who_am_i = pypozyx.NetworkID()
        status = self.pozyx.getNetworkId(who_am_i)
        self.id = who_am_i.id
        print('Initalization complete.')

    def getHeader(self):
        """ 
        Generates the header for the Pozyx IMU Source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns)')
        self.number_data_columns = 1
        # Generate the header string
        if self.record_accel:
            header_fields.append(str(hex(self.id)) + " Accel_x (mg)")
            header_fields.append(str(hex(self.id)) + " Accel_y (mg)")
            header_fields.append(str(hex(self.id)) + " Accel_z (mg)")
            self.number_data_columns += 3

        if self.record_gyro:
            header_fields.append(str(hex(self.id)) + " Gyro_x (deg/s)")
            header_fields.append(str(hex(self.id)) + " Gyro_y (deg/s)")
            header_fields.append(str(hex(self.id)) + " Gyro_z (deg/s)")
            self.number_data_columns += 3

        if self.record_mag:
            header_fields.append(str(hex(self.id)) + " Mag_x (uT)")
            header_fields.append(str(hex(self.id)) + " Mag_y (uT)")
            header_fields.append(str(hex(self.id)) + " Mag_z (uT)")
            self.number_data_columns += 3
            
        if self.record_quat:
            header_fields.append(str(hex(self.id)) + " Quat_w")
            header_fields.append(str(hex(self.id)) + " Quat_x")
            header_fields.append(str(hex(self.id)) + " Quat_y")
            header_fields.append(str(hex(self.id)) + " Quat_z")
            self.number_data_columns += 4

        if self.record_pres:
            header_fields.append(str(hex(self.id)) + " Pressure (Pa)")
            self.number_data_columns += 1

        return header_fields

    def getData(self):
        """ 
        Reads the Pozyx IMU/mag/barometer and returns the data as a list.
        """

        # Containers for storing the data 
        accel_data = pypozyx.Acceleration()
        gyro_data = pypozyx.AngularVelocity()
        mag_data = pypozyx.Magnetic()
        quat_data = pypozyx.Quaternion()
        pres_data = pypozyx.Pressure()
        data_values = list()

        
        interrupt_register = pypozyx.SingleRegister()
        self.pozyx.waitForFlagSafe(pypozyx.PozyxBitmasks.INT_MASK_IMU, 0.1, interrupt_register)

        # Create datastring
        data_values.append(time_ns())
        if self.record_accel:
            status = self.pozyx.getAcceleration_mg(accel_data)
            data_values.append(accel_data.x)
            data_values.append(accel_data.y)
            data_values.append(accel_data.z)

        if self.record_gyro:
            status = self.pozyx.getAngularVelocity_dps(gyro_data)
            data_values.append(gyro_data.x)
            data_values.append(gyro_data.y)
            data_values.append(gyro_data.z)

        if self.record_mag:
            status = self.pozyx.getMagnetic_uT(mag_data)
            data_values.append(mag_data.x)
            data_values.append(mag_data.y)
            data_values.append(mag_data.z)

        if self.record_quat:
            status = self.pozyx.getQuaternion(quat_data)
            data_values.append(quat_data.w)
            data_values.append(quat_data.x)
            data_values.append(quat_data.y)
            data_values.append(quat_data.z)

        if self.record_pres:
            status = self.pozyx.getPressure_Pa(pres_data)
            data_values.append(pres_data.value)

        return data_values

class PozyxRangeSource(DataSource):
    """
    Create a Pozyx Range source object to do UWB ranging with other pozyx 
    devices. Will automatically detect other pozyx devices within UWB range.

    Args:
        pozyx: [PozyxSerial] object. See findPozyxSerial()

        exclude_ids: [list] of device network ids to exclude from the ranging 
            process.

        allow_self_ranging: [bool] toggle to include the above exclude_ids
    """
    def __init__(self, pozyx_serial,  exclude_ids = [], allow_self_ranging = True):
        super().__init__()
        self.pozyx = pozyx_serial
        self.allow_self_ranging = allow_self_ranging
        self.exclude_ids = exclude_ids

        who_am_i = pypozyx.NetworkID()
        status = self.pozyx.getNetworkId(who_am_i)
        self.id = who_am_i.id

        self.device_list = self.findNeighbors()
        self._neighbor_to_range = 0
        self._number_of_neighbors = len(self.device_list.data)
        

    def findNeighbors(self):
        """ 
        Automatically discovers any pozyx anchor or tag devices within UWB range.

        Returns:
            device_lists: [pypozyx.DeviceList] list containing the device IDs
        """
        
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

        # Print device list 
        id_string = "Device " + str(hex(who_am_i.id))+" has discovered the following other devices: "
        for id in device_list.data:
            id_string += str(hex(id)) + ", "
            
        print(id_string)

        if not self.allow_self_ranging:
            print("However, self-ranging is currently deactivated. " \
                + "No range measurements between devices connected to the same computer ")
        return device_list
        
    def getHeader(self):
        """
        Creates the header for the Pozyx Range source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns)')
        for id in self.device_list.data:
            header_fields.append(str(hex(self.id))\
                + ' Range to ' + str(hex(id)) + ' (mm)')
            header_fields.append(str(hex(self.id))\
                + ' RSS to ' + str(hex(id)) + ' (dB)')
        return header_fields

    def getData(self):
        """
        Performances ranging with one of its neighbors. A different neighbor is 
        selected on each of these function calls. Returns range and RSS data.
        """

        # Perform the rangin, exclude self-ranging if user has chosen this.
        device_range = pypozyx.DeviceRange()
        status_range = pypozyx.POZYX_FAILURE
        id = self.device_list.data[self._neighbor_to_range]
        if id in self.exclude_ids and not self.allow_self_ranging:
            pass
        else:
            status_range = self.pozyx.doRanging(id, device_range)
            if status_range == pypozyx.POZYX_FAILURE:
                # Try ranging a second time, sometimes this occurs 
                # if multiple devices are used.
                status_range = self.pozyx.doRanging(id, device_range)

        # Put data in list.
        data_values = list()
        data_values.append(time_ns())
        data_values += [" "]*2*self._neighbor_to_range
        if status_range == pypozyx.POZYX_SUCCESS:
            data_values.append(device_range.distance)
            data_values.append(device_range.RSS)
        else:
            data_values += [" "]*2
        data_values += [" "]*2*(len(self.device_list.data)- 1 - self._neighbor_to_range)
        
        # New neighbor for next function call.
        if self._neighbor_to_range == (self._number_of_neighbors - 1):
            self._neighbor_to_range = 0
        else:
            self._neighbor_to_range += 1

        return data_values

class PozyxPositionSource(DataSource):
    def __init__(self,pozyx,anchors):
        super().__init__()

        # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        self.algorithm = pypozyx.PozyxConstants.POSITIONING_ALGORITHM_TRACKING    
        self.dimension = pypozyx.PozyxConstants.DIMENSION_3D

        self.pozyx = pozyx
        who_am_i = pypozyx.NetworkID()
        status = self.pozyx.getNetworkId(who_am_i)
        self.id = who_am_i.id
        self.setAnchorsManual(anchors)
        print('Initalization complete.')

    def getHeader(self):
        """ 
        Creates the header for the pozyx position source.
        """
        header_fields = list()
        header_fields.append('Timestamp (ns)')
        header_fields.append(str(hex(self.id)) + " Pos_x (mm)")
        header_fields.append(str(hex(self.id)) + " Pos_y (mm)")
        header_fields.append(str(hex(self.id)) + " Pos_z (mm)")
        return header_fields

    def setAnchorsManual(self,anchors):
        """
        Adds the manually measured anchors to the Pozyx's device list 
        one for one.
        """
        
        status = self.pozyx.clearDevices()
        for anchor in anchors:
            status &= self.pozyx.addDevice(anchor)
        if len(anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(pypozyx.PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors))

        return status

    def getData(self):
        """
        Performs the built-in positioning algorithm on the pozyx device and
        returns the data.
        """
        position = pypozyx.Coordinates()
        data_values = list()
        
        status = self.pozyx.doPositioning(position, self.dimension, self.algorithm)
        data_values.append(str(time_ns()))
        if status is pypozyx.POZYX_SUCCESS:
            data_values.append(str(position.x))
            data_values.append(str(position.y))
            data_values.append(str(position.z))
        else:
            data_values += ['']*3

        return data_values


if __name__ == "__main__":
    pozyxs, ids = findPozyxSerial()
    anchors = [pypozyx.DeviceCoordinates(0x6f4a, 1, pypozyx.Coordinates(3272, -2122, 1831)),
               pypozyx.DeviceCoordinates(0x6f58, 1, pypozyx.Coordinates(-106, 2871, 1620)),
               pypozyx.DeviceCoordinates(0x6f5f, 1, pypozyx.Coordinates(5758, 1901, 2120)),
               pypozyx.DeviceCoordinates(0x6f60, 1, pypozyx.Coordinates(-3667, 176, 1820)),
               pypozyx.DeviceCoordinates(0x6f61, 1, pypozyx.Coordinates(150, -2091, 472))]


    #imu_source1 = PozyxImuSource(pozyxs[0])
    #range_source1 = PozyxRangeSource(pozyxs[0],exclude_ids=ids,allow_self_ranging=False)
    #imu_source2 = PozyxImuSource(pozyxs[1])
    #range_source2 = PozyxRangeSource(pozyxs[1],exclude_ids=ids,allow_self_ranging=False)
    #dc = DataCollector(imu_source1, range_source1, imu_source2, range_source2)
    position_source1 = PozyxPositionSource(pozyxs[0],anchors)
    position_source2 = PozyxPositionSource(pozyxs[1],anchors)
    dc = DataCollector(position_source1,position_source2)
    dc.stream(10)      # To stream data to screen and save to a file

    