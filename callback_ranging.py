from pypozyx import get_first_pozyx_serial_port, PozyxSerial, DeviceRange, PozyxRegisters, Data, SingleRegister, \
    PozyxBitmasks, SensorData, NetworkID
from time import time_ns

pozyx = PozyxSerial(get_first_pozyx_serial_port())

destination_id = NetworkID(0x6f61)


class AsynchronousReader(object):
    def __init__(self):
        self.just_ranged = False

    def start_ranging(self):
        pozyx.clearInterruptStatus()

        address = PozyxRegisters.DO_RANGING
        params = destination_id
        data = Data([])
        params.load_hex_string()
        s = 'F,%0.2x,%s,%i\r' % (address, params.byte_data, data.byte_size + 1)
        #status = pozyx.useFunction(PozyxRegisters.DO_RANGING, destination_id, Data([]))
        pozyx.ser.write(s.encode())

        self.just_ranged = True

    def on_error(self):
        #print("Error occured during ranging")
        if self.just_ranged:
            _ = pozyx.ser.readline().decode()
            self.just_ranged = False
        self.start_ranging()
        

    def on_range(self):
        if self.just_ranged:
            _ = pozyx.ser.readline().decode()
            self.just_ranged = False
        device_range = DeviceRange()
        pozyx.getDeviceRangeInfo(destination_id, device_range)
        print("Got a range {}".format(device_range))
        self.start_ranging()

    def on_imu(self):
        if self.just_ranged:
            _ = pozyx.ser.readline().decode()
            self.just_ranged = False
        sensor_data = SensorData()
        pozyx.getAllSensorData(sensor_data)
        print("Got sensor data {}".format(str(time_ns()) + ", " + str(sensor_data)))

# start up the ranging
ar = AsynchronousReader()
ar.start_ranging()
interrupt_register = SingleRegister()
timeout_s = 1

while True:

    pozyx.waitForFlagSafe(PozyxBitmasks.INT_MASK_FUNC | PozyxBitmasks.INT_MASK_IMU| PozyxBitmasks.INT_MASK_ERR, timeout_s, interrupt_register)

    if (interrupt_register.value & PozyxBitmasks.INT_MASK_ERR) == PozyxBitmasks.INT_MASK_ERR:
        ar.on_error()
    if (interrupt_register.value & PozyxBitmasks.INT_MASK_IMU) == PozyxBitmasks.INT_MASK_IMU:
        ar.on_imu()
    if (interrupt_register.value & PozyxBitmasks.INT_MASK_FUNC) == PozyxBitmasks.INT_MASK_FUNC:
        ar.on_range()
