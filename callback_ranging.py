from pypozyx import get_first_pozyx_serial_port, PozyxSerial, DeviceRange, PozyxRegisters, Data, SingleRegister, \
    PozyxBitmasks, SensorData, NetworkID
from time import time_ns

pozyx = PozyxSerial(get_first_pozyx_serial_port())

destination_id = NetworkID(0x6f61)

def start_ranging():
    pozyx.clearInterruptStatus()
    status = pozyx.useFunction(PozyxRegisters.DO_RANGING, destination_id, Data([]))

def on_error():
    print("Error occured during ranging")
    start_ranging()

def on_range():
    device_range = DeviceRange()
    pozyx.getDeviceRangeInfo(destination_id, device_range)
    print("Got a range {}".format(device_range))
    start_ranging()

def on_imu():
    sensor_data = SensorData()
    pozyx.getAllSensorData(sensor_data)
    print("Got sensor data {}".format(str(time_ns()) + ", " + str(sensor_data)))

# start up the ranging
start_ranging()
interrupt_register = SingleRegister()
timeout_s = 1

while True:

    pozyx.waitForFlagSafe(PozyxBitmasks.INT_MASK_FUNC | PozyxBitmasks.INT_MASK_IMU | PozyxBitmasks.INT_MASK_ERR, timeout_s, interrupt_register)

    if (interrupt_register.value & PozyxBitmasks.INT_MASK_ERR) == PozyxBitmasks.INT_MASK_ERR:
        on_error()
    if (interrupt_register.value & PozyxBitmasks.INT_MASK_IMU) == PozyxBitmasks.INT_MASK_IMU:
        on_imu()
    if (interrupt_register.value & PozyxBitmasks.INT_MASK_FUNC) == PozyxBitmasks.INT_MASK_FUNC:
        on_range()
