import pypozyx
from data_collector import DataCollector
from pozyx_data_sources import PozyxPositionSource, findPozyxSerial

pozyxs, ids = findPozyxSerial()
anchors =  [pypozyx.DeviceCoordinates(0x6f4a, 1, pypozyx.Coordinates(2594, -2110, 1845)),
            pypozyx.DeviceCoordinates(0x6f58, 1, pypozyx.Coordinates(-75, 2116, 149)),
            pypozyx.DeviceCoordinates(0x6f5f, 1, pypozyx.Coordinates(4731, 2149, 2120)),
            pypozyx.DeviceCoordinates(0x6f60, 1, pypozyx.Coordinates(-4232, 460, 2400)),
            pypozyx.DeviceCoordinates(0x6f61, 1, pypozyx.Coordinates(-505, -2080, 474))]

pozyx_sources = list()
for pozyx in pozyxs:
    pozyx_sources.append(PozyxPositionSource(pozyx,anchors))

dc = DataCollector(*pozyx_sources)
dc.record(300, name = 'range')