from data_collector import DataCollector
from sensehat_data_sources import SenseHatSource

imu_source = SenseHatSource()

dc = DataCollector(imu_source)
dc.record(300, name = 'imu')