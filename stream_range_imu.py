from pozyx_data_collection import DataCollector

dc = DataCollector()
dc.record_accel = True
dc.record_gyro = True
dc.record_mag = True
dc.record_pres = True

dc.stream(30)