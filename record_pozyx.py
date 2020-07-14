from pozyx_data_collection import PozyxDataCollector

dc = PozyxDataCollector()
dc.record_accel = True
dc.record_gyro = True
dc.record_mag = True
dc.record_pres = True
dc.record_range = False
dc.record_quat = False
dc.allow_self_ranging = False

dc.record(duration = 180)


