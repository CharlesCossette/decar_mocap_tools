from pozyx_data_collection import PozyxDataCollector

dc = PozyxDataCollector()

dc.record_accel = False        # Set to True to record/stream accelerometer data
dc.record_gyro = False           # Set to True to record/stream gyroscope data
dc.record_mag = False            # Set to True to record/stream magnetometer data
dc.record_pres = False           # Set to True to record/stream pressure data
dc.record_range = True         # Set to True to record/stream range (distance) data
dc.record_quat = False          # Set to True to record/stream quaternion data
dc.allow_self_ranging = False  # Set to True to get distance measurements between pozyx devices connected to the same computer.

dc.stream(duration = 30)        # Set duration in seconds

