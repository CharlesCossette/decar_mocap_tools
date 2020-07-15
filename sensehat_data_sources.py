from sense_hat import SenseHat
from time import time_ns
from data_collector import DataSource, DataCollector

# IMU Settings file stored in /home/pi/.config/sense_hat/RTIMULib.ini

class SenseHatSource(DataSource):
    """ 
    Create a SenseHat source object. Specify in argument data to NOT collect.
    For example,

    >>> source = SenseHatSource(mag = False)
    >>> source = SenseHatSource(mag = False, temp = False)
    
    Everything else (accel,gyro,mag,pres,attitude,temp) will be collected.
    """
    def __init__(self, accel = True, gyro = True, mag = True, pres = True,
                    attitude= False, temp = True):
        
        self.record_accel = accel
        self.record_gyro = gyro
        self.record_mag = mag
        self.record_pres = pres
        self.record_att = attitude
        self.record_temp = temp
        self.id = 'SenseHat'
        self.sense = SenseHat()
        self.sense._init_imu()
        self.sense._imu_poll_interval = 0

    def getHeader(self):
        """
        Creates the header for the SenseHat data source.
        """

        header_fields = list()
        header_fields.append('Timestamp (ns)')

        # Generate the header string
        if self.record_accel:
            header_fields.append(str((self.id)) + " Accel_x (g)")
            header_fields.append(str((self.id)) + " Accel_y (g)")
            header_fields.append(str((self.id)) + " Accel_z (g)")

        if self.record_gyro:
            header_fields.append(str((self.id)) + " Gyro_x (rad/s)")
            header_fields.append(str((self.id)) + " Gyro_y (rad/s)")
            header_fields.append(str((self.id)) + " Gyro_z (rad/s)")

        if self.record_mag:
            header_fields.append(str((self.id)) + " Mag_x (uT)")
            header_fields.append(str((self.id)) + " Mag_y (uT)")
            header_fields.append(str((self.id)) + " Mag_z (uT)")
            
        if self.record_att:
            header_fields.append(str((self.id)) + " Roll (rad)")
            header_fields.append(str((self.id)) + " Pitch (rad)")
            header_fields.append(str((self.id)) + " Yaw (rad)")

        if self.record_pres:
            header_fields.append(str((self.id)) + " Pressure (mbar)")

        if self.record_temp:
            header_fields.append(str((self.id)) + " Temperture (C)")

        return header_fields

    def getData(self):
        """
        Reads the desired data from the SenseHat and returns it as a list.
        """
        data_values = list()     
        timestamp = time_ns()
        if self.sense._imu.IMURead():
            data = self.sense._imu.getIMUData()
            accel_data = data['accel']
            gyro_data = data['gyro']
            mag_data = data['compass']
            timestamp = 0.5*(timestamp + time_ns())             

            data_values.append(timestamp)

            if self.record_accel:
                data_values.append(accel_data[0]) 
                data_values.append(accel_data[1]) 
                data_values.append(accel_data[2]) 

            if self.record_gyro:
                data_values.append(gyro_data[0]) 
                data_values.append(gyro_data[1]) 
                data_values.append(gyro_data[2]) 

            if self.record_mag:
                data_values.append(mag_data[0]) 
                data_values.append(mag_data[1]) 
                data_values.append(mag_data[2]) 
            
            if self.record_att:
                data_values += ['']*3

            if self.record_pres:
                pres_data = self.sense.get_pressure()
                data_values.append(pres_data)

            if self.record_temp:
                temp_data = self.sense.get_temperature()
                data_values.append(temp_data)
    
        return data_values

if __name__ == "__main__":
    shs = SenseHatSource()
    dc = DataCollector(shs)
    dc.stream(10)