from sense_hat import SenseHat
from time import time_ns, sleep
from datetime import datetime

# IMU Settings file stored in /home/pi/.config/sense_hat/RTIMULib.ini

class DataCollector(object):
    
    def __init__(self):
        self.record_accel = True
        self.record_gyro = True
        self.record_mag = True
        self.record_pres = True
        self.record_att = True
        self.record_temp = True
        self.sense = SenseHat()
        self.sense._init_imu()
        self.sense._imu_poll_interval = 0
        self.counter = 0
    def createDataFile(self,filename = None):
        """
        Creates a new csv file with the following default file name,
        
        YYYY_MM_DD_HH_MM_SS_sensehat.csv

        unless overwritten by the user. Also creates the header.

        Args:
            filename: [string](optional) custom filename
        Returns:
            file: file object
        """

        # Generate the filename string
        if filename is None:
            now = datetime.now()
            filename = now.strftime("%Y_%m_%d_%H_%M_%S")
            filename = filename + "_sensehat.csv"

        # Generate the header string 
        header_data = "Timestamp (ns),"
        if self.record_accel:
            header_data += "Accel_x (g), Accel_y (g), Accel_z (g),"
        if self.record_gyro:
            header_data += "Gyro_x (rad/s), Gyro_y (rad/s), Gyro_z (rad/s),"
        if self.record_mag:
            header_data += "Mag_x (uT), Mag_y (uT), Mag_z (uT),"
        if self.record_att:
            header_data += "Roll (rad), Pitch (rad), Yaw(rad),"
        if self.record_pres:
            header_data += "Pressure (mbar),"
        if self.record_temp:
            header_data += "Temperature (C),"
        header_data += "\n"
        # Create the file and insert the header.
        file = open(filename,"a")
        file.truncate(0)
        file.write(header_data)
        print(header_data[:-1])
        return file

    def record(self, duration):
        """
        Prints data to screen and saves to a file for a given duration.

        Args:
            duration: [float] Amount of time to stream data, in seconds.
        """


        response = input("Ready to stream data? (y/n)")
        if response == "n" or response == "no":
            quit()

        file = self.createDataFile()

        self.start_time = time_ns()

        while (time_ns() - self.start_time)*(10**-9) < duration:

            data_string = self.getData()

            # Write to file
            print(data_string[:-1])
            file.write(data_string)
               
        print(self.counter)
        # Close file
        file.flush()
        file.close()

    def stream(self, duration):
        """
        Prints data to screen for a given duration.

        Args:
            duration: [float] Amount of time to stream data, in seconds.
        """

        response = input("Ready to stream data? (y/n)")
        if response == "n" or response == "no":
            quit()

        self.start_time = time_ns()
        while (time_ns() - self.start_time)*(10**-9) < duration:
            data_string = self.getData()
            # Write to file
            print(data_string[:-1])
        print(self.counter)

    def getData(self):
        
        data_string = ""
        
        timestamp = time_ns()
        if self.sense._imu.IMURead():
            data = self.sense._imu.getIMUData()
            accel_data = data['accel']
            gyro_data = data['gyro']
            timestamp = 0.5*(timestamp + time_ns())
            

            """
            mag_data = self.sense.get_compass_raw()
            att_data = self.sense.get_orientation_radians()
            pres_data = self.sense.get_pressure()
            temp_data = self.sense.get_temperature()
            """
            data_string += str(timestamp) + ","

            data_string += str(accel_data[0]) + ","
            data_string += str(accel_data[1]) + ","
            data_string += str(accel_data[2]) + ","

            data_string += str(gyro_data[0]) + ","
            data_string += str(gyro_data[1]) + ","
            data_string += str(gyro_data[2]) + ","

            """
            data_string += str(mag_data['x']) + ","
            data_string += str(mag_data['y']) + ","
            data_string += str(mag_data['z']) + ","

            data_string += str(att_data['roll']) + ","
            data_string += str(att_data['pitch']) + ","
            data_string += str(att_data['yaw']) + ","

            data_string += str(pres_data)  + ","
            data_string += str(temp_data) + ","
            """
            data_string += "\n"
            self.counter += 1
        return data_string

if __name__ == "__main__":
    dc = DataCollector()
    dc.stream(30)