from sense_hat import SenseHat
from time import time_ns, sleep
from datetime import datetime

class DataCollector(object):
    
    def __init__(self):
        self.record_accel = True
        self.record_gyro = True
        self.record_mag = True
        self.record_pres = True
        self.record_quat = True
        self.sense = SenseHat()
        self.sense.get_accelerometer_raw()

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
        header_data = ""
        if self.record_accel:
            header_data += "Accel_x (g), Accel_y (g), Accel_z (g),"
        if self.record_gyro:
            header_data += "Gyro_x (rad/s), Gyro_y (rad/s), Gyro_z (rad/s),"
        if self.record_mag:
            header_data += "Mag_x (uT), Mag_y (uT), Mag_z (uT),"
        if self.record_quat:
            header_data += "Roll (rad), Pitch (rad), Yaw(rad),"
        if self.record_pres:
            header_data += "Pressure (Pa),"
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

    def getData(self):
        
        data_string = ""
        
        timestamp = time_ns()
        accel_data = self.sense.get_accelerometer_raw()
        gyro_data = self.sense.get_gyroscope_raw()
        timestamp = 0.5*(timestamp + time_ns())
        mag_data = self.sense.get_compass_raw()
        att_data = self.sense.get_orientation_radians()
        pres_data = self.sense.get_pressure()
        temp_data = self.sense.get_temperature()

        data_string += str(timestamp) + ","

        data_string += str(accel_data['x']) + ","
        data_string += str(accel_data['y']) + ","
        data_string += str(accel_data['z']) + ","

        data_string += str(gyro_data['x']) + ","
        data_string += str(gyro_data['y']) + ","
        data_string += str(gyro_data['z']) + ","

        data_string += str(mag_data['x']) + ","
        data_string += str(mag_data['y']) + ","
        data_string += str(mag_data['z']) + ","

        data_string += str(att_data['roll']) + ","
        data_string += str(att_data['pitch']) + ","
        data_string += str(att_data['yaw']) + ","

        data_string += str(pres_data)  + ","
        data_string += str(temp_data) + ","
        return data_string
