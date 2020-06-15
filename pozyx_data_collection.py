#!/usr/bin/env python
import pypozyx
import sched
from time import *
from datetime import datetime
class DataCollector(object):
    
    def __init__(self):
        super().__init__()
        self.start_time = time_ns()
        self.time_previous = time_ns()
        self.current_time = self.time_previous

        self.record_accel = True
        self.record_gyro = True
        self.record_mag = True
        self.record_pres = True
        
        # Detect Pozyx device on serial ports, get serial port address.
        serial_port = pypozyx.get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()
        else:
            print("Pozyx device detected on serial port " + serial_port + ".")
        
        # Initialize connection to serial port.
        self.pozyx = pypozyx.PozyxSerial(serial_port)
        self.pozyx.printDeviceInfo()


    def createDataFile(self,filename = None):
        if filename is None:
            now = datetime.now()
            filename = now.strftime("%Y_%m_%d_%H_%M_%S")
            if self.record_accel:
                filename += "a"
            if self.record_gyro:
                filename += "g"
            if self.record_mag:
                filename += "m"
            if self.record_pres:
                filename += "p"
            filename = filename + ".csv"

        header = "Timestamp (ns),"
        if self.record_accel:
            header += "Accel_x (mg), Accel_y (mg), Accel_z (mg),"
        if self.record_gyro:
            header += "Gyro_x (deg/s), Gyro_y (deg/s), Gyro_z (deg/s),"
        if self.record_mag:
            header += "Mag_x (uT), Mag_y (uT), Mag_z (uT),"
        if self.record_pres:
            header += "Pressure (Pa),"
        header += "\n"
        
        file = open(filename,"a")
        file.truncate(0)
        file.write(header)
        return file

    def record(self, duration, frequency = 100,):

        file = self.createDataFile()

        self.start_time = time_ns()

        next_collect_time = time_ns()
        while (time_ns() - self.start_time)*(10**-9) < duration:
            
            #if abs(next_collect_time - time_ns()) < 10000:
                
            data_string = self.collectData()
            # Write to file
            print(data_string[:-1])
            file.write(data_string)
            next_collect_time += (1/frequency)*(10**9)

            sleep(1/frequency - 0.003)
                

        # Close file
        file.flush()
        file.close()


    def collectData(self):
        # Containers for storing the data 
        accel_data = pypozyx.Acceleration()
        gyro_data = pypozyx.AngularVelocity()
        mag_data = pypozyx.Magnetic()
        pres_data = pypozyx.Pressure()

        # Get the data
        status = self.pozyx.getAcceleration_mg(accel_data)
        status = self.pozyx.getAngularVelocity_dps(gyro_data)
        status = self.pozyx.getMagnetic_uT(mag_data)
        status = self.pozyx.getPressure_Pa(pres_data)

        # Create datastring
        data_string = str(time_ns() - self.start_time) + ","
        if self.record_accel:
            data_string += str(accel_data.x) + "," 
            data_string += str(accel_data.y) + "," 
            data_string += str(accel_data.z) + ","
        if self.record_gyro:
            data_string += str(gyro_data.x) + "," 
            data_string += str(gyro_data.y) + "," 
            data_string += str(gyro_data.z) + ","
        if self.record_mag:
            data_string += str(mag_data.x) + "," 
            data_string += str(mag_data.y) + "," 
            data_string += str(mag_data.z) + ","
        if self.record_pres:
            data_string += str(pres_data.value)
        data_string += "\n"

        return data_string



if __name__ == "__main__":
    dc = DataCollector()
    dc.record(3600*8)