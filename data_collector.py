
from time import time_ns
from datetime import datetime

class DataCollector(object):
    def __init__(self, *args):
        self.data_sources = list()
        for data_source in args:
            self.data_sources.append(data_source)

        self._column_counter = -1
        self._start_time = time_ns()

    def createDataFile(self, name = None, filename = None):
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
            if name is None:
                filename += ".csv"
            else:
                filename += "_" + name + ".csv"

        header_fields = self.getAllHeaders(self.data_sources)
        header_string = ",".join(map(str,header_fields))
        header_string += "\n"

        # Create the file, erase any contents, and insert the header.
        file = open(filename,"a")
        file.truncate(0)
        file.write(header_string)
        print(header_string[:-1])
        return file

    def getAllHeaders(self, data_source_list):
        header_fields = list()
        self._column_counter = 0
        for data_source in data_source_list:
            source_headers = data_source.getHeader()
            header_fields += source_headers
            data_source._number_of_columns = len(source_headers)
            data_source._column_numbers = list(range(self._column_counter,
                self._column_counter + len(source_headers)))
            self._column_counter += len(source_headers)
        return header_fields
    
    def getAllData(self, data_source_list):
        data_values = ['']*(self._column_counter - 1)

        for data_source in data_source_list:
            source_values = data_source.getData()
            if len(source_values) > 0:
                data_values[data_source._column_numbers[0]:
                            data_source._column_numbers[-1]] = source_values
        if data_values == ['']*(self._column_counter - 1):
            data_values = ''
        else:
            if data_values[-1] is not '\n':
                data_values += '\n'
        return data_values
        
    def record(self,duration, name = None, filename = None):
        response = input("Ready to record data? (y/n)")
        if response is not "y" and response is not "yes":
            quit()

        file = self.createDataFile(name = name, filename = filename)
        self._start_time = time_ns()

        while (time_ns() - self._start_time)*(10**-9) < duration:
            
            data_values = self.getAllData(self.data_sources)
            data_string = ",".join(map(str,data_values))

            # Write to file
            if data_string is not '':
                print(data_string[:-1])
                file.write(data_string)
               
        # Close file
        file.flush()
        file.close()
    
    def stream(self,duration, **kwargs):

        response = input("Ready to stream data? (y/n)")
        if response is not "y" and response is not "yes":
            quit()

        _ = self.getAllHeaders(self.data_sources)
        self._start_time = time_ns()

        while (time_ns() - self._start_time)*(10**-9) < duration:
            
            data_values = self.getAllData(self.data_sources)
            data_string = ",".join(map(str,data_values))

            # Print to screen
            if data_string is not '':
                print(data_string[:-1])
               


class DataSource(object):
    def __init__(self):
        self._column_numbers = -1
        self._number_of_columns = -1
        
    def getHeader(self):
        """ 
        This is a virtual function.

        This function should return the headers for each column as a list of 
        headers for each column.
        """
        
        raise NotImplementedError(
            "You need to implement a getHeader() function in your data source.")

    def getData(self):
        """
        This is a virtual function.

        This function should return a list of data values, for each column of 
        this data source.
        """

        raise NotImplementedError(
            "You need to implement a getData() function in your data source.")