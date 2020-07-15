
from time import time_ns
from datetime import datetime

class DataCollector(object):
    def __init__(self, *args):
        self.data_sources = list()
        for data_source in args:
            self.data_sources.append(data_source)

        self._column_counter = -1

    def getAllHeaders(self):
        header_fields = list()
        header_fields.append("Timestamp")
        self._column_counter = 0
        for data_source in self.data_sources:
            source_headers = data_source.getHeader()
            header_fields += source_headers
            data_source._number_of_columns = len(source_headers)
            data_source._column_numbers = list(range(self._column_counter + 1,
                self._column_counter + len(source_headers) + 1))
            self._column_counter += len(source_headers)
        return header_fields
    
    def getAllData(self):
        data_values = list()


        


        

class DataSource(object):
    def __init__(self):
        self._column_numbers = 0
        self._number_of_columns = 1
        
    def getHeader(self):
        """ 
        This is a virtual function.

        This function should return the headers for each column as a list of
        strings.
        """
        
        raise NotImplementedError(
            "You need to implement a getHeader() function in your data source.")

    def getData(self):
        """
        This is a virtual function.

        This function should return a list of data values, as strings.
        """

        raise NotImplementedError(
            "You need to implement a getData() function in your data source.")