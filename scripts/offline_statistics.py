import pandas as pd
from typing import Union, LiteralString, List, Tuple
import numpy as np
import argparse

# Typing
FileName = LiteralString

# Boolean
VERBOSE = False
DEBUG = True


def parse_args():
	parser = argparse.ArgumentParser()

    # Add file
	parser.add_argument("--file", "-f",
                      default='tests/power_level_0_height_0.010/output.csv',
                      type=str, 
                      required=False, 
                      help="File path")
    

	return parser.parse_args()


class PandaDataFrameExtended:

    def __init__(
                self, 
                    data: Union[pd.DataFrame, FileName], 
                    ) -> None:
        

        # Create Panda Frame
        self.data = self.create_panda_dataframe(data)

        # Start and End indices
        self.start: int = 0
        self.end: int = self.data.size

        self.define_col_headers()    

    def define_col_headers(self) -> None:
        
        # For each column in the data frame
        for c in self.data.columns:

            # Create a class attribute and assign the column
            # NOTE: The .strip() method removes leading and trailing
            # whitespace.
            # NOTE: The .replace(' ', '_') replaces inner spaces with
            # underscores
            self.__setattr__(c.strip().replace(' ', '_'), self.data[c])

        # Prints all the keys
        if VERBOSE: print(self.__dict__.keys())  
        
    @staticmethod
    def create_panda_dataframe(data) -> pd.DataFrame:
        
        # Get datatype
        typ = type(data)

        # TODO: Check which type of string
        # TODO: Check if string is invalid
        if typ == str:
            return pd.read_csv(data)
        elif typ == pd.DataFrame:
            return data   

class RSSIData(PandaDataFrameExtended):

    def __init__(
                self, 
                data: Union[pd.DataFrame, FileName], 
                    ) -> None:
        
        # Call PandaDataFrameExtended
        super().__init__(data)

    
        # Extract RSSI data
    
    @property
    def max(self):
        return np.max(self.z)

    @property
    def min(self):
        return np.min(self.z)

    @property
    def range(self):
        return self.max - self.min
    
    @property
    def precision(self):

        min = np.inf

        for i in range(len(self.z) - 1):
            
            if abs(self.z[i + 1] - self.z[i]) < abs(min) and \
                abs(self.z[i + 1] - self.z[i]) > 0:

                min  = self.z[i+1] - self.z[i]

        return min
    
    @property
    def update_freq(self):
        return 1/self.averarge_update_duration

    @property
    def averarge_update_duration(self):
        # TODO This is not a good way; this method assumes updates happen
        #      when values are changing, which is not always the case
        
        # Tuple
        last_z_with_associated_timestamp: Tuple[int, float] = (np.nan, self.timestamp[0])

        list_of_diff_timestamps: List[float] = list()

        for i in range(len(self.y)):

            last_z = last_z_with_associated_timestamp[0]
            last_time = last_z_with_associated_timestamp[1]

            if self.z[i] != last_z:
                
                list_of_diff_timestamps.append(
                    self.timestamp[i] - last_time
                )
            
                last_z_with_associated_timestamp = (self.z[i], self.timestamp[i]) 

        # Take average time difference
        # NOTE: Remove the first and last entries using index slicing [1:-1]
        avg_update_duration = np.average(list_of_diff_timestamps[1:-1])

        return avg_update_duration


def example():

    # Parge arguments
    args = parse_args()  
    print(f"Extracting from {args.file}")
    path = args.file

    # Create class object from CSV file, and
    # Get RSSI peroperties
    data: RSSIData = RSSIData(path)

    # Print properties
    print(f"Max: {data.max}")
    print(f"Min: {data.min}")
    print(f"Precision: {data.precision}")
    print(f"Average update duration {data.averarge_update_duration}")
    print(f"Update Frequency: {data.update_freq}")

def main():
    example()

if __name__ == "__main__":
    main()