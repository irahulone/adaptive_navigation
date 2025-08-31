import rclpy
from rclpy.node import Node

from typing import Callable, Any, Union, Set

from math import nan
from collections import defaultdict

import csv

import os

from adaptive_navigation_interfaces.msg import Contour
from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace

# Global variables
DEBUG: bool = False

Numeric = Union[int, float]

class CsvReader(Node):

    FILE_NAME: str = 'fiename'
    CONTOUR_SUB_TOPIC: str = 'contour_sub_topic'
    ROBOT_ID: str = "robot_id"

    def __init__(self):


        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "")

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__("_".join([robot_id, __name__.split('.')[-1]]))

        
        # Pubsub
        self.pubsub: PubSubManager = PubSubManager(self)

        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())

        # Attributes
        self.x: float = nan
        self.y: float = nan
        self.z: int = 0 


        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='', # TODO: Include parameters here??
            parameters=[
                (CsvReader.ROBOT_ID, robot_id)
                (CsvReader.FILE_NAME, "output.csv"),
                (CsvReader.CONTOUR_SUB_TOPIC, "contour")
            ]
        )
        
        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
        self.debug: Callable = self.log().debug

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.csv_filename = self.get(CsvReader.FILE_NAME)

        # Create CSV file and write header after obtaining data
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = None

        # Create the columns
        self.init_csv_columns()

        # Create internal memory of latest row
        self.latest_entry = defaultdict(lambda: None)


        self.info(f"Namespace: {self.ns.namespace}")
        
        # Check if sim
        self.info(f"Is in sim mode: {self.ns.is_simulation}")

        # Create topics to subscribe to
        self.subscribe_to_topics()

        # Create topics to publish to
        self.create_publish_topics()
    
    # TODO: Include as utility function
    @staticmethod
    def get_msg_timestamp(msg) -> float:

        # Extract second
        sec: int = msg.header.stamp.sec    

        # Extract nano second 
        nanosec:int = msg.header.stamp.nanosec

        # Return timestamp
        return (sec + nanosec * 1e-9)
    
    # Subscribes to all topics
    def subscribe_to_topics(self) -> None:

        self.subscription = self.create_subscription(
            Contour,
            'contour',
            self.update_and_write_row,
            10
        )

    # Creates all publisher topics
    def create_publish_topics(self) -> None:
        pass
    
    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    def init_csv_columns(self) -> None:

        # Add fields to set of columns
        # TODO: Remove hardcoded str
        self.csv_columns: Set[str] = set(["timestamp", "x", "y", "z"])
    
    # Subscriber callback
    def update_and_write_row(self, msg) -> None:

        self.get_logger().info(f"X:{msg.x}, Y:{msg.y}, Z:{msg.z}")
        
        # Updates latest entry
        # TODO: Remove hardcoded str
        self.latest_entry["timestamp"] = self.get_msg_timestamp(msg)
        self.latest_entry["x"] = msg.x
        self.latest_entry["y"] = msg.y
        self.latest_entry["z"] = msg.z

        self.write_row()


    def write_row(self):

        # If empty
        if self.csv_writer is None:

            # Initialize CSV writer with header sorted alphabetically
            self.csv_writer = csv.DictWriter(self.csv_file,
                                              fieldnames=self.csv_columns)
            self.csv_writer.writeheader()

        # Prepare row: fill with empty strings if field missing
        row = {field: '' for field in self.csv_writer.fieldnames}
        # Update the row
        row.update(self.latest_entry)
        # Write the row
        self.csv_writer.writerow(row)




def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    csv_reader = CsvReader()

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(csv_reader)


if __name__ == "main":
    main()
