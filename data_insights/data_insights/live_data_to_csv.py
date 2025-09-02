import rclpy
from rclpy.node import Node

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, Pose2D
from pioneer_interfaces.msg import ClusterInfo

import array

from typing import Callable, Any, List

import pandas as pd

from collections import defaultdict

from pprint import pprint

from adaptive_navigation_utilities.pubsub import PubSubManager


DEBUG: bool = False


def flatten_msg(msg):
    # For simple types, just return the value
    # For complex msg, expand here recursively if needed
    if hasattr(msg, '__slots__'):
        d = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            if hasattr(val, '__slots__'):
                for subslot in val.__slots__:
                    d[f"{slot}_{subslot}"] = getattr(val, subslot)
            else:
                d[slot] = val
        return d
    else:
        return {"value": msg}

# Recursion/Loop
def extract_fields(msg, dict: dict, unique_slot_key: str = ""):
    
    # Base case
    # if the ROS message does not have a __slots__ type
    if not hasattr(msg, '__slots__'):

        return {}
    
    else:

        # For all the slots in the message
        for slot in msg.__slots__:

            # Get value
            val = getattr(msg, slot)

            # Base case
            # If the msg's slot's do not contain any
            # more subslots
            if not hasattr(val, '__slots__'):

                # Prepend slot with accumulated unique key
                _key = unique_slot_key + "." + slot

                # FOR DEBUG ONLY
                # print(f"slot: {slot}, type val: {type(val)}")

                # If data type is an array
                if slot == "_data" and isinstance(val, array.array):

                    # Create each key for each data
                    for i, v in enumerate(val):
                        
                        # Create enumerated key
                        enum_key = f"{_key}[{i}]"

                        # Add key
                        dict[enum_key] = v

                # Ignore this field as it is insignificant
                elif slot == "_check_fields": pass

                # Else, if not an array type
                else:
                    # Add value to dictionary
                    dict[_key] = val
            
            # Else, recursively call for 
            # extract_fields until base case
            # is reached
            else:

                # Append to key name for uniqueness from different slots
                _unique_slot_key: str = unique_slot_key + "." + \
                                       slot
                
                dict = extract_fields(val, dict, _unique_slot_key)
            
        return dict
        

class Recorder(Node):

    SAVE_TO_FILE: str = "save_to_file"
    PERIOD: str = "period"

    def __init__(self):

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])
    
        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='',
            parameters=[
                (Recorder.SAVE_TO_FILE, False),
                (Recorder.PERIOD, 0.05)
            ]
        )

        # Save parameters as attributes
        self.save_to_file: bool = self.get(Recorder.SAVE_TO_FILE)
        self.period: float = self.get(Recorder.PERIOD)

        # Create pubsub manager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Create cachce dictionary of logged objects
        self.cdict: dict = defaultdict(lambda: None)

        # Create publishers and subscribers
        self.create_publishers_and_subscribers()

        # Set clock
        # TODO: Option 1: Should expect a /timestamp node
        # TODO: Option 2: Check .stamp attribute for each message
        # TODO: Defualt:  Use current node's system clock (which is not the same as replay)
        self.use_system_clock = True

        # If no /stamp node, then manually create node
        if self.use_system_clock:
            self.cdict["stamp"] = None


        # Add aliases for logging
        self.debug: Callable = self.log().debug
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error

        # Data frame
        self.df: pd.DataFrame = pd.DataFrame(columns=self.columns)
        self.output_name: str= f"output_{self.stamp}"
        self.output_ext: str = ".csv"

        # Create timer
        self.create_timer(self.period, self.write_to_database)# lambda: pprint(self.cdict))

        # DEBUG
        # TODO: Get rid of this
        self.create_timer(2, lambda: pprint(self.cdict))

        # Callback function for set parameters
        self.add_on_set_parameters_callback(self.parameters_callback)
    

    # Update cache dictionary
    def update_cache_dictionary(self, msg, topic) -> None:

        # Update value in dictionary
        self.cdict[topic] = msg

        if self.use_system_clock:
            self.cdict["stamp"] = self.stamp

    def subscribe_to_all_topics(self) -> None:

        topics = self.get_topic_names_and_types()

        # For every topic
        for name, list_of_types in topics:

            # print the topic name and its types
            if DEBUG: print(name, list_of_types)

            # Check recursively for topics with multiple fields
            # TODO

            # Typecast the list of types from str into its actual type
            type = get_message(list_of_types[0]) 

            # If the topic does not already exist in the pubsub's list of
            # subscribers
            if name not in self.pubsub._subscribers[1]:

                # Create key in logged dictionary
                self.cdict[name] = None

                # create a subscription
                self.pubsub.create_subscription(
                    type,
                    name,
                    lambda msg, n=name: self.wrapper_extract_fields(msg, n),
                    10
                )


    def wrapper_extract_fields(self, msg, name) -> None:

        self.cdict = extract_fields(msg, self.cdict, name)

    def test_subscription(self) -> None:
        
        self.pubsub.create_subscription(
            Twist,
            '/ctrl/cmd_vel',
            lambda msg, name: self.wrapper_extract_fields(msg, '/ctrl/cmd_vel'),
            10
        )

        self.pubsub.create_subscription(
            Pose2D,
            '/p1/pose2D',
            lambda msg, name: self.wrapper_extract_fields(msg, '/p1/pose2D'),
            10
        ) 

        self.pubsub.create_subscription(
            ClusterInfo,
            '/cluster_info',
            lambda msg, name: self.wrapper_extract_fields(msg, '/cluster_info'),
            10
        ) 


    # Create all publishers and subscribers
    def create_publishers_and_subscribers(self):
        
        # Subscribe to all topics
        # TODO: Add argparse / rosparam to
        # enable this feature
        # self.subscribe_to_all_topics()

        # Subscribe to specific topic list
        self.test_subscription()


    # Timestamp
    @property
    def stamp(self) -> float:

        # Use ROS node built-in method for clock
        return self.get_clock().now().nanoseconds
    
    @property
    def columns(self) -> List[str]:

        # NOTE: The .strip() method removes leading and trailing
        # whitespace.
        # NOTE: The .replace(' ', '_') replaces inner spaces with
        # underscores
        cols = [s.strip().replace(' ', '_') for s in self.cdict.keys()]
        
        return cols


    # Write to a csv file
    def write_to_database(self) -> None:
            
        # Use current snapshot of cache dictionary to write next row
        self.df = pd.concat([self.df, pd.DataFrame([self.cdict])], ignore_index=True)

        if DEBUG:
            print(self.df)

    @property
    def output(self) -> str:

        # Combine output name and extension
        return self.output_name + self.output_ext
    
    @output.setter
    def output(self, name: str) -> None:

        # Split string based on file name
        _list = name.split('.')
        
        if len(_list) < 2: 
            print("invalid output name")
            return
        else:

            # Re-stitch list together except for last entry
            self.output_name = ".".join(_list[:-1])

            # Get extension from last entry
            self.output_ext = _list[-1]


    def write_to_file(self, output: str = None) -> None:

        # Set default output if no args given
        if output == None: output = self.output
        
        # Write to file depending on extension
        if self.output_ext == ".csv":
            self.df.to_csv(output, index=False)
        elif self.output_ext == ".xlsx":
            self.df.to_excel(output, index=False)
        

    # Add callback if there is any change in ROS parameters
    def parameters_callback(self, params):

        # For all parameters
        for p in params:
        
            # If the class has this attribute and
            # is the same as the parameter name
            if hasattr(self, p.name):

                # Set the attribute value
                self.info(f"{p.name}")
                setattr(self, p.name, p.value)

                self.info(f"Set {p.name} to {p.value}")

            else:

                # Create a variable and set it as such
                setattr(self, p.name, p.value)

            
        # For specific parameter checks
        if self.save_to_file:

            # Write to file
            self.write_to_file()

            # Reset attribute and parameter
            self.save_to_file = False
            _param = rclpy.parameter.Parameter(
                Recorder.SAVE_TO_FILE,
                rclpy.Parameter.Type.BOOL,
                False
            )
            self.set_parameters([_param])
            self.info(f"Set parameters for {Recorder.SAVE_TO_FILE}")

        return SetParametersResult(successful=True)
            


    # Crete alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    def log(self):
        return self.get_logger()

 

def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    recorder = Recorder()

    try:
        while rclpy.ok():
            rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()
