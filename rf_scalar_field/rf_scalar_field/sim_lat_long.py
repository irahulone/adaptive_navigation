import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from typing import Any, Union, Callable

from random import random, randrange

from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace


# Global variables
Numeric = Union[int, float]
DEBUG: bool = True


class SimLatLong(Node):
    """  
    Simulates Lat/Long Topic Data and publishes it
    """

    FREQ: str = 'freq'
    PUB_TOPIC: str = 'pub_topic'

    
    def __init__(self) -> None:

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])

        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())
    
        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='',
            parameters=[
                (SimLatLong.FREQ, 5.0),
                (SimLatLong.PUB_TOPIC, "message")
            ]
        )

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.freq: int = self.get(SimLatLong.FREQ)

        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error

        self.info(f"Namespace: {self.ns.namespace}")
        
        # Check if sim
        self.info(f"Is in sim mode: {self.ns.is_simulation}")

        # Create publish topics
        self.create_publish_topics()

        # If simulation, use the node
        if self.ns.is_simulation:

            self.timer = self.create_timer(self.period, self.send_fake_gps_data)


    @property
    def period(self) -> Numeric:
        return 1/self.freq
    
    # Create publish topics
    def create_publish_topics(self) -> None:

        # Create publish topic of message
        self.pubsub.create_publisher(
            NavSatFix,
            self.get(SimLatLong.PUB_TOPIC),
            10
        )


    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    
    def send_fake_gps_data(self) -> None:

        # Simple algorithm for fake GPS data
        # TODO: Fix hardcoded values
        lat: float = -10.0 + random()
        long: float = -20.0 + random()

        # Create ROS msg of GPS
        msg: NavSatFix = NavSatFix(
            latitude = lat,
            longitude = long
        )

        # Log info
        self.info(f"Latitude: {lat}, Longitude: {long}")

        # Publish message
        self.pubsub.publish(
            self.get(SimLatLong.PUB_TOPIC),
            msg
        )

def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    sim_lat_long = SimLatLong()

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(sim_lat_long)


if __name__ == '__main__':
    main()