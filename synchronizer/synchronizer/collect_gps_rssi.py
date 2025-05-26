import rclpy
from rclpy.node import Node

from typing import Callable, Any, Union

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16


from math import nan

from adaptive_navigation_interfaces.msg import Contour
from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace

# Global variables
DEBUG: bool = False

Numeric = Union[int, float]

class SyncPublisher(Node):

    X_NAME: str = "x"
    Y_NAME: str = "y"
    Z_NAME: str = "z"
    T_NAME: str = "t"
    # TODO: Create a topic where names can be imported 
    # without any issues
    GPS_SUB_TOPIC: str = "gps_sub_topic"
    RSSI_SUB_TOPIC: str = "rssi_sub_topic"
    CONTOUR_PUB_TOPIC: str = "contour_sub_topic"
    FREQ: str = "freq"

    def __init__(self):

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])
        
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
                (SyncPublisher.X_NAME, 0.0),
                (SyncPublisher.Y_NAME, 0.0),
                (SyncPublisher.Z_NAME, 0.0),
                (SyncPublisher.T_NAME, 0.0),
                # TODO: Create a topic where names can be imported 
                # without any issues
                (SyncPublisher.GPS_SUB_TOPIC, "message"), # "/p1/gps1"
                (SyncPublisher.RSSI_SUB_TOPIC, "rssi"),
                (SyncPublisher.CONTOUR_PUB_TOPIC, "contour"),
                (SyncPublisher.FREQ, 2.0)
            ]
        )

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.freq = self.get(SyncPublisher.FREQ)
        
        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
        self.debug: Callable = self.log().debug

        self.info(f"Namespace: {self.ns.namespace}")
        
        # Check if sim
        self.info(f"Is in sim mode: {self.ns.is_simulation}")

        # Create topics to subscribe to
        self.subscribe_to_topics()

        # Create topics to publish to
        self.create_publish_topics()

        self.create_timer(self.period, self.publish_contour)

    @property
    def period(self) -> Numeric:
        return 1/self.freq

    # Subscribes to all topics
    def subscribe_to_topics(self) -> None:

        # Subscribe to GPS data
        self.pubsub.create_subscription(
            NavSatFix,
            self.get(SyncPublisher.GPS_SUB_TOPIC),
            self.extract_x_y_values,
            10
        )

        # Subscribe to RSSI data
        self.pubsub.create_subscription(
            Int16,
            self.get(SyncPublisher.RSSI_SUB_TOPIC),
            self.extract_z_value,
            10
        )

    # Creates all publisher topics
    def create_publish_topics(self) -> None:

        # Create Contour topic
        self.pubsub.create_publisher(
            Contour,
            self.get(SyncPublisher.CONTOUR_PUB_TOPIC),
            10
        )

    # Publish contour data
    def publish_contour(self) -> None:

        # Form contour msg
        contour: Contour = Contour(
            x = self.x,
            y = self.y,
            z = self.z
        )

        # Log contour
        # TODO: use contour instead
        self.info(f"X:{self.x}, Y:{self.y}, Z:{self.z}")

        # Include timestamp
        contour.header.stamp = self.get_clock().now().to_msg()

        # Publish contour
        self.pubsub.publish(
            self.get(SyncPublisher.CONTOUR_PUB_TOPIC),
            contour
        )
    
    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value

    # TODO
    def extract_x_y_values(self, msg) -> None:
        self.x = msg.latitude
        self.y = msg.longitude
    
    # TODO
    def extract_z_value(self, msg) -> None:
        self.z = msg.data



def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    syncpub = SyncPublisher()

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(syncpub)


if __name__ == "main":
    main()
