import rclpy
from rclpy.node import Node

from typing import Callable, Any, Union

from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Pose2D

import os
import numpy as np
from typing import List, Any, Callable
from functools import partial

from math import nan, inf

from adaptive_navigation_interfaces.msg import Contour
from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace


class StringWrapper(str):
    def prepend(self, str):
        return self.__class__(str + self)
    

def prepend(str1: str, str2: str):
    return str1+str2

class Data:
    def __init__(self):
        self.data: List = []
    
    @property
    def size(self):
        return len(self.data)
    
    def _insert(self, data: Any, ind: int) -> bool:
        try:
            if ind >= self.size:
                self.data.append(data)
            else:
                self.data[ind] = data
            return True
        except Exception as e:
            print(e)
            return False
    
    def append(self, data: Any) -> bool:
        try:
            self.data.append(data)
            return True
        except Exception as e:
            print(e)
            return False

class ContinuousData(Data):
    def __init__(self):
        super().__init__()
        self._precision: float = inf
        self._p1: int = -1
        self._p2: int = -2
        self._precision_ind_1 = 0
        self._precision_ind_2 = 0

    @property
    def max(self):
        return np.max(self.data)

    @property
    def min(self):
        return np.min(self.data)

    @property
    def avg(self):
        return np.average(self.data)
    
    @property
    def std(self):
        return np.std(self.data)
    
    @property
    def precision(self):
        return self._precision

    def _update_precision(self) -> bool:

        # Need at least two measurements
        if len(self.data) >= 2:

            # Get current difference between last 2 successive measurments
            diff: float = abs(self.data[self._p1] - self.data[self._p2])

            # Update precision if difference is less than current
            if diff < self.precision:
                self._precision = diff 
                self._precision_ind_1 = self._p1
                self._precision_ind_2 = self._p2
            
            return True
        else:
            return False

    def append(self, data) -> bool:

        # Perform append as usual
        if super().append(data):

            # Update precision
            if self._update_precision():
                return True
            else:
                return False
            
    def _insert(self, data: Any, ind: int) -> bool:

        # Perform insert as usual
        if super()._insert(data, ind):

            # Update precision
            if self._update_precision():
                return True
            else:
                return False


class MovingAverage(ContinuousData):
    def __init__(self, buffer: int = 10):
        super().__init__()
        self._buffer: int = buffer
        self._ind: int = 0

    @property
    def buffer(self):
        return self._buffer
    
    @buffer.setter
    def buffer(self, buffer):
        self._buffer: float = buffer

    def append(self, data: Any) -> bool:

        # Update precision indices
        self._p1 = self._ind
        self._p2 = self._ind - 1 % self.buffer

        # Insert data
        super()._insert(data, self._ind)
        
        # Wrap index if larger than bugger
        self._ind = (self._ind + 1) % self.buffer


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
    DIST_PER_UNIT_RSSI_PUB_TOPIC: str = "dist_per_unit_rssi_pub_topic"
    UPDATE_FREQ_PUB_TOPIC: str = "update_freq_pub_topic"
    POSE_SUB_TOPIC: str = "pose_sub_topic"
    ROBOT_ID: str = "robot_id"
    FREQ: str = "freq"

    def __init__(self):

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name

        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "")
        prepend_robot = partial(prepend, robot_id)
        
        super().__init__("_".join([robot_id, __name__.split('.')[-1]]))
        
        # Pubsub
        self.pubsub: PubSubManager = PubSubManager(self)

        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())
        self.ns.parts = [robot_id]
        # Attributes
        self.x: float = nan
        self.y: float = nan
        self.z: int = 0 
    
        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='', # TODO: Include parameters here??
            parameters=[
                (SyncPublisher.ROBOT_ID, robot_id),
                (SyncPublisher.X_NAME, 0.0),
                (SyncPublisher.Y_NAME, 0.0),
                (SyncPublisher.Z_NAME, 0.0),
                (SyncPublisher.T_NAME, 0.0),
                # TODO: Create a topic where names can be imported 
                # without any issues
                (SyncPublisher.GPS_SUB_TOPIC, prepend_robot("message")), # "/p1/gps1"
                (SyncPublisher.RSSI_SUB_TOPIC, prepend_robot("rssi")),
                (SyncPublisher.CONTOUR_PUB_TOPIC, "contour"),
                (SyncPublisher.FREQ, 5.0),
                (SyncPublisher.POSE_SUB_TOPIC, prepend_robot("pose2D")),
                (SyncPublisher.DIST_PER_UNIT_RSSI_PUB_TOPIC, prepend_robot("dist_per_rssi")),
                (SyncPublisher.UPDATE_FREQ_PUB_TOPIC, prepend_robot("rssi_freq"))
            ]
        )


        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.timer_freq = self.get(SyncPublisher.FREQ)
        self.robot_id = self.get(SyncPublisher.ROBOT_ID)

        # Update frequency
        self.freq_buffer: int = 20
        self.freq: MovingAverage = MovingAverage(self.freq_buffer)
        # self.freq: ContinuousData = ContinuousData()
        self.prev_stamp: float = 0.0
        
        # Meters per change in RSSI
        self.last_x: float = 0.0
        self.last_y: float = 0.0
        self.last_z: float = 0.0


        # Distance per RSSI
        self.dist_per_rssi_buffer: int = 20
        self.dist_per_rssi: MovingAverage = MovingAverage(self.dist_per_rssi_buffer)
        # self.dist_per_rssi: ContinuousData = ContinuousData()


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
        return 1/self.timer_freq

    # Subscribes to all topics
    def subscribe_to_topics(self) -> None:

        # # Subscribe to GPS data
        # self.pubsub.create_subscription(
        #     NavSatFix,
        #     self.get(SyncPublisher.GPS_SUB_TOPIC),
        #     self.extract_x_y_values,
        #     10
        # )

        # Subscribe to RSSI data
        self.pubsub.create_subscription(
            Float64,
            self.get(SyncPublisher.RSSI_SUB_TOPIC),
            self.extract_z_value,
            10
        )

        self.pubsub.create_subscription(
            Pose2D,
            self.get(SyncPublisher.POSE_SUB_TOPIC), 
            self.extract_x_y_values,
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

        # Create Distance per unit RSSI pub topic
        self.pubsub.create_publisher(
            Float64,
            self.get(SyncPublisher.DIST_PER_UNIT_RSSI_PUB_TOPIC),
            10
        )

        # Create update frequency topic
        self.pubsub.create_publisher(
            Float64,
            self.get(SyncPublisher.UPDATE_FREQ_PUB_TOPIC)
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

        # Update dist per unit RSSI
        self.update_dist_per_unit_rssi()

        # Publish contour
        self.pubsub.publish(
            self.get(SyncPublisher.CONTOUR_PUB_TOPIC),
            contour
        )
    
    @staticmethod
    def get_norm2_distance(vec1, vec2) -> np.float64:
        return np.linalg.norm(vec1 - vec2)


    def update_dist_per_unit_rssi(self):

        # If there is a change in RSSI
        if self.z != self.last_z:

            # Get Norm 2 distance
            vec1: np.ndarray = np.array([self.x, self.y])
            vec2: np.ndarray = np.array([self.last_x, self.last_y])
            dist_per_unit_rssi: np.float64 = self.get_norm2_distance(vec1, vec2)

            # Update Change per RSSI
            self.dist_per_rssi.append(dist_per_unit_rssi)
        

            self.info(f"Dist Per RSSI Avg: {self.dist_per_rssi.avg}")
            self.info(f"Dist Per RSSI Std: {self.dist_per_rssi.std}")
            self.info(f"Dist Per RSSI Min: {self.dist_per_rssi.min}")
            self.info(f"Dist Per RSSI Max: {self.dist_per_rssi.max}")
            self.info(f"Dist Per RSSI Size: {self.dist_per_rssi.size}")  
            self.info(f"Dist Per RSSI Ind: {self.dist_per_rssi._ind}")
            self.info(f"Precision: {self.dist_per_rssi.precision}")
            self.info(f"Precision Index 1: {self.dist_per_rssi._precision_ind_1}")
            self.info(f"Precision Index 2: {self.dist_per_rssi._precision_ind_2}")
            self.info(f"Dist Per RSSI Data: {self.dist_per_rssi.data}")
            self.info("~~~~~~~~~~~~~~~~~~~~~~~")


            # Update last RSSI value
            self.last_y = self.x
            self.last_y = self.y
            self.last_z = self.z

            # Publisher topic
            self.pubsub.publish(self.get(SyncPublisher.DIST_PER_UNIT_RSSI_PUB_TOPIC),
                                Float64(data=dist_per_unit_rssi))

    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value

    def extract_x_y_values(self, msg) -> None:
        self.x = msg.x
        self.y = msg.y
    
    def extract_z_value(self, msg) -> None:
        self.z = msg.data
        self.update_rssi_stats(msg)

    @staticmethod
    def extract_time(time: Time, digits: int = 3) -> float:
        return round(time.sec + time.nanosec * 1e-9, digits)


    def update_rssi_stats(self, msg):

        current_stamp: float = self.extract_time(self.get_clock().now().to_msg())
        freq = round(1/(current_stamp - self.prev_stamp), 3)
        self.freq.append(freq)
        self.prev_stamp = current_stamp

        self.info(f"Avg: {self.freq.avg}")
        self.info(f"Std: {self.freq.std}")
        self.info(f"Min: {self.freq.min}")
        self.info(f"Max: {self.freq.max}")
        self.info(f"Size: {self.freq.size}")  
        self.info(f"Ind: {self.freq._ind}")
        self.info(f"Precision: {self.freq.precision}")
        self.info(f"Precision Index 1: {self.freq._precision_ind_1}")
        self.info(f"Precision Index 2: {self.freq._precision_ind_2}")
        self.info(f"Data: {self.freq.data}")
        self.info("~~~~~~~~~~~~~~~~~~~~~~~")

        # Publish RSSI freq topic
        self.pubsub.publish(self.get(SyncPublisher.UPDATE_FREQ_PUB_TOPIC),
                            Float64(data=freq))


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
