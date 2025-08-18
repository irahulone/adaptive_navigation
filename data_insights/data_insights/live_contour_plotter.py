import rclpy
from rclpy.node import Node
from adaptive_navigation_interfaces.msg import Contour

from typing import List, Any, Callable

from builtin_interfaces.msg import Time
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt
import numpy as np

from math import inf

from adaptive_navigation_utilities.pubsub import PubSubManager


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


class ContourPlotter(Node):

    RSSI_SUB_TOPIC: str = "rssi"

    def __init__(self):
        super().__init__('contour_plotter')

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Create publish topics
        self.create_subscriptions()

        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
        self.debug: Callable = self.log().debug

        self.x = list()
        self.y = list()
        self.z = list()

        self.latest_data = None

        # Meters per change in RSSI
        self.last_x: float = 0.0
        self.last_y: float = 0.0
        self.last_z: float = 0.0

        # Update frequency
        self.freq_buffer: int = 20
        self.freq: MovingAverage = MovingAverage(self.freq_buffer)
        # self.freq: ContinuousData = ContinuousData()
        self.prev_stamp: float = 0.0

        # Distance per RSSI
        self.dist_per_rssi_buffer: int = 20
        self.dist_per_rssi: MovingAverage = MovingAverage(self.dist_per_rssi_buffer)
        # self.dist_per_rssi: ContinuousData = ContinuousData()


        # Set up matplotlib
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        # Set the view: elevation=90 looks from top, azimuth=270 or 0 rotates view horizontally
        self.ax.view_init(elev=90, azim=270)

        # Use ROS 2 timer to update the plot
        self.timer = self.create_timer(0.5, self.update_plot)  # Every 0.5 seconds

    def create_subscriptions(self) -> None:
        self.pubsub.create_subscription(
            Contour,
            'contour',
            self.listener_callback,
            10
        )
        
        self.pubsub.create_subscription(
            Int16,
            ContourPlotter.RSSI_SUB_TOPIC, 
            self.update_rssi_stats,
            10
        )

    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    def update_rssi_stats(self, msg):

        current_stamp: float = self.extract_time(self.get_clock().now().to_msg())
        self.freq.append(round(1/(current_stamp - self.prev_stamp), 3))
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

    @staticmethod
    def get_norm2_distance(vec1, vec2) -> np.float64:
        return np.linalg.norm(vec1 - vec2)

    def listener_callback(self, msg: Contour):
        # if len(msg.x) != len(msg.y) or len(msg.x) != len(msg.z):
        #     self.get_logger().warn("Received data with mismatched array lengths.")
        #     return

        # self.info(f"X:{msg.x}, Y:{msg.y}, Z:{msg.z}")

        # If there is a change in RSSI
        if msg.z != self.last_z:

            # Get Norm 2 distance
            vec1: np.ndarray = np.array([msg.x, msg.y])
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
            self.last_y = msg.x
            self.last_y = msg.y
            self.last_z = msg.z

        self.latest_data = (np.array(msg.x), np.array(msg.y), np.array(msg.z))

    def extract_time(self, time: Time, digits: int = 3) -> float:
        return round(time.sec + time.nanosec * 1e-9, digits)

    def update_plot(self):
        if self.latest_data is None:
            return

        x, y, z = self.latest_data
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)


        try:
            self.ax.clear()
            self.ax.scatter(self.x, self.y, self.z, c=self.z, cmap='coolwarm')
            self.ax.set_title("Live Contour Plot")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        except Exception as e:
            self.error(f"Plot error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ContourPlotter()

    try:
        plt.show(block=False)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
