import rclpy
from rclpy.node import Node
from adaptive_navigation_interfaces.msg import Contour

from typing import List, Any, Callable

from builtin_interfaces.msg import Time
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt
import numpy as np

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

        # Update frequency
        self.buffer: int = 20
        self.freq: MovingAverage = MovingAverage(self.buffer)
        self.prev_stamp: float = 0.0

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
        self.info(f"Data: {self.freq.data}")
        self.info("~~~~~~~~~~~~~~~~~~~~~~~")


    def listener_callback(self, msg: Contour):
        # if len(msg.x) != len(msg.y) or len(msg.x) != len(msg.z):
        #     self.get_logger().warn("Received data with mismatched array lengths.")
        #     return

        self.info(f"X:{msg.x}, Y:{msg.y}, Z:{msg.z}")

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
