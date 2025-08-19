import rclpy
from rclpy.node import Node
from adaptive_navigation_interfaces.msg import Contour


from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

import matplotlib.pyplot as plt
import numpy as np
from typing import Callable, Any, Union
import os

from adaptive_navigation_utilities.pubsub import PubSubManager


class ContourPlotter(Node):

    CONTOUR_SUB_TOPIC: str = "contour_sub_topic"
    ROBOT_ID: str = "robot_id"

    def __init__(self):

        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "")

        super().__init__("_".join([robot_id, __name__.split('.')[-1]]))

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Declare parameters provided with default params
        # if not using ros2 launch
        self.declare_parameters(
            namespace='', # TODO: Include parameters here??
            parameters=[
                (ContourPlotter.CONTOUR_SUB_TOPIC, "contour"),
                (ContourPlotter.ROBOT_ID, robot_id)
            ]
        )

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level  
        self.robot_id = self.get(ContourPlotter.ROBOT_ID)

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
            self.prepend(self.get(ContourPlotter.CONTOUR_SUB_TOPIC)),
            self.listener_callback,
            10
        )

    # Create alias function for getting logger
    def log(self):
        return self.get_logger()

    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value

    def listener_callback(self, msg: Contour):
        # if len(msg.x) != len(msg.y) or len(msg.x) != len(msg.z):
        #     self.get_logger().warn("Received data with mismatched array lengths.")
        #     return

        # self.info(f"X:{msg.x}, Y:{msg.y}, Z:{msg.z}")

        self.latest_data = (np.array(msg.x), np.array(msg.y), np.array(msg.z))

  
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

    # TODO: Replace with namespace builder
    def prepend(self, topic: str) -> str:
        return f"{self.robot_id}/{topic}"


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
