import rclpy
from rclpy.node import Node
from adaptive_navigation_interfaces.msg import Contour

import matplotlib.pyplot as plt
import numpy as np

class ContourPlotter(Node):
    def __init__(self):
        super().__init__('contour_plotter')

        self.subscription = self.create_subscription(
            Contour,
            'contour',
            self.listener_callback,
            10
        )

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

    def listener_callback(self, msg: Contour):
        # if len(msg.x) != len(msg.y) or len(msg.x) != len(msg.z):
        #     self.get_logger().warn("Received data with mismatched array lengths.")
        #     return

        self.get_logger().info(f"X:{msg.x}, Y:{msg.y}, Z:{msg.z}")

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
            self.get_logger().error(f"Plot error: {e}")

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
