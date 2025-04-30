import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import time

# Replace with your actual serial port and baud rate
PORT = "/dev/ttyUSB2"  # For Windows: "COM3"
BAUD_RATE = 9600

# This is the 64-bit broadcast address
BROADCAST_ADDRESS = XBee64BitAddress.BROADCAST_ADDRESS

device = XBeeDevice(PORT, BAUD_RATE)

class MinimalPublisher(Node):

    def __init__(self):
		    
		# Create node called "minimal_publisher"
        super().__init__('minimal_publisher')
        
        # Assign publisher functionality for a topic named 'topic'
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Have a publisher create a timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Additional stuff
        self.i = 0

    def timer_callback(self):
    
		    # Create a `String` msg instance and change .data attribute
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        
        # Send message
        self.publisher_.publish(msg)
        
        # Log this onto console
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # Iterate additional stuff
        self.i += 1



	# Initialize ros client library
    rclpy.init(args=args)

	# Create publisher node
    minimal_publisher = MinimalPublisher()
		
	# Continuously blocking poll
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def main(args=None):

    try:
        device.open()

        # Create a broadcast RemoteXBeeDevice (optional step)
        broadcast_device = RemoteXBeeDevice(device, BROADCAST_ADDRESS)

        print("Broadcasting messages every 1 second...")

        while True:
            message = "Hello from broadcast!"
            device.send_data(broadcast_device, message)
            print(f"Sent broadcast: {message}")
            time.sleep(1)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()