import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import time

# Replace with your actual serial port and baud rate
PORT = "/dev/ttyUSB2"  # For Windows: "COM3"
BAUD_RATE = 115200
device = XBeeDevice(PORT, BAUD_RATE)
period = 0.1

def main(args=None):

    try:
        device.open()

        print(f"Broadcasting messages every {period} second...")
        while True:
            message = "Hello from broadcast!"
            device.send_data_broadcast(message)
            print(f"Sent broadcast: {message}")
            time.sleep(period)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()