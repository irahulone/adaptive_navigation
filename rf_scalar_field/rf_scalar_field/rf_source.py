import rclpy
from rclpy.node import Node

from digi.xbee.devices import XBeeDevice
from typing import Any, Union, Callable


# Global variables
Numeric = Union[int, float]
DEBUG: bool = True


class RFSource(Node):
    """  
    Creates the RF Source node and continously
    sends broadcast data using the send_broadcast_data
    method. 

    Implemented using in ROS2 and Digi-Xbee library.
    """

    PORT: str = "port"
    BAUD_RATE: str = "baud_rate"
    FREQ: int = "freq"
    
    def __init__(self) -> None:

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
                (RFSource.PORT, "/dev/ttyUSB2"),
                (RFSource.BAUD_RATE, 115200),
                (RFSource.FREQ, 0.5)
            ]
        )

        # Set attributes from parameters
        # Note: Parameters are at the node level
        #       while attributes are the class level
        self.port: str = self.get(RFSource.PORT)
        self.baud_rate: str = self.get(RFSource.BAUD_RATE)
        self.freq: int = self.get(RFSource.FREQ)

        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error

        if DEBUG: print(self.port)

        # Initialize XBee Device
        self.device: XBeeDevice = self.xbee_init()

        # Open the device
        self.device.open()
        if DEBUG: print("Device is opening...")

        # Add a callback fcn in ROS per timer
        self.timer = self.create_timer(self.period, self.send_data_broadcast)

    @property
    def period(self) -> Numeric:
        return 1/self.freq

    # Crete alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    def log(self):
        return self.get_logger()

    # Create XBee device
    def xbee_init(self) -> XBeeDevice:
        return XBeeDevice(self.port, self.baud_rate)
    
    # Send broadcast data
    def send_data_broadcast(self, msg: str = 'Hello world!') -> None:
        try:

            # If the device is not already open, open it
            if not self.device.is_open():
                self.device.open()

            # Send broadcast message
            self.info(f"Sending message: {msg}")
            self.device.send_data_broadcast(msg)

        except Exception as e:

            # Print error
            print(f"Error: {e}")
            return False
        
        finally:

            # Close device if caught exception
            if self.device is not None and self.device.is_open():
                self.device.close()


def main(args=None):

    if DEBUG: print(__name__)
    
    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    rf_source = RFSource()

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(rf_source)


if __name__ == '__main__':
    main()