import rclpy


from adaptive_navigation_utilities.nodewrapper import NodeWrapper
from adaptive_navigation_utilities.config_schema import PublisherTopic, SubscriberTopic

from typing import List


package_name = 'data_insights'


class TestNode(NodeWrapper):

    def __init__(self, package='data_insights'):


        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1], package=package)

        # Print node name
        self.info(f"Node name: {self.get_name()}")

        # Print if namespace
        print(f"Is simulation?: {self.ns.is_simulation}")

        print(f"Configs: {dict(self.config.extra)}")
        print(f"Robot id: {self.config.extra["robot_id"]}")


    def print_sim_hi(self, msg):
        print("sim hi")

    @NodeWrapper.alternate_sim_func(print_sim_hi)
    def print_hi(self, msg):
        print("hi")



def main(args=None):

    # Initialize ros client library
    rclpy.init(args=args)

    # Creates a rf source node
    test_node = TestNode()
    

    # Continuously execute the node
    # The execution lies in the RFSource.timer() 
    # function that uses a callback function
    rclpy.spin(test_node)

    


if __name__ == '__main__':
    main()