import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from typing import Any, Union, Callable
from types import ModuleType
from functools import wraps

import importlib
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from collections import defaultdict

from adaptive_navigation_utilities.pubsub import PubSubManager
from adaptive_navigation_utilities.namespace_builder import HardwareNamespace
from adaptive_navigation_utilities.config_schema import BaseRosConfig




class NodeWrapper(Node):

    def __init__(self, 
                 name: str = __name__.split('.')[-1],
                 package: str = None,
                 config: str = None) -> None:

        # Assign name
        self.name = name

        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name       
        super().__init__(name)

        # Get namespace
        self.ns: HardwareNamespace = HardwareNamespace(self.get_namespace())

        # Add aliases for logging
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error
        self.debug: Callable = self.log().debug

        # Create PubSubManager
        self.pubsub: PubSubManager = PubSubManager(self)


        # Declare package
        descriptor = ParameterDescriptor(
            dynamic_typing=True
        )
        self.declare_parameter('package', package, descriptor=descriptor)
        self.package = self.get('package')
  
        # Declare config parameter whose default value is None
        self.declare_parameter('config', config, descriptor=descriptor)

        # Initialize config with empty dictionary
        self.config = defaultdict(lambda: None)
        # Render config if exists
        self.update_config_path()
        self.create_config()

        # Create publish and subcription topics
        self.create_publish_topics()
        self.create_subscription_topics()



   # Skips the method if it requires config file
    def requires_config(func: Callable):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if not self.config_exists:
                self.warn(f"No config exists. Skipping {func.__name__}")
                return
            else:
                result = func(self, *args, **kwargs)
                return result
        return wrapper

   # Skips the method if it requires config file
    def requires_package_name(func: Callable):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.get('package') == None:
                self.warn(f"Need ROS parameter of package name. Skipping {func.__name__}")
                return
            else:
                result = func(self, *args, **kwargs)
                return result
        return wrapper

    # Reports if the function is non-simulation only
    def non_simulation(func: Callable):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            if self.ns.is_simulation:
                self.error(f"Cannot run {func.__name__} at while in sim!")
                return
            else:
                result = func(self, *args, **kwargs)
                return result
        return wrapper
    
    # Provides alternative function to be run in sim
    def alternate_sim_func(alt_func: Callable):
        @wraps(alt_func)
        def decorator(func: Callable):
            def wrapper(self, *args, **kwargs):
                if self.ns.is_simulation:
                    self.warn(f"Cannot run {func.__name__} at while in sim!")
                    result = alt_func(self, *args, **kwargs)
                else:
                    result = func(self, *args, **kwargs)
                    return result
            return wrapper
        return decorator

    @property
    # TODO should this be replaced
    def config_exists(self) -> bool:
        return self.get('config') != None

    # Create alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    # Create alias function for getting logger
    def log(self):
        return self.get_logger()
    
    # Update config path
    @requires_package_name
    def update_config_path(self) -> None:


        config_file = os.path.join(
            get_package_share_directory(self.package),
            'config',
            f"{self.name}.yaml"
        )
        print(config_file)

        # If a config path exists
        if Path(config_file).exists():

            # Create new parameter
            config = rclpy.parameter.Parameter(
                'config',
                rclpy.Parameter.Type.STRING,
                config_file
            )

            # Update config ROS parameter
            self.set_parameters([config])

        else:
            self.warn(f"Create a config path under the path: {config_file}")
            return

    @requires_config
    def create_config(self) -> None:

        # Get config path
        config_path: str = self.get('config')
        with open(config_path, 'r') as f:
            config_dict = defaultdict(lambda: None, yaml.safe_load(f))

        # Process config into template and access node directly
        self.config = BaseRosConfig.model_validate(config_dict)


    @requires_config
    def create_publish_topics(self) -> None:

        # For each publish topicc
        for t in self.config.topics.publishers:

            # Extract message type and import in Node
            msg_cls: ModuleType = self.import_ros_interface_type(t.type)

            # Create publish topic
            self.pubsub.create_publisher(msg_cls, t.name, t.qos)

    @requires_config
    def create_subscription_topics(self):

        # For each subscription topic
        for s in self.config.topics.subscribers:

            # Extract message type and import in Node
            msg_cls: ModuleType = self.import_ros_interface_type(s.type)

            # Exract callback function 
            # NOTE: Assumes that callback is a method inside class,
            #       which is almost always the case
            callback_fcn: Callable = getattr(self, s.callback, None)

            # Skip iteration if invalid callback
            if not callable(callback_fcn):
                self.error(f"Invalid callback: {s.callback}")
                continue
            else:
                # Create subscription
                self.pubsub.create_subscription(msg_cls, s.name, callback_fcn, s.qos)
    
    @staticmethod
    def import_ros_interface_type(type_str) -> ModuleType:
        """
        Dynamically imports a ROS 2 interface type (msg, srv, or action) from a string.

        Example inputs:
        - "geometry_msgs/msg/Twist"
        - "std_srvs/srv/SetBool"
        - "nav2_msgs/action/NavigateToPose"

        Returns:
        - The class object for the message, service, or action
        """
        try:
            if '/msg/' in type_str:
                iface_type = 'msg'
            elif '/srv/' in type_str:
                iface_type = 'srv'
            elif '/action/' in type_str:
                iface_type = 'action'
            else:
                raise ValueError(f"Unknown interface type in: {type_str}")

            pkg, name = type_str.split(f'/{iface_type}/')
            module = importlib.import_module(f"{pkg}.{iface_type}")
            return getattr(module, name)

        except (ValueError, ModuleNotFoundError, AttributeError) as e:
            raise ImportError(f"Could not import ROS {iface_type.upper()} type '{type_str}': {e}")
