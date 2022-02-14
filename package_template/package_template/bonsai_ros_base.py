import requests
import json
from typing import Dict
import math
import numpy as np
import os
import json
import uuid

# ROS
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class BonsasiROSBase(Node):
    def __init__(self, node_name):
        # Calls Node.__init__('listener')
        super().__init__(node_name)

        # Add topic subscribers and publishers here
        # self.data_sub = self.create_subscription(msg_type, "/topic", self._callback, 10)

        # Initialize states and parameters
        self.state = {}
        self.init_params()

    def init_params(self):
        # initialize all state values to 0
        self.state['value_name'] = 0.0

    # define callback functions for topic subscribers
    # recommended to separate saving state data and 
    # fetching state data to send to Bonsai
    # Example
    
    # def _callback(self, data):
    #     self.data = data

    # def get_data(self):
    #     self.state["value_name"] = self.data.value

