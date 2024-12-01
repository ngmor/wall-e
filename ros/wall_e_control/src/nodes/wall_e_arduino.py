#!/usr/bin/env python3

# Copyright 2024 Nick Morales.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from wall_e import ArduinoDevice

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class WALLEArduino(Node):

    def __init__(self):
        """Initialize the node."""

        super().__init__('wall_e_arduino')

        # PARAMETERS ---------------------------------------------------------------------------
        self.declare_parameter(
            'device',
            '',
            ParameterDescriptor(
                description='Serial device port to communicate over which to communicate'
                + ' with the Arduino'
            )
        )
        self.device = self.get_parameter('device').get_parameter_value().string_value
        if self.device == '':
            raise RuntimeError("No serial 'device' parameter provided, aborting")

        # Init Arduino device and attempt to connect
        self.arduino = ArduinoDevice()
        self.connect()

        self.get_logger().info(f'{self.get_fully_qualified_name()} node started')

    def connect(self, log: bool=True) -> bool:
        """Connect to the Arduino with logger message if log=True."""

        success = self.arduino.connect(self.device)

        if log:
            if success:
                self.get_logger().info(f'Successfully connected to Arduino on {self.device}')
            else:
                self.get_logger().error(f'Failed to connect to Arduino on {self.device}')

        return success
    
    def disconnect(self, log: bool=True) -> bool:
        """Disconnect from the Arduino with logger message if log=True."""

        success = self.arduino.disconnect()

        if log:
            if success:
                self.get_logger().info(f'Successfully disconnected from Arduino on {self.device}')
            else:
                self.get_logger().error(f'Failed to disconnect from Arduino on {self.device}')

        return success



def main(args=None):
    rclpy.init(args=args)
    node = WALLEArduino()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.disconnect(False)

if __name__ == '__main__':
    main()