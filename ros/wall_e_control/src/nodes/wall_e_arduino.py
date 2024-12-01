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

from wall_e import ArduinoDevice, SERVO_INDEX_TO_NAME

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from wall_e_interfaces.msg import BatteryLevel, ServoPosition, ServoPositions

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
        self.declare_parameter(
            'status_update_rate',
            0.1,
            ParameterDescriptor(
                description='Rate at which status messages from Arduino are published'
            )
        )

        # PUBLISHERS ---------------------------------------------------------------------------
        self.pub_battery_level = self.create_publisher(BatteryLevel, 'battery_level', 10)
        self.pub_servo_positions = self.create_publisher(ServoPositions, 'servo_positions', 10)

        # TIMERS ---------------------------------------------------------------------------
        self.tmr_status = self.create_timer(
            self.get_parameter('status_update_rate').get_parameter_value().double_value,
            self.tmr_status_callback
        )

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

    # TIMERS ---------------------------------------------------------------------------
    def tmr_status_callback(self):
        if not self.arduino.is_connected():
            return

        battery_level = self.arduino.get_battery_level()

        # only publish valid value has been received
        if battery_level is not None:
            self.pub_battery_level.publish(BatteryLevel(level=battery_level))

        servo_pos = self.arduino.get_servo_positions()

        # only publish if valid value has been received for all servos
        if None not in servo_pos:
            msg = ServoPositions()

            for i in range(len(servo_pos)):
                msg.servos.append(ServoPosition(
                    name=SERVO_INDEX_TO_NAME[i],
                    position=servo_pos[i]
                ))

            self.pub_servo_positions.publish(msg)



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