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
from std_srvs.srv import Trigger
from wall_e_interfaces.msg import BatteryLevel, ServoPosition, ServoPositions, ArduinoStatus
from wall_e_interfaces.srv import ArduinoIntCommand, PlayAnimation

class WALLEArduino(Node):

    def __init__(self):
        """Initialize the node."""

        super().__init__('wall_e_arduino')

        # PARAMETERS ----------------------------------------------------------------------
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
        self.declare_parameter(
            'drive_motors_enable',
            True,
            ParameterDescriptor(
                description='Enable drive motor control through Arduino (if a motor shield is used)'
            )
        )
        self.drive_motors_enable = \
            self.get_parameter('drive_motors_enable').get_parameter_value().bool_value

        # PUBLISHERS ----------------------------------------------------------------------
        self.pub_arduino_status = self.create_publisher(ArduinoStatus, 'arduino_status', 10)
        self.pub_battery_level = self.create_publisher(BatteryLevel, 'battery_level', 10)
        self.pub_servo_positions = self.create_publisher(ServoPositions, 'servo_positions', 10)

        # SERVICE SERVERS ----------------------------------------------------------------------
        self.srv_connect = self.create_service(
            Trigger,
            'connect',
            self.srv_connect_callback
        )
        self.srv_disconnect = self.create_service(
            Trigger,
            'disconnect',
            self.srv_disconnect_callback
        )
        if self.drive_motors_enable:
            self.srv_move_linearly = self.create_service(
                ArduinoIntCommand,
                'move_linearly',
                self.srv_move_linearly_callback
            )
            self.srv_turn = self.create_service(
                ArduinoIntCommand,
                'turn',
                self.srv_turn_callback
            )
            self.srv_set_turn_offset = self.create_service(
                ArduinoIntCommand,
                'set_turn_offset',
                self.srv_set_turn_offset_callback
            )
            self.srv_set_motor_deadzone = self.create_service(
                ArduinoIntCommand,
                'set_motor_deadzone',
                self.srv_set_motor_deadzone_callback
            )
        self.srv_play_animation = self.create_service(
            PlayAnimation,
            'play_animation',
            self.srv_play_animation_callback
        )

        # TIMERS ----------------------------------------------------------------------
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

    # SERVICE SERVERS ----------------------------------------------------------------------
    def srv_connect_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Connect to the Arduino as a response to a "connect" service call."""

        response.success = self.connect()

        return response

    def srv_disconnect_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Disconnect from the Arduino as a response to a "disconnect" service call."""

        response.success = self.disconnect()

        return response

    def srv_move_linearly_callback(
        self,
        request: ArduinoIntCommand.Request,
        response: ArduinoIntCommand.Response
    ) -> ArduinoIntCommand.Response:
        """Move drive motors linearly at the specified speed [-100, 100]."""

        response.success = self.arduino.move_linearly(request.argument)

        return response

    def srv_turn_callback(
        self,
        request: ArduinoIntCommand.Request,
        response: ArduinoIntCommand.Response
    ) -> ArduinoIntCommand.Response:
        """Turn drive motors at the specified speed [-100, 100]."""

        response.success = self.arduino.turn(request.argument)

        return response


    def srv_set_turn_offset_callback(
        self,
        request: ArduinoIntCommand.Request,
        response: ArduinoIntCommand.Response
    ) -> ArduinoIntCommand.Response:
        """Set the turn offset of the drive motors to the specified value."""

        response.success = self.arduino.set_turn_offset(request.argument)

        return response


    def srv_set_motor_deadzone_callback(
        self,
        request: ArduinoIntCommand.Request,
        response: ArduinoIntCommand.Response
    ) -> ArduinoIntCommand.Response:
        """Set the drive motor deadzone to the specified value."""

        response.success = self.arduino.set_motor_deadzone(request.argument)

        return response

    def srv_play_animation_callback(
        self,
        request: PlayAnimation.Request,
        response: PlayAnimation.Response
    ) -> PlayAnimation.Response:
        """Play a servo animation by number."""

        response.success = self.arduino.play_animation(request.animation)

        return response

    # TIMERS ----------------------------------------------------------------------
    def tmr_status_callback(self):
        """Update Arduino status and publish on corresponding topics."""

        is_connected = self.arduino.is_connected()

        self.pub_arduino_status.publish(ArduinoStatus(is_connected=is_connected))

        if not is_connected:
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