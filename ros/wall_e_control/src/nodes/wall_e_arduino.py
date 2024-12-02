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

from wall_e import \
    ArduinoDevice, SERVO_INDEX_TO_NAME, servo_index_is_valid, servo_name_is_valid, \
    EyeMovements, HeadMovements, ArmMovements

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from wall_e_interfaces.msg import \
    BatteryLevel, ServoPosition, ServoPositions, ArduinoStatus, DriveMotorSpeed
from wall_e_interfaces.srv import \
    ArduinoIntCommand, PlayAnimation, MoveEyes, MoveHead, MoveArms

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

        # SUBSCRIBERS ----------------------------------------------------------------------
        if self.drive_motors_enable:
            self.sub_linear_speed = self.create_subscription(
                DriveMotorSpeed,
                'linear_speed',
                self.sub_linear_speed_callback,
                10
            )
            self.sub_turn_speed = self.create_subscription(
                DriveMotorSpeed,
                'turn_speed',
                self.sub_turn_speed_callback,
                10
            )
        self.sub_servo_position_commands = self.create_subscription(
            ServoPositions,
            'servo_position_commands',
            self.sub_servo_position_commands_callback,
            10
        )

        # PUBLISHERS ----------------------------------------------------------------------
        self.pub_arduino_status = self.create_publisher(ArduinoStatus, 'arduino_status', 10)
        self.pub_battery_level = self.create_publisher(BatteryLevel, 'battery_level', 10)
        self.pub_servo_position_feedback = \
            self.create_publisher(ServoPositions, 'servo_position_feedback', 10)

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
        self.srv_activate_auto_servo_mode = self.create_service(
            Trigger,
            'activate_auto_servo_mode',
            self.srv_activate_auto_servo_mode_callback
        )
        self.srv_deactivate_auto_servo_mode = self.create_service(
            Trigger,
            'deactivate_auto_servo_mode',
            self.srv_deactivate_auto_servo_mode_callback
        )
        self.srv_move_eyes = self.create_service(
            MoveEyes,
            'move_eyes',
            self.srv_move_eyes_callback
        )
        self.srv_move_head = self.create_service(
            MoveHead,
            'move_head',
            self.srv_move_head_callback
        )
        self.srv_move_arms = self.create_service(
            MoveArms,
            'move_arms',
            self.srv_move_arms_callback
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

    # SUBSCRIBERS ----------------------------------------------------------------------
    def sub_linear_speed_callback(self, msg: DriveMotorSpeed):
        """Move drive motors linearly at the specified speed [-100, 100]."""

        success = self.arduino.move_linearly(msg.speed)

        if not success:
            self.get_logger().error(f'Error commanding linear drive speed: {msg.speed}')

    def sub_turn_speed_callback(self, msg: DriveMotorSpeed):
        """Turn drive motors at the specified speed [-100, 100]."""

        success = self.arduino.turn(msg.speed)

        if not success:
            self.get_logger().error(f'Error commanding turn speed: {msg.speed}')
    
    def sub_servo_position_commands_callback(self, msg: ServoPositions):
        """Manually move servos as per positions received in message."""

        for servo in msg.servos:
            name_specified = servo.name != ''
            index_specified = servo.index != -1

            if not name_specified and not index_specified:
                self.get_logger().error(
                    'Servo position message received that contained an unspecified servo, skipping'
                )
                continue
            elif name_specified and index_specified:
                if not servo_index_is_valid(servo.index):
                    self.get_logger().error(
                        f'Invalid servo index received: {servo.index}: {servo.name}, skipping'
                    )
                    continue
                if SERVO_INDEX_TO_NAME[servo.index] != servo.name:
                    self.get_logger().error(
                        'Mismatch between received servo name'
                        + f' ({servo.name}) and index ({servo.index}), skipping'
                    )
                    continue

            if index_specified:
                success = self.arduino.move_servo_by_index(servo.index, servo.position)
            else:
                success = self.arduino.move_servo_by_name(servo.name, servo.position)

            if not success:
                self.get_logger().error(f'Failed to move servo {servo.index}: {servo.name}')


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

        if response.success:
            self.get_logger().info(f'Successfully triggered animation {request.animation}')
        else:
            self.get_logger().info(f'Failed to trigger animation {request.animation}')

        return response

    def srv_activate_auto_servo_mode_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Activate auto servo mode."""

        response.success = self.arduino.activate_auto_servo_mode()

        if response.success:
            self.get_logger().info('Auto servo mode activated')
        else:
            self.get_logger().error('Failed to activate auto servo mode')

        return response

    def srv_deactivate_auto_servo_mode_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Deactivate auto servo mode."""

        if response.success:
            self.get_logger().info('Auto servo mode deactivated')
        else:
            self.get_logger().error('Failed to deactivate auto servo mode')

        response.success = self.arduino.deactivate_auto_servo_mode()

        return response

    def srv_move_eyes_callback(
        self,
        request: MoveEyes.Request,
        response: MoveEyes.Response
    ) -> MoveEyes.Response:
        """Trigger a specific eye movement."""

        response.success = False

        try:
            movement = EyeMovements(request.movement)

            response.success = self.arduino.move_eyes(movement)

            if response.success:
                self.get_logger().info(f'Successfully triggered {movement.name} eye movement')
            else:
                self.get_logger().info(f'Failed to trigger {movement.name} eye movement')

        except ValueError as ex:
            self.get_logger().error(f'Failed to interpret movement command: {ex}')

        return response

    def srv_move_head_callback(
        self,
        request: MoveHead.Request,
        response: MoveHead.Response
    ) -> MoveHead.Response:
        """Trigger a specific head movement."""

        response.success = False

        try:
            movement = HeadMovements(request.movement)

            response.success = self.arduino.move_head(movement)

            if response.success:
                self.get_logger().info(f'Successfully triggered {movement.name} head movement')
            else:
                self.get_logger().info(f'Failed to trigger {movement.name} head movement')

        except ValueError as ex:
            self.get_logger().error(f'Failed to interpret movement command: {ex}')

        return response

    def srv_move_arms_callback(
        self,
        request: MoveArms.Request,
        response: MoveArms.Response
    ) -> MoveArms.Response:
        """Trigger a specific arm movement."""

        response.success = False

        try:
            movement = ArmMovements(request.movement)

            response.success = self.arduino.move_arms(movement)

            if response.success:
                self.get_logger().info(f'Successfully triggered {movement.name} arm movement')
            else:
                self.get_logger().info(f'Failed to trigger {movement.name} arm movement')

        except ValueError as ex:
            self.get_logger().error(f'Failed to interpret movement command: {ex}')

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
                    index=i,
                    position=servo_pos[i]
                ))

            self.pub_servo_position_feedback.publish(msg)



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