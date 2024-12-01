#############################################
# WALL-E Robot Arduino Device Interface
#
# @file       arduino_device.py
# @brief      Arduino interface to control Wall-e robot
# @author     Simon Bluett, Nick Morales
# @website    https://wired.chillibasket.com
# @copyright  Copyright (C) 2021-2024 - Distributed under MIT license
# @version    1.0
# @date       29th November 2024
#############################################

from queue import Queue
from threading import Event, Thread
from serial import Serial
import serial.tools.list_ports as list_ports
import time

from .servos import SERVO_INDEX_TO_NAME, SERVO_NAME_TO_INDEX, SERVO_INDEX_TO_COMMAND
from .movements import \
    EyeMovements, EYE_MOVEMENT_TO_COMMAND, \
    HeadMovements, HEAD_MOVEMENT_TO_COMMAND, \
    ArmMovements, ARM_MOVEMENT_TO_COMMAND

###############################################################
#
# Arduino Device Class
#
###############################################################

class ArduinoDevice:
    """Class used for managing communication with the Arduino"""

    # ---------------------------------------------------------
    def __init__(self):
        """
        Constructor for Arduino serial communication thread class
        :param port:     The serial port where the Arduino is connected
        """
        self.queue: Queue = Queue()
        self.exit_flag: Event = Event()
        self.port_name: str = ""
        self.serial_port: Serial | None = None
        self.serial_thread: Thread | None = None
        self.battery_level: int | None = None
        self.init_servo_positions()
        self.exit_flag.clear()

    # ---------------------------------------------------------
    def init_servo_positions(self):
        """Init servo position list to default value"""

        self.servo_positions: list[float | None] = [None] * len(SERVO_INDEX_TO_NAME)

    # ---------------------------------------------------------
    def __del__(self):
        """Destructor - ensures serial port is closed correctly"""
        self.disconnect()

    # ---------------------------------------------------------
    def connect(self, port: str | int = "") -> bool:
        """
        Connect to the serial port
        :param port: The port to connect to (leave blank to use previous port)
        :return: True if connected successfully, False otherwise
        """
        try:
            usb_ports = [
                p.device for p in list_ports.comports()
            ]

            if type(port) is str and port == "":
                port = self.port_name

            if type(port) is int and port >= 0 and port < len(usb_ports):
                port = usb_ports[port]

            # Check port exists and we are not already connected
            if ((not self.is_connected() or port != self.port_name) and port in usb_ports):
                
               # Ensure old port is properly disconnected first
                self.disconnect() 

                # Connect to the new port
                self.serial_port = Serial(port, 115200)
                self.serial_port.flushInput()
                self.port_name = port

                # Start the command handler in a background thread
                self.exit_flag.clear()
                self.serial_thread = Thread(target = self.__communication_thread)
                self.serial_thread.start()

        except Exception as ex:
            print(f'Serial connect error: {repr(ex)}')

        return self.is_connected()

    # ---------------------------------------------------------
    def disconnect(self) -> bool:
        """
        Disconnect from the serial port
        :return: True if disconnected successfully, False otherwise
        """
        try:
            self.battery_level = None
            self.init_servo_positions()

            if self.serial_thread is not None:
                self.exit_flag.set()
                self.serial_thread.join()
                self.serial_thread = None

            if self.serial_port is not None:
                self.serial_port.close()
                self.serial_port = None

        except Exception as ex:
            print(f'Serial disconnect error: {repr(ex)}')

        return (self.serial_thread is None and self.serial_port is None)

    # ---------------------------------------------------------
    def is_connected(self) -> bool:
        """
        Check if serial device is connected
        :return: True if connected, False otherwise
        """
        return (self.serial_thread is not None and self.serial_thread.is_alive()
             and self.serial_port is not None and self.serial_port.is_open)

    # ---------------------------------------------------------
    def send_command(self, command: str) -> bool:
        """
        Send a serial command
        :param command: The command to be sent
        :return: True if port is open and message has been added to queue
        """
        success = False

        if self.is_connected():
            self.queue.put(command)
            success = True

        return success

    # ---------------------------------------------------------
    def turn(self, speed: int) -> bool:
        """
        Send a turn command
        :param speed: speed of command to set [-100, 100]
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'X{speed}')

    # ---------------------------------------------------------
    def move_linearly(self, speed: int) -> bool:
        """
        Send a linear move command
        :param speed: speed of command to set [-100, 100]
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'Y{speed}')

    # ---------------------------------------------------------
    def set_turn_offset(self, val: int) -> bool:
        """
        Set the drive motor turn offset
        :param val: value to set [-100, 100]
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'S{val}')

    # ---------------------------------------------------------
    def set_motor_deadzone(self, val: int) -> bool:
        """
        Set the drive motor deadzone
        :param val: value to set [0, 250]
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'O{val}')

    # ---------------------------------------------------------
    def play_animation(self, number: int) -> bool:
        """
        Play animation by number
        :param number: number of animation to play
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'A{number}')

    # ---------------------------------------------------------
    def activate_auto_servo_mode(self) -> bool:
        """
        Activate auto servo mode
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'M1')

    # ---------------------------------------------------------
    def deactivate_auto_servo_mode(self) -> bool:
        """
        Deactivate auto servo mode
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'M0')

    # ---------------------------------------------------------
    def move_servo_by_index(self, index: int, position: int) -> bool:
        """
        Send a manual servo command to a servo specified by index
        :param index: index of servo to command
        :param position: position to command servo to [0, 100]
        :return: True if port is open and message has been added to queue
        """
        return self.send_command(f'{SERVO_INDEX_TO_COMMAND[index]}{position}')

    # ---------------------------------------------------------
    def move_servo_by_name(self, name: str, position: int) -> bool:
        """
        Send a manual servo command to a servo specified by name
        :param name: name of servo to command
        :param position: position to command servo to [0, 100]
        :return: True if port is open and message has been added to queue
        """
        return self.move_servo_by_index(SERVO_NAME_TO_INDEX[name], position)

    # ---------------------------------------------------------
    def move_eyes(self, movement: EyeMovements | int) -> bool:
        """
        Send an eye movement command
        :param movement: enumeration of desired movement
        :return: True if port is open and message has been added to queue
        """
        if type(movement) is int:
            try:
                movement = EyeMovements(movement)
            except Exception as ex:
                print(f'EyeMovement interpretation error: {repr(ex)}')
                return False

        return self.send_command(f'{EYE_MOVEMENT_TO_COMMAND[movement]}0')

    # ---------------------------------------------------------
    def move_head(self, movement: HeadMovements | int) -> bool:
        """
        Send a head movement command
        :param movement: enumeration of desired movement
        :return: True if port is open and message has been added to queue
        """
        if type(movement) is int:
            try:
                movement = HeadMovements(movement)
            except Exception as ex:
                print(f'HeadMovement interpretation error: {repr(ex)}')
                return False

        return self.send_command(f'{HEAD_MOVEMENT_TO_COMMAND[movement]}0')

    # ---------------------------------------------------------
    def move_arms(self, movement: ArmMovements | int) -> bool:
        """
        Send an arm movement command
        :param movement: enumeration of desired movement
        :return: True if port is open and message has been added to queue
        """
        if type(movement) is int:
            try:
                movement = ArmMovements(movement)
            except Exception as ex:
                print(f'ArmMovement interpretation error: {repr(ex)}')
                return False

        return self.send_command(f'{ARM_MOVEMENT_TO_COMMAND[movement]}0')

    # ---------------------------------------------------------
    def clear_queue(self):
        """
        Clear the serial send queue
        """
        while not self.queue.empty():
            self.queue.get()

    # ---------------------------------------------------------
    def get_battery_level(self) -> int | None:
        """
        Get the robot battery level
        :return: The battery level as an integer, or None
        """
        return self.battery_level

    def get_servo_positions(self) -> list[float | None]:
        """
        Get full list of servo positions
        :return: full list of servo positions
        """
        return self.servo_positions

    # ---------------------------------------------------------
    def get_servo_position_by_index(self, index: int) -> float | None:
        """
        Get the servo position of the servo with the associated index
        :param index: The index of the servo in question
        :return: the current position of the servo
        """
        return self.servo_positions[index]

    # ---------------------------------------------------------
    def get_servo_position_by_name(self, name: str) -> float | None:
        """
        Get the servo position of the servo with the associated name
        :param name: The name of the servo in question
        :return: the current position of the servo
        """
        return self.get_servo_position_by_index(SERVO_NAME_TO_INDEX[name])

    # ---------------------------------------------------------
    def __communication_thread(self):
        """
        Handle sending and receiving data with the serial device
        """
        dataString: str = ""
        print(f'Starting Arduino Thread ({self.port_name})')

        # Keep this thread running until the exit_flag changes
        while not self.exit_flag.is_set():
            try:
                # If there are any messages in the queue, send them
                if not self.queue.empty():
                    data = self.queue.get() + '\n'
                    self.serial_port.write(data.encode())

                # Read any incomming messages
                while (self.serial_port.in_waiting > 0):
                    data = self.serial_port.read()
                    if (data.decode() == '\n' or data.decode() == '\r'):
                        self.__parse_message(dataString)
                        dataString = ""
                    else:
                        dataString += data.decode()

            # If an error occured in the serial communication
            except Exception as ex:
                print(f'Serial handler error: {repr(ex)}')
                #exit_flag.set()

            time.sleep(0.01)
        
        print(f'Stopping Arduino Thread ({self.port_name})')

    # ---------------------------------------------------------
    def __parse_message(self, dataString: str):
        """
        Parse messages received from the connected device
        :param dataString: String containing the serial message to be parsed
        """
        try:
            # Servo position feedback
            # Format "Servo_<INDEX>_<VALUE>"
            if "Servo" in dataString:
                dataList = dataString.split('_')
                self.servo_positions[int(dataList[1])] = float(dataList[2])
            # Battery level message
            # Format "Battery_<VALUE>"
            elif "Battery" in dataString:
                dataList = dataString.split('_')
                self.battery_level = int(dataList[1])

        except Exception as ex:
            print(f'Error parsing message [{dataString}]: {repr(ex)}')

# End of class: ArduinoDevice