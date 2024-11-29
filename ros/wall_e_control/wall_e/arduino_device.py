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
        self.battery_level: str | None = None
        self.exit_flag.clear()

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
    def clear_queue(self):
        """
        Clear the serial send queue
        """
        while not self.queue.empty():
            self.queue.get()

    # ---------------------------------------------------------
    def get_battery_level(self) -> str | None:
        """
        Get the robot battery level
        :return: The battery level as a string, or None
        """
        return self.battery_level

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
            # Battery level message
            if "Battery" in dataString:
                dataList = dataString.split('_')
                if len(dataList) > 1 and dataList[1].isdigit():
                    self.battery_level = dataList[1]

        except Exception as ex:
            print(f'Error parsing message [{dataString}]: {repr(ex)}')

# End of class: ArduinoDevice