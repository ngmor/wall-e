#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from wall_e_interfaces.srv import GetSounds, PlaySound, SetVolume

class WALLEAudio(Node):

    def __init__(self):
        """Initialize the node."""

        super().__init__("wall_e_audio")


def main(args=None):
    rclpy.init(args=args)
    node = WALLEAudio()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()