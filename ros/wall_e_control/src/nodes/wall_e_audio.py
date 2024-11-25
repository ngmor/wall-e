#!/usr/bin/env python3

import subprocess

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from wall_e_interfaces.srv import GetSounds, PlaySound, SetVolume


SET_VOLUME_COMMAND = ['amixer', '-c0', '-M', '-q', 'sset', 'Headphone']
PLAY_SOUND_COMMAND = ['ffplay', '-v', '0', '-nodisp', '-autoexit']
SOUND_DIRECTORY = get_package_share_directory('wall_e_control') + '/sounds/'

class WALLEAudio(Node):

    def __init__(self):
        """Initialize the node."""

        super().__init__('wall_e_audio')

        # PARAMETERS ---------------------------------------------------------------------------
        self.declare_parameter(
            'default_volume',
            100,
            ParameterDescriptor(
                description='Default volume to set on startup (%). If negative, the default will'
                ' not be set on startup. Otherwise clamped between [0, 100]'
            )
        )
        self.declare_parameter(
            'sounds.names',
            ['none'],
            ParameterDescriptor(
                description='List of names of sounds that will be used to play them.'
                + ' Corresponds to sounds.files and lengths must match'
            )
        )
        sound_names = self.get_parameter('sounds.names').get_parameter_value().string_array_value
        self.declare_parameter(
            'sounds.files',
            ['none'],
            ParameterDescriptor(
                description='List of files corresponding to the names in sound.names.'
                + ' These files reside in the wall_e_control package share directory,'
                + ' "sounds" subdirectory. Length must match sounds.names'
            )
        )
        sound_files = self.get_parameter('sounds.files').get_parameter_value().string_array_value

        if len(sound_names) != len(sound_files):
            raise RuntimeError(
                'Sound names and files arrays do not have the same length,'
                + ' likely due to a malformed parameter file'
            )

        # Construct sound dictionary, pointing to sound files
        self.sounds = dict()
        for i in range(len(sound_names)):
            self.sounds[sound_names[i]] = SOUND_DIRECTORY + sound_files[i]

        # SERVICES ---------------------------------------------------------------------------
        self.srv_get_sounds = self.create_service(
            GetSounds,
            'get_sounds',
            self.srv_get_sounds_callback
        )
        self.srv_play_sound = self.create_service(
            PlaySound,
            'play_sound',
            self.srv_play_sound_callback
        )
        self.srv_set_volume = self.create_service(
            SetVolume,
            'set_volume',
            self.srv_set_volume_callback
        )

        # set the default volume
        self.set_volume(self.get_parameter('default_volume').get_parameter_value().integer_value)

        self.get_logger().info(f'{self.get_fully_qualified_name()} node started')

    def set_volume(self, volume: int):
        """
        Set volume to the specified integer percentage.

        Args:
            volume (int): percentage volumte to set
        """

        if volume < 0:
            return

        # clamp volume to [0, 100]
        volume = max(0, min(100, volume))

        cmd = SET_VOLUME_COMMAND + [f'{volume}%']

        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.get_logger().info(f'Volume set to {volume}%')

    def get_available_sounds(self):
        """Compile list of available sounds."""

        return [name for name in self.sounds.keys()]

    def srv_get_sounds_callback(self, request, response):
        """
        Get a list of available sounds by name.

        Args:
            request (GetSounds.Request): service request, empty
            response (GetSounds.Response): service response, contains list of sound names

        Returns:
            response: filled out response
        """

        response.sounds = self.get_available_sounds()

        self.get_logger().info(f'Available sounds: {response.sounds}')

        return response

    def srv_play_sound_callback(self, request, response):
        """
        Play a configured sound.

        Args:
            request (PlaySound.Request): service request with name of sound to play
            response (PlaySound.Response): service response, indicating success/failure

        Returns:
            response: filled out response
        """

        if request.sound not in self.sounds:
            self.get_logger().error(
                f'"{request.sound}" not available as a configured sound.'
                + f' Available sounds: {self.get_available_sounds()}'
            )
            response.success = False
            return response

        cmd = PLAY_SOUND_COMMAND + [self.sounds[request.sound]]

        subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        response.success = True

        return response

    def srv_set_volume_callback(self, request, response):
        """
        Set the volume via the service

        Args:
            request (SetVolume.Request): service request, has volume argument
            response (SetVolume.Response): service response, empty

        Returns:
            response: empty response
        """

        self.set_volume(request.volume)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = WALLEAudio()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()