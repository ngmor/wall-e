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

###############################################################
#
# Servo Mappings
#
###############################################################

# Mapping from servo index to servo joint name
SERVO_INDEX_TO_NAME = [
    'head_center',
    'neck_top',
    'neck_bottom',
    'eye_right',
    'eye_left',
    'arm_left',
    'arm_right',
]

# Number of servos
NUM_SERVOS = len(SERVO_INDEX_TO_NAME)

# Mapping from servo joint name to servo index
SERVO_NAME_TO_INDEX = dict()

for i, servo in enumerate(SERVO_INDEX_TO_NAME):
    SERVO_NAME_TO_INDEX[servo] = i

# Mapping from servo index to servo manual command
SERVO_INDEX_TO_COMMAND = [
    'G', # head_center
    'T', # neck_top
    'B', # neck_bottom
    'U', # eye_right
    'E', # eye_left
    'L', # arm_left
    'R', # arm_right
]