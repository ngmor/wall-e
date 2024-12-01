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

from enum import Enum

###############################################################
#
# Movement Mappings
#
###############################################################

class EyeMovements(Enum):
    NEUTRAL         = 0
    SAD             = 1
    LEFT_TILT       = 2
    RIGHT_TILT      = 3

EYE_MOVEMENT_TO_COMMAND = {
    EyeMovements.NEUTRAL:       'k',
    EyeMovements.SAD:           'i',
    EyeMovements.LEFT_TILT:     'j',
    EyeMovements.RIGHT_TILT:    'l',
}

class HeadMovements(Enum):
    FORWARD     = 0
    UP          = 1
    DOWN        = 2

HEAD_MOVEMENT_TO_COMMAND = {
    HeadMovements.FORWARD:      'g',
    HeadMovements.UP:           'f',
    HeadMovements.DOWN:         'h',
}

class ArmMovements(Enum):
    NEUTRAL             = 0
    LEFT_LOW_RIGHT_HIGH = 1
    LEFT_HIGH_RIGHT_LOW = 2

ARM_MOVEMENT_TO_COMMAND = {
    ArmMovements.NEUTRAL:               'm',
    ArmMovements.LEFT_LOW_RIGHT_HIGH:   'b',
    ArmMovements.LEFT_HIGH_RIGHT_LOW:   'n',
}