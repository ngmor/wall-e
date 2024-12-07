// Copyright 2024 Nick Morales.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief Common kinematics functions/variables source.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/src/rigid2d.cpp

#include <wall_e_control/kinematics/common.hpp>

namespace wall_e
{

namespace kinematics
{

double normalize_angle(double rad)
{
  //Bound rotation between -pi and pi
  while (rad > PI)
  {
    rad -= 2.0*PI;
  }
  while (rad <= -PI)
  {
    rad += 2.0*PI;
  }

  return rad;
}

}  // namespace kinematics

}  // namespace wall_e