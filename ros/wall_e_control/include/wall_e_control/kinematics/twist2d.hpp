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
/// @brief 2D twist struct header.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/include/turtlelib/rigid2d.hpp

#pragma once

#include <iosfwd>

namespace wall_e
{

namespace kinematics
{

/// @brief A 2-Dimensional Twist
struct Twist2D
{
  /// @brief the angular velocity
  double w = 0.0;

  /// @brief the linear x velocity
  double x = 0.0;

  /// @brief the lineary y velocity
  double y = 0.0;
};

/// @brief output a 2 dimensional twist as [wcomponent xcomponent ycomponent]
/// @param os stream to output to
/// @param V the twist to print
std::ostream& operator<<(std::ostream& os, const Twist2D& V);

/// @brief input a 2 dimensional twist
/// Should be able to read vectors entered as follows:
/// [w x y] or w x y
/// @param is stream from which to read
/// @param V [out] output twist
std::istream& operator>>(std::istream& is, Twist2D& V);

}  // namespace kinematics

}  // namespace wall_e