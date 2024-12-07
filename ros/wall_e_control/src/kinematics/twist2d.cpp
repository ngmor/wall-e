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
/// @brief 2D twist struct source.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/src/rigid2d.cpp

#include <wall_e_control/kinematics/twist2d.hpp>

#include <iostream>

namespace wall_e
{

namespace kinematics
{

std::ostream & operator<<(std::ostream & os, const Twist2D & V) 
{
  os << '[' << V.w << ' ' << V.x << ' ' << V.y << ']';
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & V) 
{
  //remove leading whitespace
  is >> std::ws;

  auto c = is.peek();

  if (c == '[')
  {
    //Remove leading bracket
    c = is.get();
  }

  //Get actual data
  is >> V.w >> V.x >> V.y;

  c = is.peek();

  if (c == ']')
  {
    //Remove trailing bracket
    c = is.get();
  }

  return is;
}

}  // namespace kinematics

}  // namespace wall_e