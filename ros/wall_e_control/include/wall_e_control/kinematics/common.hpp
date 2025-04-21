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
/// @brief Common kinematics functions/variables header.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/include/turtlelib/rigid2d.hpp

#pragma once

#include<limits>
#include<cmath>

namespace wall_e
{

namespace kinematics
{

/// @brief PI.  Not in C++ standard until C++20.
constexpr double PI=3.14159265358979323846;

/// @brief infinity
constexpr double INF=std::numeric_limits<double>::infinity();

/// @brief approximately compare two floating-point numbers using
/// an absolute comparison
/// @param d1 a number to compare
/// @param d2 a second number to compare
/// @param epsilon absolute threshold required for equality
/// @return true if abs(d1 - d2) < epsilon
constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
{
  return std::abs(d1 - d2) < epsilon;
}

/// @brief convert degrees to radians
/// @param deg angle in degrees
/// @returns radians
constexpr double deg2rad(double deg)
{
  return deg * PI / 180.0;
}

/// @brief convert radians to degrees
/// @param rad angle in radians
/// @returns the angle in degrees
constexpr double rad2deg(double rad)
{
  return rad * 180.0 / PI;
}

/// @brief determine the sign of an input value
/// @tparam T type of input variable
/// @param val value to check type of
/// @return -1 for negative, 0 for 0, 1 for positive
/// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

/// @brief bound an angle to equivalent angle between (-PI, PI]
/// @param rad angle to normalize
/// @return normalized angle
double normalize_angle(double rad);

}  // namespace kinematics

}  // namespace wall_e