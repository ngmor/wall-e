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
/// @brief 2D vector struct source.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/src/rigid2d.cpp

#include <wall_e_control/kinematics/vector2d.hpp>

#include <cmath>
#include <iostream>

#include <wall_e_control/kinematics/common.hpp>

namespace wall_e
{

namespace kinematics
{

double Vector2D::magnitude() const
{
  return std::sqrt(std::pow(x,2) + std::pow(y,2));
}

Vector2D& Vector2D::operator+=(const Vector2D& rhs)
{
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs)
{
  auto out = lhs;
  out += rhs;
  return out;
}

Vector2D& Vector2D::operator-=(const Vector2D& rhs)
{
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}

Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs)
{
  auto out = lhs;
  out -= rhs;
  return out;
}

Vector2D & Vector2D::operator*=(double scalar)
{
  x *= scalar;
  y *= scalar;
  return *this;
}

Vector2D operator*(const Vector2D& vector, double scalar)
{
  auto out = vector;
  out*=scalar;
  return out;
}

Vector2D operator*(double scalar, const Vector2D& vector)
{
  return vector*scalar;
}

std::ostream& operator<<(std::ostream& os, const Vector2D& v)
{
  os << '[' << v.x << ' ' << v.y << ']';
  return os;
}

std::istream& operator>>(std::istream& is, Vector2D& v)
{
  //remove leading whitespace
  is >> std::ws;

  auto c = is.peek();

  if (c == '[') {
    //Remove leading bracket
    c = is.get();
  }

  //Get actual data
  is >> v.x >> v.y;

  c = is.peek();

  if (c == ']') {
    //Remove trailing bracket
    c = is.get();
  }

  return is;
}

bool almost_equal(const Vector2D& v1, const Vector2D& v2, double epsilon)
{
  return almost_equal(v1.x, v2.x, epsilon) && almost_equal(v1.y, v2.y, epsilon);
}

Vector2D normalize(const Vector2D& v)
{
  const auto mag = v.magnitude();

  return {
    v.x / mag,
    v.y / mag
  };
}

double dot(const Vector2D& lhs, const Vector2D& rhs)
{
  return lhs.x*rhs.x + lhs.y*rhs.y;
}

double angle_between(const Vector2D& start, const Vector2D& end)
{
  //https://www.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
  return std::atan2(start.x*end.y - start.y*end.x, dot(start, end));
}

}  // namespace kinematics

}  // namespace wall_e