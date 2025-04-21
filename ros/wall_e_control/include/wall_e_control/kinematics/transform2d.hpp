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
/// @brief 2D transform class header.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/include/turtlelib/rigid2d.hpp

#pragma once

#include <iosfwd>

#include <wall_e_control/kinematics/vector2d.hpp>
#include <wall_e_control/kinematics/twist2d.hpp>

namespace wall_e
{

namespace kinematics
{

/// @brief a rigid body transformation in 2 dimensions
class Transform2D
{
private:
  /// @brief translational component
  Vector2D trans_ {0., 0.};

  /// @brief rotational component
  double rot_ = 0.;

  /// @brief cached sine of rotational angle
  double rot_sin_ = 0.;

  /// @brief cached cosine of rotational angle
  double rot_cos_ = 1.;

  /// @brief calculate and store trigonmetric values from rotational component
  /// to increase computational efficiency
  void cache_trig();

public:
  /// @brief Create an identity transformation
  Transform2D();

  /// @brief create a transformation that is a pure translation
  /// @param trans the vector by which to translate
  explicit Transform2D(const Vector2D& trans);

  /// @brief create a pure rotation
  /// @param rot angle of the rotation, in radians
  explicit Transform2D(double rot);

  /// @brief Create a transformation with a translational and rotational
  /// component
  /// @param trans the translation
  /// @param rot the rotation, in radians
  Transform2D(const Vector2D& trans, double rot);

  /// @brief apply a transformation to a Vector2D
  /// @param v the vector to transform
  /// @return a vector in the new coordinate system
  Vector2D operator()(const Vector2D& v) const;

  /// @brief apply a transformation to a Twist2D
  /// @param V the twist to transform
  /// @return a twist in the new coordinate system
  Twist2D operator()(const Twist2D& V) const;

  /// @brief invert the transformation
  /// @return the inverse transformation. 
  Transform2D inv() const;

  /// @brief compose this transform with another and store the result 
  /// in this object
  /// @param rhs the first transform to apply
  /// @return a reference to the newly transformed operator
  Transform2D& operator*=(const Transform2D& rhs);

  /// @brief the translational component of the transform
  /// @return the x,y translation
  Vector2D translation() const;

  /// @brief get the angular displacement of the transform
  /// @return the angular displacement, in radians
  double rotation() const;

  /// @brief print a human readable version of the transform:
  /// An example output:
  /// deg: 90 x: 3 y: 5
  /// @param os an output stream
  /// @param tf the transform to print
  friend std::ostream& operator<<(std::ostream& os, const Transform2D& tf);
};

/// @brief Read a transformation from stdin
/// Able to read input either as output by operator<< or
/// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
/// For example:
/// 90 2 3
std::istream& operator>>(std::istream& is, Transform2D& tf);

/// @brief multiply two transforms together, returning their composition
/// @param lhs the left hand operand
/// @param rhs the right hand operand
/// @return the composition of the two transforms
Transform2D operator*(const Transform2D& lhs, const Transform2D& rhs);

/// @brief approximately compare two Transform2Ds for equivalence
/// @param T1 first transform
/// @param T2 second transform
/// @param epsilon absolute threshold required for equality
/// @return true if both translation and rotational components are within the epsilon threshold
/// of eachother
bool almost_equal(const Transform2D& T1, const Transform2D& T2, double epsilon=1.0e-12);

/// @brief compute the transformation corresponding to a rigid body following
/// a constant twist (in its original body frame) for one time unit
/// @param twist twist to follow
/// @return transformation from original body frame to frame after following twist
Transform2D integrate_twist(const Twist2D& twist);

}  // namespace kinematics

}  // namespace wall_e