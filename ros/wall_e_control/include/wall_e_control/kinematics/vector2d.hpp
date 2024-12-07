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
/// @brief 2D vector struct header.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/include/turtlelib/rigid2d.hpp

#pragma once

#include<iosfwd>

namespace wall_e
{

namespace kinematics
{

/// @brief A 2-Dimensional Vector
struct Vector2D
{
  /// @brief the x coordinate
  double x = 0.0;

  /// @brief the y coordinate
  double y = 0.0;

  /// @brief compute the magnitude of the vector
  /// @return the magnitude of the vector
  double magnitude() const;

  /// @brief add this vector with another and store the result 
  /// in this object
  /// @param rhs the vector to add
  /// @return a reference to the newly added vector
  Vector2D& operator+=(const Vector2D& rhs);

  /// @brief subtract another vector from this vector and store
  /// the result in this object
  /// @param rhs the vector to subtract
  /// @return a reference to the newly subtracted vector
  Vector2D& operator-=(const Vector2D& rhs);

  /// @brief multiply a vector by a scalar and store
  /// the result in this object
  /// @param scalar the scalar to multiply by
  /// @return a reference to the newly multiplied vector
  Vector2D& operator*=(double scalar);
};

/// @brief add two vectors together, returning their sum
/// @param lhs the left hand operand
/// @param rhs the right hand operand
/// @return the sum of the two vectors
Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs);

/// @brief subtract a vector from another, returning the difference
/// @param lhs the left hand operand
/// @param rhs the right hand operand
/// @return the difference of the two vectors
Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs);

/// @brief multiply a vector by a scalar, returning the scaled vector
/// @param vector the vector to be multiplied
/// @param scalar the scalar to multiply by
/// @return the scaled vector
Vector2D operator*(const Vector2D& vector, double scalar);

/// @brief multiply a vector by a scalar, returning the scaled vector
/// @param scalar the scalar to multiply by
/// @param vector the vector to be multiplied
/// @return the scaled vector
Vector2D operator*(double scalar, const Vector2D& vector);

/// @brief output a 2 dimensional vector as [xcomponent ycomponent]
/// @param os stream to output to
/// @param v the vector to print
std::ostream& operator<<(std::ostream& os, const Vector2D& v);

/// @brief input a 2 dimensional vector
/// Should be able to read vectors entered as follows:
/// [x y] or x y
/// @param is stream from which to read
/// @param v [out] output vector
std::istream& operator>>(std::istream& is, Vector2D& v);

/// @brief approximately compare two Vector2Ds for equivalence
/// @param v1 first vector
/// @param v2 second vector
/// @param epsilon absolute threshold required for equality
/// @return true if both x and y components are are within the epsilon threshold of eachother
bool almost_equal(const Vector2D& v1, const Vector2D& v2, double epsilon=1.0e-12);

/// @brief normalize a 2 dimensional vector
/// @param v the vector to normalize
Vector2D normalize(const Vector2D& v);

/// @brief compute the dot product of two vectors
/// @param lhs the left hand operand
/// @param rhs the right hand operand
/// @return the dot product
double dot(const Vector2D& lhs, const Vector2D& rhs);

/// @brief compute the angle between two vectors, with directional information
/// (sign) based on the order in which the vectors are passed into the function
/// @param start the vector from which the measurement is started
/// @param end the vector to which the measurement goes
/// @return the angle between the vectors, signed positive for CCW motion and
/// negative for CW motion
double angle_between(const Vector2D& start, const Vector2D& end);

}  // namespace kinematics

}  // namespace wall_e