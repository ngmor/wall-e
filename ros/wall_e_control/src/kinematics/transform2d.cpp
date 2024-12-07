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
/// @brief 2D transform class source.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/src/rigid2d.cpp

#include <wall_e_control/kinematics/transform2d.hpp>

#include <iostream>
#include <cmath>

#include <wall_e_control/kinematics/common.hpp>

namespace wall_e
{

namespace kinematics
{

Transform2D::Transform2D(): Transform2D({0.0, 0.0}, 0.0) {}

Transform2D::Transform2D(const Vector2D& trans): Transform2D(trans, 0.0) {}

Transform2D::Transform2D(double rot): Transform2D({0.0, 0.0}, rot) {}

Transform2D::Transform2D(const Vector2D& trans, double rot)
: trans_{trans}, rot_{normalize_angle(rot)}
{
  cache_trig();
}

void Transform2D::cache_trig()
{
  rot_sin_ = std::sin(rot_);
  rot_cos_ = std::cos(rot_);
}

Vector2D Transform2D::operator()(const Vector2D& v) const
{
  return {
    v.x*rot_cos_ - v.y*rot_sin_ + trans_.x,
    v.x*rot_sin_ + v.y*rot_cos_ + trans_.y
  };
}

Twist2D Transform2D::operator()(const Twist2D& V) const
{
  return {
    V.w,
    V.w*trans_.y + V.x*rot_cos_ - V.y*rot_sin_,
    -V.w*trans_.x + V.x*rot_sin_ + V.y*rot_cos_
  };
}

Transform2D Transform2D::inv() const
{
  return {
    //translation
    {
      -trans_.x*rot_cos_ - trans_.y*rot_sin_,
      -trans_.y*rot_cos_ + trans_.x*rot_sin_
    },
    //rotation
    -rot_
  };
}

Transform2D& Transform2D::operator*=(const Transform2D& rhs)
{
  //Output translation adds current translation to incoming translation
  //modified by current rotation
  trans_.x += rhs.translation().x*rot_cos_
            - rhs.translation().y*rot_sin_;
  trans_.y += rhs.translation().x*rot_sin_
            + rhs.translation().y*rot_cos_;

  //Output rotation just adds the angles together
  rot_ += rhs.rotation();

  //Bound rotation
  rot_ = normalize_angle(rot_);

  //Cache trig values
  cache_trig();

  return *this;
}

Transform2D operator*(const Transform2D& lhs, const Transform2D& rhs) {
  auto out = lhs;
  out*=rhs;
  return out;
}

Vector2D Transform2D::translation() const { return trans_; }

double Transform2D::rotation() const { return rot_; }

std::ostream& operator<<(std::ostream& os, const Transform2D& tf)
{
  os << "deg: " << rad2deg(tf.rotation())
      << " x: " << tf.translation().x
      << " y: " << tf.translation().y;
  return os;
}

std::istream& operator>>(std::istream& is, Transform2D& tf)
{
  double deg, x, y;

  //remove leading whitespace
  is >> std::ws;

  auto c = is.peek();

  //remove any characters that aren't digits
  while (!(std::isdigit(c) || (c == '-') || (c == '.')))
  {
    c = is.get();
    c = is.peek();
  }

  //Get rotation
  is >> deg;


  c = is.peek();

  //remove any characters that aren't digits
  while (!(std::isdigit(c) || (c == '-') || (c == '.')))
  {
    c = is.get();
    c = is.peek();
  }

  //Get x translation
  is >> x;


  c = is.peek();

  //remove any characters that aren't digits
  while (!(std::isdigit(c) || (c == '-') || (c == '.')))
  {
    c = is.get();
    c = is.peek();
  }

  //Get y translation
  is >> y;


  //Init transform
  tf = Transform2D{
      {x,y},
      deg2rad(deg)
  };

  return is;
}

bool almost_equal(const Transform2D& T1, const Transform2D& T2, double epsilon)
{
  return almost_equal(T1.translation(), T2.translation(), epsilon)
      && almost_equal(T1.rotation(), T2.rotation(), epsilon);
}

Transform2D integrate_twist(const Twist2D& twist)
{
  if (almost_equal(twist.w, 0.0))
  {
    return {
      {twist.x, twist.y},
      0.0
    };
  }
  else
  {
    //Frames:
    //b = body frame before motion
    //bp = body frame after motion
    //s = frame at center of rotation (COR) aligned with b frame
    //sp = frame at center of rotation (COR) aligned with bp frame

    //Purely rotational motion at COR to get from s to sp
    Transform2D Tssp {twist.w};

    //Use adjoint between known twists in s and b frames to solve for Tsb
    Transform2D Tsb {
      {
        twist.y / twist.w,
        -twist.x / twist.w
      },
      0.0 //no rotation between s and b frames, they are aligned
    };

    //Tbbp = TbsTsspTspbp
    //Tsb = Tspbp
    //so
    //Tbbp = Tsb^-1 * Tssp * Tsb
    return Tsb.inv()*Tssp*Tsb;
  }
}

}  // namespace kinematics

}  // namespace wall_e