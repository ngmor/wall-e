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
/// @brief Differential drive kinematics class header.
/// Adapted from https://github.com/ngmor/turtlebot3-ekf-slam/blob/main/turtlelib/include/turtlelib/diff_drive.hpp

#pragma once

#include <wall_e_control/kinematics/vector2d.hpp>
#include <wall_e_control/kinematics/transform2d.hpp>

namespace wall_e
{

namespace kinematics
{

/// @brief wheel data (positions, velocities, etc)
struct Wheel
{
  /// @brief left wheel data
  double left = 0.0;

  /// @brief right wheel data
  double right = 0.0;
};

/// @brief differential drive robot
class DiffDrive
{
public:

  /// @brief Pose/joint configuration of a differential drive robot
  struct Config
  {
    /// @brief pose of robot in the world frame
    Transform2D pose {Vector2D{0.0, 0.0}, 0.0};

    /// @brief wheel positions
    Wheel wheel_pos {0.0, 0.0};
  };

  /// @brief create a differential drive robot starting at the origin
  /// @param wheel_track distance between the robot's wheels, must be > 0
  /// @param wheel_radius wheel radius, must be > 0
  DiffDrive(double wheel_track, double wheel_radius);

  /// @brief create a differential drive robot with a custom starting pose
  /// @param wheel_track distance between the robot's wheels, must be > 0
  /// @param wheel_radius wheel radius, must be > 0
  /// @param start_pose starting pose transformation
  DiffDrive(double wheel_track, double wheel_radius, const Transform2D& start_pose);

  /// @brief create a differential drive robot with custom starting wheel positions
  /// @param wheel_track distance between the robot's wheels, must be > 0
  /// @param wheel_radius wheel radius, must be > 0
  /// @param start_wheel_pos starting position of the wheels
  DiffDrive(double wheel_track, double wheel_radius, const Wheel& start_wheel_pos);
  
  /// @brief create a differential drive robot with a custom starting configuration
  /// @param wheel_track distance between the robot's wheels, must be > 0
  /// @param wheel_radius wheel radius, must be > 0
  /// @param start_config starting configuration of the robot
  DiffDrive(double wheel_track, double wheel_radius, const Config& start_config);

  /// @brief reset current configuration to initial configuration
  void reset();

  /// @brief the current config of the robot
  /// @return the current config structure
  const Config& config() const;

  /// @brief overwrite the current config with a new config
  /// @param new_config new config to overwrite with
  void set_config(const Config& new_config);

  /// @brief overwrite the current pose with a new pose
  /// @param new_pose new pose to overwrite with
  void set_pose(const Transform2D& new_pose);

  /// @brief overwrite the current wheel positions with new positions
  /// @param new_wheel_pos new wheel positions to overwrite with
  void set_wheel_pos(const Wheel& new_wheel_pos);

  /// @brief calculate the body twist produced by a change in wheel position in unit time
  /// @param new_wheel_pos the new wheel position,
  /// change from current position is calculated
  /// @return the produced body twist
  Twist2D get_body_twist(const Wheel& new_wheel_pos) const;

  /// @brief use forward kinematics to update the configuration of the robot
  // given new wheel positions
  /// @param new_wheel_pos new wheel positions with which to calculate new configuration
  /// @return the updated config (also stored internally)
  const Config& update_config(const Wheel& new_wheel_pos);

  /// @brief use inverse kinematics to get the required wheel velocities to
  /// produce a given body twist
  /// @param twist the requested body twist
  /// @return wheel velocities
  Wheel get_required_wheel_vel(const Twist2D& twist) const;

private:

  /// @brief distance between the robot's wheels, must be > 0
  double wheel_track_ = 0.0;

  /// @brief wheel radius, must be > 0
  double wheel_radius_ = 0.0;

  /// @brief initial configuration
  Config config_init_ {{Vector2D{0., 0. }, 0.}, Wheel{0., 0.}};

  /// @brief current configuration
  Config config_ {{Vector2D{0., 0.}, 0.}, Wheel{0., 0.}};

  /// @brief inverse kinematics angular velocity coefficient
  double coeff_ik_w_ = 0.0;

  /// @brief inverse kinematics linear x velocity coefficient
  double coeff_ik_x_ = 0.0;

  /// @brief forward kinematics angular velocity coefficient
  double coeff_fk_w_ = 0.0;

  /// @brief forward kinematics linear x velocity coefficient 
  double coeff_fk_x_ = 0.0;

  /// @brief calculate and store coefficients for kinematics
  /// based on robot's physical parameters
  void calc_kinematic_coeff();

};

}  // namespace kinematics

}  // namespace wall_e