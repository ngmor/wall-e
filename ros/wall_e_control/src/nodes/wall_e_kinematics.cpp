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
/// @brief Differential drive kinematics control for the WALL-E robot

#include <cmath>
#include <algorithm>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <wall_e_control/kinematics/diff_drive.hpp>

#include <geometry_msgs/msg/twist.hpp>

// Motor controller specific includes
#ifdef USE_ROBOCLAW
// TODO fix when roboclaw messages are defined
#include <std_msgs/msg/empty.hpp>
#else
#include <wall_e_interfaces/msg/drive_motor_speed.hpp>
#endif

using wall_e::kinematics::DiffDrive;

class WALLEKinematics : public rclcpp::Node
{
private:


  // CLASS MEMBERS --------------------------------------------------------------------------------

  // ROS objects
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_ = nullptr;

  DiffDrive diff_drive_ {0.1, 0.1};  // values will be overwritten in class constructor
  double motor_speed_max_ = 0.0;  // rad/s

  // Motor controller specific members
#ifdef USE_ROBOCLAW
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_wheel_left_ = nullptr; // TODO fix when roboclaw messages are defined
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_wheel_right_ = nullptr; // TODO fix when roboclaw messages are defined
#else
  rclcpp::Publisher<wall_e_interfaces::msg::DriveMotorSpeed>::SharedPtr pub_linear_speed_ = nullptr;
  rclcpp::Publisher<wall_e_interfaces::msg::DriveMotorSpeed>::SharedPtr pub_turn_speed_ = nullptr;
#endif

public:


  // INITIALIZATION --------------------------------------------------------------------------------

  /// @brief iniitialize the node
  WALLEKinematics()
  : Node("wall_e_kinematics")
  {

    // PARAMETERS ------------------------------------------------------------
    rcl_interfaces::msg::ParameterDescriptor desc;

    // check if parameters were valid
    bool parameters_valid = true;

    desc.description = "The distance (m) between the centers of the tracks";
    declare_parameter("track_width", 0.0, desc); // TODO update with actual value
    const auto track_width = get_parameter("track_width").get_parameter_value().get<double>();
    if (track_width <= 0.0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid track width provided: " << track_width);
      parameters_valid = false;
    }

    desc.description = "The distance (m) from the drive shaft center to the bottom of the track";
    declare_parameter("drive_radius", 0.0, desc); // TODO update with actual value
    const auto drive_radius = get_parameter("drive_radius").get_parameter_value().get<double>();
    if(drive_radius <= 0.0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid drive radius provided: " << drive_radius);
      parameters_valid = false;
    }

    desc.description = "The max speed (rad/s) of the motors in rad/sec";
    declare_parameter("motor_speed_max", 0.0, desc); // TODO update with actual value
    motor_speed_max_ = get_parameter("motor_speed_max").get_parameter_value().get<double>();
    if (motor_speed_max_ <= 0.0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid max motor speed provided: " << motor_speed_max_);
      parameters_valid = false;
    }

    //Abort if any parameters were invalid
    if (!parameters_valid) {
      throw std::logic_error("One or more parameters were invalid");
    }

    // SUBSCRIBERS ------------------------------------------------------------
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS{10},
      std::bind(&WALLEKinematics::sub_cmd_vel_callback, this, std::placeholders::_1)
    );

    // MOTOR CONTROLLER SPECIFIC INITS ------------------------------------------------------------
  #ifdef USE_ROBOCLAW
    pub_wheel_left_ = create_publisher<std_msgs::msg::Empty>(  // TODO fix when roboclaw messages are defined
      "drive/left/velocity_setpoint",
      rclcpp::QoS{10}
    );
    pub_wheel_right_ = create_publisher<std_msgs::msg::Empty>(  // TODO fix when roboclaw messages are defined
      "drive/right/velocity_setpoint",
      rclcpp::QoS{10}
    );
    RCLCPP_INFO_STREAM(get_logger(),
      get_fully_qualified_name() << " node configured to use RoboClaw motor controller"
    );
  #else
    pub_linear_speed_ = create_publisher<wall_e_interfaces::msg::DriveMotorSpeed>(
      "linear_speed",
      rclcpp::QoS{10}
    );
    pub_turn_speed_ = create_publisher<wall_e_interfaces::msg::DriveMotorSpeed>(
      "turn_speed",
      rclcpp::QoS{10}
    );
    RCLCPP_INFO_STREAM(get_logger(),
      get_fully_qualified_name() << " node configured to use Arduino motor shield"
    );
  #endif

    diff_drive_ = DiffDrive{track_width, drive_radius};

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }

private:

  // SUBSCRIBERS --------------------------------------------------------------------------------

  /// @brief Receive commanded twists, convert them to wheel velocities via
  /// differential drive inverse kinematics, and publish them to wheel command topics
  void sub_cmd_vel_callback(const geometry_msgs::msg::Twist& msg)
  {
    // Use inverse kinematics to calculate the required wheel veloctiies for the
    // input twist
    auto wheel_vel = diff_drive_.get_required_wheel_vel({
      msg.angular.z,
      msg.linear.x,
      msg.linear.y
    });

  #ifndef USE_ROBOCLAW
    if ((msg.linear.x != 0.0) && (msg.angular.z != 0.0))
    {
      RCLCPP_WARN_STREAM(get_logger(), 
        "Arduino motor shield does not support combined"
        " linear/angular velocity, ignoring angular component"
      );
      wheel_vel = diff_drive_.get_required_wheel_vel({
        0.0,
        msg.linear.x,
        msg.linear.y
      });
    }
  #endif

    // Clamp wheel velocities
    if (std::abs(wheel_vel.left) > motor_speed_max_)
    {
      RCLCPP_WARN_STREAM(get_logger(),
        "Desired left wheel velocity (" << wheel_vel.left << ") exceeds range ["
        << -motor_speed_max_ << ", " << motor_speed_max_ << "], clamping"
      );
      wheel_vel.left = std::clamp(wheel_vel.left, -motor_speed_max_, motor_speed_max_);
    }
    if (std::abs(wheel_vel.right) > motor_speed_max_)
    {
      RCLCPP_WARN_STREAM(get_logger(),
        "Desired right wheel velocity (" << wheel_vel.right << ") exceeds range ["
        << -motor_speed_max_ << ", " << motor_speed_max_ << "], clamping"
      );
      wheel_vel.right = std::clamp(wheel_vel.right, -motor_speed_max_, motor_speed_max_);
    }

    // Construct wheel commands
  #ifdef USE_ROBOCLAW
    std_msgs::msg::Empty wheel_cmd_left; // TODO fix when roboclaw messages are defined
    std_msgs::msg::Empty wheel_cmd_right; // TODO fix when roboclaw messages are defined
    pub_wheel_left_->publish(wheel_cmd_left);
    pub_wheel_right_->publish(wheel_cmd_right);
  #else
    wall_e_interfaces::msg::DriveMotorSpeed wheel_cmd;
    if (msg.linear.x != 0.0)
    {
      // Linear move
      wheel_cmd.speed = static_cast<int>((wheel_vel.right / motor_speed_max_)*100.0);
      pub_linear_speed_->publish(wheel_cmd);
    }
    else
    {
      // Angular move
      wheel_cmd.speed = static_cast<int>((wheel_vel.right / motor_speed_max_)*100.0);
      pub_turn_speed_->publish(wheel_cmd);
    }
  #endif

  }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WALLEKinematics>());
  rclcpp::shutdown();
  return 0;
}