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
/// @brief Joint state publishing for the WALL-E robot

#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <wall_e_interfaces/msg/servo_positions.hpp>

// Motor controller specific includes
#ifdef USE_ROBOCLAW
// TODO fix when roboclaw messages are defined
#include <std_msgs/msg/empty.hpp>
#endif

const std::vector<std::string> SERVO_NAMES = {
  "head_center",
  "neck_top",
  "neck_bottom",
  "eye_right",
  "eye_left",
  "arm_left",
  "arm_right",
};

const std::string WHEEL_LEFT_JOINT = "wheel_left_joint";
const std::string WHEEL_RIGHT_JOINT = "wheel_right_joint";

using namespace std::chrono_literals;

class WALLEJointStatePublisher : public rclcpp::Node
{
private:

  // TYPES --------------------------------------------------------------------------------

  /// @brief A class for converting between servo percentages and actual joint positions
  class ServoLimits
  {
  private:

    /// @brief lower limit of the servo joint, in radians. Corresponds to 0%
    const double low_ = 0.0;

    /// @brief upper limit of the servo joint, in radians. Corresponds
    const double high_ = 2*M_PI;

    const double range_ = high_ - low_;

  public:

    /// @brief Init the class by providing the low and high position limits of the servo
    /// @param low low position limit of the servo, in radians. Corresponds to 0%
    /// @param high high position limit of the servo, in radians. Corresponds to 100%
    ServoLimits(double low, double high)
    : low_{low}, high_{high}, range_{high_ - low_}
    {}

    /// @brief Convert from a servo percentage to
    /// the actual joint position in radians
    /// @param percentage servo position as a percentage along its limits,
    /// from 0.0 (0%) to 1.0 (100%)
    /// @return servo position in radians
    double percentage_to_rad(double percentage) const
    {
      return percentage*range_ + low_;
    }
  };

  // CLASS MEMBERS --------------------------------------------------------------------------------

  // ROS objects
  rclcpp::Subscription<wall_e_interfaces::msg::ServoPositions>::SharedPtr
    sub_servo_position_feedback_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  /// @brief map between servo name and servo limits class
  std::unordered_map<std::string, ServoLimits> servo_limits_;

  // Motor controller specific members
#ifdef USE_ROBOCLAW
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_wheel_left_ = nullptr; // TODO fix when roboclaw messages are defined
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_wheel_right_ = nullptr; // TODO fix when roboclaw messages are defined
#else
  rclcpp::TimerBase::SharedPtr tmr_wheel_dummy_ = nullptr;
#endif

public:


  // INITIALIZATION --------------------------------------------------------------------------------

  /// @brief iniitialize the node
  WALLEJointStatePublisher()
  : Node("wall_e_jsp")
  {

    // PARAMETERS ------------------------------------------------------------
    rcl_interfaces::msg::ParameterDescriptor desc;

    // Check if parameters are valid
    bool parameters_valid = true;

    for (const auto& servo : SERVO_NAMES)
    {
      const std::string param_prefix = "servo.limits." + servo;

      desc.description = "Lower limit of movement for " + servo +
                         " servo (radians), corresponds to 0%";
      declare_parameter(param_prefix + ".low", 0.0, desc);
      const auto low = get_parameter(param_prefix + ".low").get_parameter_value().get<double>();

      desc.description = "Higher limit of movement for " + servo +
                         " servo (radians), corresponds to 100%";
      declare_parameter(param_prefix + ".high", 0.0, desc);
      const auto high = get_parameter(param_prefix + ".high").get_parameter_value().get<double>();

      if (low == high)
      {
        RCLCPP_ERROR_STREAM(get_logger(),
          servo << " low (" << low << ") and high ("
          << high << ") limits are equal, please specify a valid range"
        );
        parameters_valid = false;
      }

      servo_limits_.emplace(servo, ServoLimits{low, high});
    }

    //Abort if any parameters were invalid
    if (!parameters_valid) {
      throw std::logic_error("One or more parameters were invalid");
    }


    // SUBSCRIBERS ------------------------------------------------------------
    sub_servo_position_feedback_ = create_subscription<wall_e_interfaces::msg::ServoPositions>(
      "servo_position_feedback",
      rclcpp::QoS{10},
      std::bind(
        &WALLEJointStatePublisher::sub_servo_position_feedback_callback,
        this,
        std::placeholders::_1
      )
    );

    // PUBLISHERS ------------------------------------------------------------
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::QoS{10}
    );

    // MOTOR CONTROLLER SPECIFIC INITS ------------------------------------------------------------
  #ifdef USE_ROBOCLAW
    sub_wheel_left_ = create_subscription<std_msgs::msg::Empty>( // TODO fix when roboclaw messages are defined
      "drive/left/feedback",
      rclcpp::QoS{10},
      // TODO fix when roboclaw messages are defined
      [this](const std_msgs::msg::Empty& msg)
      {
        sub_wheel_callback(WHEEL_LEFT_JOINT, msg);
      }
    );
    sub_wheel_right_ = create_subscription<std_msgs::msg::Empty>( // TODO fix when roboclaw messages are defined
      "drive/right/feedback",
      rclcpp::QoS{10},
      // TODO fix when roboclaw messages are defined
      [this](const std_msgs::msg::Empty& msg)
      {
        sub_wheel_callback(WHEEL_RIGHT_JOINT, msg);
      }
    );
  #else
    // When using the Arduino motor shield, no drive motor feedback is reported, so dummy
    // joint states are published instead
    tmr_wheel_dummy_ = create_wall_timer(
      0.1s,
      std::bind(&WALLEJointStatePublisher::tmr_wheel_dummy_callback, this)
    );
  #endif

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }

private:

  // SUBSCRIBERS --------------------------------------------------------------------------------

  /// @brief Receive servo position messages and convert them to actual units (i.e. radians)
  /// along the range of the servos, then publish joint state messages
  void sub_servo_position_feedback_callback(const wall_e_interfaces::msg::ServoPositions& msg)
  {
    sensor_msgs::msg::JointState joint_states;
    joint_states.header.stamp = msg.stamp;
    joint_states.name.reserve(msg.servos.size());
    joint_states.position.reserve(msg.servos.size());

    for (const auto& servo: msg.servos)
    {
      joint_states.name.push_back(servo.name + "_joint");

      // Convert joint positions as percentage to radians and store
      joint_states.position.push_back(
        servo_limits_.at(servo.name).percentage_to_rad(servo.position)
      );
    }

    // Publish servo joint states
    pub_joint_states_->publish(joint_states);
  }

#ifdef USE_ROBOCLAW
  /// @brief Receive a wheel feedback message and convert it to a joint state to send out
  /// @param joint_name the name of the joint
  /// @param msg msg to convert
  void sub_wheel_callback(const std::string& joint_name, const std_msgs::msg::Empty& msg)
  {
    // Convert feedback to joint state
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = get_clock()->now(); // TODO get timestamp from message when roboclaw messages are defined
    joint_state.name.push_back(joint_name);
    joint_state.position.push_back(0.0); // TODO get position from message when roboclaw messages are defined
    joint_state.velocity.push_back(0.0); // TODO get velocity from message when roboclaw messages are defined

    // Publish wheel joint state
    pub_joint_states_->publish(joint_state);
  }
#else
  void tmr_wheel_dummy_callback()
  {
    // Create dummy wheel joint states
    sensor_msgs::msg::JointState joint_states;
    joint_states.name.push_back(WHEEL_LEFT_JOINT);
    joint_states.position.push_back(0.0);
    joint_states.velocity.push_back(0.0);
    joint_states.name.push_back(WHEEL_RIGHT_JOINT);
    joint_states.position.push_back(0.0);
    joint_states.velocity.push_back(0.0);

    // Publish wheel joint states
    pub_joint_states_->publish(joint_states);
  }
#endif

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WALLEJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}