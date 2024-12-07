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
    double percentage_to_position(double percentage) const
    {
      return percentage*range_ + low_;
    }
  };

  // CLASS MEMBERS --------------------------------------------------------------------------------

  // ROS objects
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  /// @brief map between servo name and servo limits class
  std::unordered_map<std::string, ServoLimits> servo_limits_;

  // Motor controller specific members
#ifdef USE_ROBOCLAW
  // TODO
#else
  // TODO
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
    // TODO

    // PUBLISHERS ------------------------------------------------------------
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>(
      "/wall_e/joint_states",
      rclcpp::QoS{10}
    );

    // MOTOR CONTROLLER SPECIFIC INITS ------------------------------------------------------------
  #ifdef USE_ROBOCLAW
    // TODO
  #else
    // TODO
  #endif

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }

private:

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WALLEJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}