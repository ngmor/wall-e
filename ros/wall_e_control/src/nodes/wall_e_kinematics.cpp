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
/// @brief TODO

#include <rclcpp/rclcpp.hpp>

#include <wall_e_control/kinematics/diff_drive.hpp>

// Motor controller specific includes
#ifdef USE_ROBOCLAW
// TODO fix when roboclaw messages are defined
#include <std_msgs/msg/empty.hpp>
#else
#include <wall_e_interfaces/msg/drive_motor_speed.hpp>
#endif

class WALLEKinematics : public rclcpp::Node
{
private:


  // CLASS MEMBERS --------------------------------------------------------------------------------

public:


  // INITIALIZATION --------------------------------------------------------------------------------

  /// @brief iniitialize the node
  WALLEKinematics()
  : Node("wall_e_kinematics")
  {


    // MOTOR CONTROLLER SPECIFIC INITS ------------------------------------------------------------
  #ifdef USE_ROBOCLAW
    RCLCPP_INFO_STREAM(get_logger(),
      get_fully_qualified_name() << " node configured to use RoboClaw motor controller"
    );
  #else
    RCLCPP_INFO_STREAM(get_logger(),
      get_fully_qualified_name() << " node configured to use Arduino motor shield"
    );
  #endif

    RCLCPP_INFO_STREAM(get_logger(), get_fully_qualified_name() << " node started");
  }

private:

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WALLEKinematics>());
  rclcpp::shutdown();
  return 0;
}