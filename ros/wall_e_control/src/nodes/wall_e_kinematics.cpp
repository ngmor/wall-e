#include <rclcpp/rclcpp.hpp>

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