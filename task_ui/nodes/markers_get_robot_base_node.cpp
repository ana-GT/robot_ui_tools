/**
 * @file markers_get_robot_base_node.cpp
 */
#include <task_ui/markers_get_robot_base.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string server_name = "robot_task";
  auto get_robot_base = std::make_shared<MarkersGetRobotBase>(server_name);

  get_robot_base->init();
  
  rclcpp::spin(get_robot_base);
  get_robot_base->stop();
  rclcpp::shutdown();
}

