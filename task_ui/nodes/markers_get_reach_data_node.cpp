/**
 * @file markers_get_reach_data_node.cpp
 */
#include <task_ui/markers_get_reach_data.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string server_name = "robot_task";
  auto get_reach_data = std::make_shared<MarkersGetReachData>(server_name);
  
  get_reach_data->init();
  
  rclcpp::spin(get_reach_data);
  get_reach_data->stop();
  rclcpp::shutdown();
}

