/**
 * @file markers_get_mobile_poses_node.cpp
 */
#include <task_ui/markers_get_mobile_poses.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string server_name = "robot_task";
  auto get_mobile_poses = std::make_shared<MarkersGetMobilePoses>(server_name);
  
  get_mobile_poses->init();
  
  rclcpp::spin(get_mobile_poses);
  get_mobile_poses->stop();
  rclcpp::shutdown();
}

