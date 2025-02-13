/**
 * @file markers_get_robot_base_node.cpp
 */
#include <task_ui/markers_get_reach_poses.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("markers_get_reach_poses_node");

  std::string server_name = "robot_task";
  auto get_reach_poses = std::make_shared<MarkersGetReachPoses>(server_name);
  
  get_reach_poses->init();
  
  rclcpp::spin(get_reach_poses);
  get_reach_poses->stop();
  rclcpp::shutdown();
}

