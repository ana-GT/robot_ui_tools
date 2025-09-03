/**
 * @file markers_get_ik_pose_node.cpp
 */
#include <task_ui/markers_get_ik_pose.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string server_name = "robot_task";
  auto get_ik_pose = std::make_shared<MarkersGetIKPose>(server_name);
  
  get_ik_pose->init();
  
  rclcpp::spin(get_ik_pose);
  get_ik_pose->stop();
  rclcpp::shutdown();
}

