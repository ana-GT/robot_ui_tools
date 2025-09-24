/**
 * @file markers_get_grasp_object_poses_node.cpp
 */
#include <task_ui/markers_get_grasp_object_poses.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::string server_name = "robot_task";
  auto get_grasp_object_poses = std::make_shared<MarkersGetGraspObjectPoses>(server_name);
  
  get_grasp_object_poses->init();
  
  rclcpp::spin(get_grasp_object_poses);
  get_grasp_object_poses->stop();
  rclcpp::shutdown();
}

