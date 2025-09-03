/**
 * @file robot_task_markers_node.cpp
 */
#include <task_ui/robot_task_markers.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto task_markers = std::make_shared<RobotTaskMarkers>("/task_marker");
  
  if(!task_markers->init())
    return 0;
  
  rclcpp::spin(task_markers);
  task_markers->stop();
  rclcpp::shutdown();
}

