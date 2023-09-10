/**
 * @file robot_to_task_markers_node.cpp
 */
#include <task_ui/robot_to_task_markers.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_to_task_markers_node");

  std::string group;

  group = node->declare_parameter("group", "");
  if(group.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "group parameter was not read!");
    return 0;
  }
  RCLCPP_WARN(node->get_logger(), "group parameter: %s \n", group.c_str() );
  
  JoseMarkers jm(node);
  jm.init(group);
  
  rclcpp::spin(node);
  jm.stop();
  rclcpp::shutdown();
}

