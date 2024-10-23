/**
 * @file markers_get_robot_base_node.cpp
 */
#include <task_ui/markers_get_robot_base.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("markers_get_robot_base_node");

  std::string group;

  group = node->declare_parameter("group", "");
  if(group.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "group parameter was not read!");
    return 0;
  }
  RCLCPP_WARN(node->get_logger(), "group parameter: %s \n", group.c_str() );
  
  MarkersGetRobotBase mgrb(node);
  mgrb.init(group);
  
  rclcpp::spin(node);
  mgrb.stop();
  rclcpp::shutdown();
}

