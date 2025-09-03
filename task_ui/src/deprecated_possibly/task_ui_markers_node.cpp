#include <task_ui/task_ui_markers.h>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("task_ui_markers_node");

  std::string group;

  group = node->declare_parameter("group", "");
  if(group.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "group parameter was not read!");
    return 0;
  }
  RCLCPP_WARN(node->get_logger(), "group parameter: %s \n", group.c_str() );
  
  TaskUiMarkers tum(node);
  tum.init(group);
  
  rclcpp::spin(node);
  tum.stop();
  rclcpp::shutdown();
}

