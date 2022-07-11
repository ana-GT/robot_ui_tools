#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

/**********************************/
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("trac_ik_test");
  
  std::string base_link, tip_link;
  std::string urdf_param;

  // Test with Tiago
  base_link = "torso_lift_link";
  tip_link = "arm_tool_link";
  urdf_param = "robot_description";

  RCLCPP_INFO(node->get_logger(), "Create trac ik \n");
  TRAC_IK::TRAC_IK ti(node, base_link, tip_link, urdf_param);
  RCLCPP_INFO(node->get_logger(), "Spin! ");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
