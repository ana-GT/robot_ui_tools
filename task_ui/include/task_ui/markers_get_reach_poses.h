#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/generate_reach_poses.hpp>

class MarkersGetReachPoses : public RobotTaskMarkers {

public:

  MarkersGetReachPoses(rclcpp::Node::SharedPtr _node);
  void init_(const std::string &_group) override;
  
protected:

  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;
  void client_cb(rclcpp::Client<reachability_msgs::srv::GenerateReachPoses>::SharedFuture _future);


  // Simulate motion
  rclcpp::Client<reachability_msgs::srv::GenerateReachPoses>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Response>> client_result_;


};


