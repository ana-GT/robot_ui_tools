#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/generate_reach_poses.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MarkersGetReachPoses : public RobotTaskMarkers {

public:

  MarkersGetReachPoses(rclcpp::Node::SharedPtr _node);
  void init_(const std::string &_group) override;
  
protected:

  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;
  void client_cb(rclcpp::Client<reachability_msgs::srv::GenerateReachPoses>::SharedFuture _future);
  void createArrowMarker(visualization_msgs::msg::Marker &_marker, const int &_id, const geometry_msgs::msg::PoseStamped &_pi);

  // Simulate motion
  rclcpp::Client<reachability_msgs::srv::GenerateReachPoses>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Response>> client_result_;

  // Added to show EE poses
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_;

};


