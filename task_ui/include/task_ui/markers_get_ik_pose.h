#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/get_ik_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MarkersGetIKPose : public RobotTaskMarkers {

public:

  MarkersGetIKPose(const std::string &_server_name);
  bool init_(const std::string &_group) override;
  
protected:

  void processFeedback_( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;
  void client_cb(rclcpp::Client<reachability_msgs::srv::GetIKPose>::SharedFuture _future);
  void createArrowMarker(visualization_msgs::msg::Marker &_marker, const int &_id, const geometry_msgs::msg::PoseStamped &_pi);

  // Request and receive solution
  rclcpp::Client<reachability_msgs::srv::GetIKPose>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::GetIKPose::Response>> client_result_;


  // 
  std::string group_;

};


