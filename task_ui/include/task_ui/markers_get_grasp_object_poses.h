#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/get_grasp_object_poses.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MarkersGetGraspObjectPoses : public RobotTaskMarkers {

public:

  MarkersGetGraspObjectPoses(const std::string &_server_name);
  bool init_(const std::string &_group) override;
  
protected:

  void processFeedback_( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;
  void client_cb(rclcpp::Client<reachability_msgs::srv::GetGraspObjectPoses>::SharedFuture _future);
  void createArrowMarker(visualization_msgs::msg::Marker &_marker, const int &_id, const geometry_msgs::msg::PoseStamped &_pi);

  // Request and receive solution
  rclcpp::Client<reachability_msgs::srv::GetGraspObjectPoses>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::GetGraspObjectPoses::Response>> client_result_;


  // 
  std::string group_;

};


