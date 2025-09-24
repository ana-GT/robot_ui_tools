#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/get_reach_data.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class MarkersGetReachData : public RobotTaskMarkers {

public:

  MarkersGetReachData(const std::string &_server_name);
  bool init_(const std::string &_group) override;
  
protected:

  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;
  void client_cb(rclcpp::Client<reachability_msgs::srv::GetReachData>::SharedFuture _future);
  void createArrowMarker(visualization_msgs::msg::Marker &_marker, const int &_id, const geometry_msgs::msg::PoseStamped &_pi);

  // Request and receive solution
  rclcpp::Client<reachability_msgs::srv::GetReachData>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::GetReachData::Response>> client_result_;


  // 
  std::string group_;

};


