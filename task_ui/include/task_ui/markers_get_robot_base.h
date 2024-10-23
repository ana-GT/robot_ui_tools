#pragma once
#include <task_ui/robot_task_markers.h>

#include <reachability_msgs/srv/move_robot_to_task.hpp>


/**
 * @class MarkersGetRobotBase
 */
class MarkersGetRobotBase : public RobotTaskMarkers {

public:

  MarkersGetRobotBase(rclcpp::Node::SharedPtr _nh);
  void init_(const std::string &_group) override;
  
protected:

  void showSolution(reachability_msgs::msg::PlaceRobotSolution &_msg);  
  void client_cb(rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedFuture _future);
  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;

  // Simulate motion
  rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedPtr client_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Response>> client_result_;

};


