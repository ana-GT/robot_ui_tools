#pragma once
#include <task_ui/robot_task_markers.h>

#include <robot_sim_msgs/srv/move_robot_to_task.hpp>


/**
 * @class MarkersGetRobotBase
 */
class MarkersGetRobotBase : public RobotTaskMarkers {

public:

  MarkersGetRobotBase(const std::string &_server_name);
  bool init_(const std::string &_group) override;
  
protected:

  void showSolution(robot_sim_msgs::msg::PlaceRobotSolution &_msg);  
  void client_cb(rclcpp::Client<robot_sim_msgs::srv::MoveRobotToTask>::SharedFuture _future);
  void processFeedback_( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) override;

  // Simulate motion
  rclcpp::Client<robot_sim_msgs::srv::MoveRobotToTask>::SharedPtr client_;
  std::shared_future<std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Response>> client_result_;

};


