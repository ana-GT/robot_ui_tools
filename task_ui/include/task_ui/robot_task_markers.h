#pragma once

/**
 * @file robot_to_task_markers.h
 */
#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>

#include <reachability_msgs/srv/move_robot_to_task.hpp>
#include <reachability_msgs/srv/set_robot_pose.hpp>


#include <math.h>

// Parameters
#include "robot_task_ui_params.hpp"

/**
 * @class RobotTaskMarkers
 */
class RobotTaskMarkers
{
public:
  RobotTaskMarkers(rclcpp::Node::SharedPtr _nh);
  void init(std::string _group);
  void stop();
  
protected:
  visualization_msgs::msg::Marker makeBox( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::Marker makeMeshMarker( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::InteractiveMarkerControl& makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg );
 
  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void make6DofMarker( bool fixed, unsigned int interaction_mode,
		       const geometry_msgs::msg::Pose& _pose, 
           bool show_6dof,
		       std::string frame_id); // base_link

  void makeMenuMarker( const tf2::Vector3& position,
		       std::string frame_id);
  void makeButtonMarker( const tf2::Vector3& position,
			 std::string frame_id);
  void makeMovingMarker( const tf2::Vector3& position,
			 std::string frame_id); // moving_frame
  
  double rand( double min, double max );
  void saveMarker( visualization_msgs::msg::InteractiveMarker int_marker );

  bool doubleArrayToPose(const std::vector<double> &_arr, 
                         geometry_msgs::msg::Pose &_pose);

  void updatePose(geometry_msgs::msg::PoseStamped &_pose, 
                const geometry_msgs::msg::Pose &_pos, 
                const std_msgs::msg::Header &_hed);  

  void updatePose(geometry_msgs::msg::PoseStamped &_pose, 
                  const geometry_msgs::msg::Pose &_pos, 
                  const std::string &_frame_id);                                       

  void timer_cb();

  // To simulate motion
  void showSolution(reachability_msgs::msg::PlaceRobotSolution &_msg);  
  void moveBase(const geometry_msgs::msg::PoseStamped &_pose);  

  // Parameters
  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  
  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedPtr client_;

  // Params
  robot_task_ui_params::Params params_;

  // Simulate motion
  rclcpp::Client<reachability_msgs::srv::SetRobotPose>::SharedPtr client_move_base_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  std::shared_future<std::shared_ptr<reachability_msgs::srv::MoveRobotToTask::Response>> client_result_;

  // Update
  rclcpp::TimerBase::SharedPtr timer_;
  bool wait_for_result_;
};



