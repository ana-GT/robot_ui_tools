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

#include <Eigen/Geometry>

#include <reachability_msgs/srv/set_robot_pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
  void init(const std::string &_group);
  virtual void init_(const std::string &_group) = 0;
  void stop();
  
protected:
  visualization_msgs::msg::Marker makeBox( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::Marker makeMeshMarker( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::InteractiveMarkerControl& makeMeshControl( visualization_msgs::msg::InteractiveMarker &msg );
 
  virtual void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) = 0;
  void switchGimbal( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void make6DofMarker( bool fixed, unsigned int interaction_mode,
		       const geometry_msgs::msg::Pose& _pose, 
                      bool show_6dof,
		       std::string frame_id,
                      std::string _marker_name); // base_link

  void makeMenuMarker( const tf2::Vector3& position,
		       std::string frame_id);
  void makeButtonMarker( const tf2::Vector3& position,
			 std::string frame_id);
  void makeMovingMarker( const tf2::Vector3& position,
			 std::string frame_id); // moving_frame
  
  bool isGimbalShowing();
  void showGimbal();
  void hideGimbal();
  void addGimbal( visualization_msgs::msg::InteractiveMarker &_im );
  
  double rand( double min, double max );
  void saveMarker( visualization_msgs::msg::InteractiveMarker int_marker );

  bool doubleArrayToPose(const std::vector<double> &_arr, 
                         geometry_msgs::msg::Pose &_pose);
                                      

  // To simulate motion
  void moveBase(const geometry_msgs::msg::PoseStamped &_pose);  

  std::string getTaskMarkerName(int i);
  void createTaskMarkers();

  // Parameters
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  
  // Params
  robot_task_ui_params::Params params_;
  std::vector<std::string> marker_names_;
  std::string reference_frame_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Client<reachability_msgs::srv::SetRobotPose>::SharedPtr client_move_base_;
};

