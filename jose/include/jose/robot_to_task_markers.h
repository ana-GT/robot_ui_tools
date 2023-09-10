#pragma once

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

#include <math.h>

/**
 * @class JoseMarkers
 */
class JoseMarkers
{
public:
  JoseMarkers(rclcpp::Node::SharedPtr _nh);
  void init(std::string _group);
  void stop();
  
  visualization_msgs::msg::Marker makeBox( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::Marker makeBottle( visualization_msgs::msg::InteractiveMarker &msg );
  visualization_msgs::msg::InteractiveMarkerControl& makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg );
 
  void processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void make6DofMarker( bool fixed, unsigned int interaction_mode,
		       const tf2::Vector3& position, bool show_6dof,
		       std::string frame_id); // base_link

  void makeMenuMarker( const tf2::Vector3& position,
		       std::string frame_id);
  void makeButtonMarker( const tf2::Vector3& position,
			 std::string frame_id);
  void makeMovingMarker( const tf2::Vector3& position,
			 std::string frame_id); // moving_frame
  
  double rand( double min, double max );
  void saveMarker( visualization_msgs::msg::InteractiveMarker int_marker );

  
protected:
  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  //Jose jose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedPtr client_;

};



