#pragma once

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <jose/jose.h>

#include <math.h>

/**
 * @class JoseMarkers
 */
class JoseMarkers
{
public:
  JoseMarkers(ros::NodeHandle _nh);
  void init(std::string _group);
  void stop();
  
  visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg );
  visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg );
  void frameCallback(const ros::TimerEvent&);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void make6DofMarker( bool fixed, unsigned int interaction_mode,
		       const tf::Vector3& position, bool show_6dof,
		       std::string frame_id); // base_link

  void makeMenuMarker( const tf::Vector3& position,
		       std::string frame_id);
  void makeButtonMarker( const tf::Vector3& position,
			 std::string frame_id);
  void makeMovingMarker( const tf::Vector3& position,
			 std::string frame_id); // moving_frame
  
  double rand( double min, double max );
  void saveMarker( visualization_msgs::InteractiveMarker int_marker );

  
protected:
  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Timer frame_timer_;
  tf::TransformBroadcaster br_;
  Jose jose_;
  geometry_msgs::PoseStamped goal_pose_;


  // Sim joint_states
  std::string js_topic_;
  ros::Publisher js_pub_;
};



