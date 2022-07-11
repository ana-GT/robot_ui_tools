/**
 * @file jose_markers.cpp
 */

#include <jose/jose_markers.h>

using std::placeholders::_1;

/**
 * @function JoseMarkers
 * @brief Constructor
 */
JoseMarkers::JoseMarkers(rclcpp::Node::SharedPtr _nh) :
  nh_(_nh)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("/jose_markers",
									   nh_->get_node_base_interface(),
									   nh_->get_node_clock_interface(),
									   nh_->get_node_logging_interface(),
									   nh_->get_node_topics_interface(),
									   nh_->get_node_services_interface());

}

void JoseMarkers::stop()
{
  server_.reset();
}

void JoseMarkers::init(std::string _group)
{
  // Init jose kinematics
  jose_.init(nh_, _group);
  

  //ros::Duration(0.1).sleep();

  menu_handler_.insert( "Get IK", std::bind(&JoseMarkers::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&JoseMarkers::processFeedback, this, _1));
  menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&JoseMarkers::processFeedback, this, _1));

  tf2::Vector3 position;
  std::string frame_id = jose_.getBaseLink();
  
  position = tf2::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		  position, true, frame_id );

  js_topic_ = "/move_group/fake_controller_joint_states";
  js_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(js_topic_, 10);

  server_->applyChanges();

}

/**
 * @function makeBox
 */
visualization_msgs::msg::Marker JoseMarkers::makeBox( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.05; //msg.scale * 0.4;
  marker.scale.y = 0.05; //msg.scale * 0.4;
  marker.scale.z = 0.05; //msg.scale * 0.4;
  marker.color.r = 0.8;
  marker.color.g = 0.1;
  marker.color.b = 0.8;
  marker.color.a = 0.5;

  return marker;
}

/**
 * @functino makeBoxControl
 */
visualization_msgs::msg::InteractiveMarkerControl& JoseMarkers::makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( this->makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}


// %Tag(processFeedback)%
void JoseMarkers::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
      RCLCPP_INFO_STREAM( nh_->get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      // Get IK
      KDL::Twist bounds = KDL::Twist::Zero();
      Eigen::Vector3d pos;
      Eigen::Quaterniond rot;
      std::vector<double> joints;
      pos.x() = goal_pose_.pose.position.x;
      pos.y() = goal_pose_.pose.position.y;
      pos.z() = goal_pose_.pose.position.z;
      rot.x() = goal_pose_.pose.orientation.x;
      rot.y() = goal_pose_.pose.orientation.y;
      rot.z() = goal_pose_.pose.orientation.z;
      rot.w() = goal_pose_.pose.orientation.w;
      if(jose_.getIK(pos, rot, joints))
      {
	printf("Found a solution! \n");
	for(auto ji : joints)
	  printf("%f ", ji);
	printf("--** \n");

	// Publish
	sensor_msgs::msg::JointState msg;
	if(jose_.getMsg(joints, msg))
	  js_pub_->publish(msg);
      }
      else
	printf("Didn't found a solution! Aw \n");
    }
    break;

  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      goal_pose_.pose = feedback->pose;
      goal_pose_.header = feedback->header;
    }
    break;

  }

  server_->applyChanges();
}

/**
 * @function alignMarker
 */
void JoseMarkers::alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  RCLCPP_INFO_STREAM( nh_->get_logger(), feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server_->setPose( feedback->marker_name, pose );
  server_->applyChanges();
}

/**
 * @function make6DofMarkers
 */
void JoseMarkers::make6DofMarker( bool fixed, unsigned int interaction_mode,
				  const tf2::Vector3& position, bool show_6dof,
				  std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 0.4;

  int_marker.name = "goal";
  int_marker.description = "6dof goal";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&JoseMarkers::processFeedback, this, _1));
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler_.apply( *server_, int_marker.name );
}


/**
 * @function makeMenuMarker
 */
void JoseMarkers::makeMenuMarker( const tf2::Vector3& position,
				  std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&JoseMarkers::processFeedback,this,_1));
  menu_handler_.apply( *server_, int_marker.name );
}

/**
 * @function makeButtonMarker
 */
void JoseMarkers::makeButtonMarker( const tf2::Vector3& position,
				    std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&JoseMarkers::processFeedback, this, _1));
}

/**
 * @function makeMovingMarker
 */
void JoseMarkers::makeMovingMarker( const tf2::Vector3& position,
				    std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0, 0, 0, 1);
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&JoseMarkers::processFeedback, this, _1));
}

  
void JoseMarkers::saveMarker( visualization_msgs::msg::InteractiveMarker int_marker )
{
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&JoseMarkers::processFeedback, this, _1));
}
