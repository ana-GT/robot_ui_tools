/**
 * @file task_ui_markers.cpp
 */
#include <task_ui/task_ui_markers.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @function TaskUiMarkers
 * @brief Constructor
 */
TaskUiMarkers::TaskUiMarkers(rclcpp::Node::SharedPtr _nh) :
  nh_(_nh)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("/human_marker",
									   nh_->get_node_base_interface(),
									   nh_->get_node_clock_interface(),
									   nh_->get_node_logging_interface(),
									   nh_->get_node_topics_interface(),
									   nh_->get_node_services_interface());
}

/**
 * @function stop
 */
void TaskUiMarkers::stop()
{
  server_.reset();
}

/**
 * @function init
 */
void TaskUiMarkers::init(std::string _group)
{
  client_ = nh_->create_client<reachability_msgs::srv::GetHandToUser>("hand_to_user");

  //ros::Duration(0.1).sleep();

  menu_handler_.insert( "Get Hand Pose", std::bind(&TaskUiMarkers::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&TaskUiMarkers::processFeedback, this, _1));
  menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&TaskUiMarkers::processFeedback, this, _1));

  tf2::Vector3 position;
  std::string frame_id = "world"; //jose_.getBaseLink();
  
  position = tf2::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		  position, true, frame_id );

  //js_topic_ = "/move_group/fake_controller_joint_states";
  //js_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(js_topic_, 10);

  server_->applyChanges();

}

/**
 * @function makeBox
 */
visualization_msgs::msg::Marker TaskUiMarkers::makeBox( visualization_msgs::msg::InteractiveMarker &msg )
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
 * @function makeHuman
 */
visualization_msgs::msg::Marker TaskUiMarkers::makeHuman( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::Marker marker;
  RCLCPP_WARN(rclcpp::get_logger("task_ui_markers"), "Make Human...");
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://task_ui/meshes/person_standing_reference.dae";
  marker.mesh_use_embedded_materials = true;
  marker.scale.x =1.0; //msg.scale * 0.4;
  marker.scale.y = 1.0; //msg.scale * 0.4;
  marker.scale.z = 1.0; //msg.scale * 0.4;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0;

  return marker;
}

/**
 * @function makeBoxControl
 */
visualization_msgs::msg::InteractiveMarkerControl& TaskUiMarkers::makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( this->makeHuman(msg) ); // makeBox
  msg.controls.push_back( control );

  return msg.controls.back();
}

/**
 * @function processFeedback
 */
void TaskUiMarkers::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  std::ostringstream s;
  s << "* Feedback from marker '" << feedback->marker_name;

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

      RCLCPP_INFO_STREAM( nh_->get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked");

      
       auto request = std::make_shared<reachability_msgs::srv::GetHandToUser::Request>();
       request->pose = goal_pose_;

      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client_->async_send_request(request);
     // Do not wait for result or crash. No nested wait spinning
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
void TaskUiMarkers::alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
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
void TaskUiMarkers::make6DofMarker( bool fixed, unsigned int interaction_mode,
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
  server_->setCallback(int_marker.name, std::bind(&TaskUiMarkers::processFeedback, this, _1));
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler_.apply( *server_, int_marker.name );
}


/**
 * @function makeMenuMarker
 */
void TaskUiMarkers::makeMenuMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&TaskUiMarkers::processFeedback,this,_1));
  menu_handler_.apply( *server_, int_marker.name );
}

/**
 * @function makeButtonMarker
 */
void TaskUiMarkers::makeButtonMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&TaskUiMarkers::processFeedback, this, _1));
}

/**
 * @function makeMovingMarker
 */
void TaskUiMarkers::makeMovingMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&TaskUiMarkers::processFeedback, this, _1));
}

  
void TaskUiMarkers::saveMarker( visualization_msgs::msg::InteractiveMarker int_marker )
{
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&TaskUiMarkers::processFeedback, this, _1));
}
