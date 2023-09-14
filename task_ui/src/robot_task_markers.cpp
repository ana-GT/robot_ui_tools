/**
 * @file task_ui_markers.cpp
 */

#include <task_ui/robot_task_markers.h>


using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @function RobotTaskMarkers
 * @brief Constructor
 */
RobotTaskMarkers::RobotTaskMarkers(rclcpp::Node::SharedPtr _nh) :
  nh_(_nh)
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("/task_marker",
									   nh_->get_node_base_interface(),
									   nh_->get_node_clock_interface(),
									   nh_->get_node_logging_interface(),
									   nh_->get_node_topics_interface(),
									   nh_->get_node_services_interface());

}

void RobotTaskMarkers::stop()
{
  server_.reset();
}

/**
 * @function init
 */
void RobotTaskMarkers::init(std::string _chain_group)
{
  client_ = nh_->create_client<reachability_msgs::srv::MoveRobotToTask>("robot_to_task");
  client_move_base_ = nh_->create_client<reachability_msgs::srv::SetRobotPose>("set_robot_pose");
  pub_js_ = nh_->create_publisher<sensor_msgs::msg::JointState>("joint_state_command", 10);

  //timer_ = nh_->create_wall_timer(std::chrono::milliseconds(200), std::bind(&RobotTaskMarkers::timer_cb, this));
  wait_for_result_ = false;

  // Load
  std::string param_prefix = "robot_task_ui_params." + _chain_group;

  std::shared_ptr<robot_task_ui_params::ParamListener> param_listener;
  param_listener = std::make_shared<robot_task_ui_params::ParamListener>(nh_, param_prefix);
  params_ = param_listener->get_params();


  menu_handler_.insert( "Get Robot pose", std::bind(&RobotTaskMarkers::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&RobotTaskMarkers::processFeedback, this, _1));
  menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&RobotTaskMarkers::processFeedback, this, _1));

  // Create poses
  createTaskMarkers();


  server_->applyChanges();

}

/**
 * @function getTaskMarkerName 
 */
std::string RobotTaskMarkers::getTaskMarkerName(int i)
{
  return std::string("task_marker_") + std::to_string(i);
}

void RobotTaskMarkers::createTaskMarkers()
{
  reference_frame_ = params_.reference_frame; 
  std::vector<geometry_msgs::msg::Pose> marker_poses(2);

  doubleArrayToPose(params_.object_start_pose, marker_poses[0]);  
  doubleArrayToPose(params_.object_goal_pose, marker_poses[1]);  
  int ind;

  for(int i = 0; i < params_.num_task_poses; ++i)
  {
    std::string name_i = getTaskMarkerName(i);
    marker_names_.push_back(name_i);
    make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		                marker_poses[marker_poses.size() <= 2? i : 1], true, reference_frame_ , name_i);    
  }
}

/**
 * @function doubleArrayToPose
 */
bool RobotTaskMarkers::doubleArrayToPose(const std::vector<double> &_arr, 
                                      geometry_msgs::msg::Pose &_pose)
{
  if(_arr.size() != 6)
  {
    _pose.position.x = 0; _pose.position.y = 0; _pose.position.z = 0;
    _pose.orientation.x = 0; _pose.orientation.y = 0; _pose.orientation.z = 0; _pose.orientation.w = 1.0;
    return false;
  }  

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(_arr[5], Eigen::Vector3d(0,0,1))*
      Eigen::AngleAxisd(_arr[4], Eigen::Vector3d(0,1,0))*
      Eigen::AngleAxisd(_arr[3], Eigen::Vector3d(1,0,0));
  q.normalize();
  
  _pose.position.x = _arr[0]; _pose.position.y = _arr[1]; _pose.position.z = _arr[2]; 
  _pose.orientation.x = q.x(); _pose.orientation.y = q.y(); 
  _pose.orientation.z = q.z(); _pose.orientation.w = q.w();

  return true;
}

/**
 * @function makeBox
 */
visualization_msgs::msg::Marker RobotTaskMarkers::makeBox( visualization_msgs::msg::InteractiveMarker &msg )
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
 * @function makeMeshMarker
 */
visualization_msgs::msg::Marker RobotTaskMarkers::makeMeshMarker(visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = params_.object_mesh;
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
visualization_msgs::msg::InteractiveMarkerControl& RobotTaskMarkers::makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( this->makeMeshMarker(msg) ); // makeBox
  msg.controls.push_back( control );

  return msg.controls.back();
}


// %Tag(processFeedback)%
void RobotTaskMarkers::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
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

       auto request = std::make_shared<reachability_msgs::srv::MoveRobotToTask::Request>();

      // Get feedback poses
      for(int i = 0; i < marker_names_.size(); ++i)
      {
        visualization_msgs::msg::InteractiveMarker im;
        if(!server_->get(marker_names_[i], im))
          return;
        geometry_msgs::msg::PoseStamped pi;
        pi.pose = im.pose;
        pi.header.frame_id = reference_frame_;
  
        request->tcp_poses.push_back(pi);
      }

       request->num_robot_placements = params_.num_robot_placements;
      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(nh_->get_logger(), "service not available, waiting again...");
      }
    
      RCLCPP_INFO(nh_->get_logger(), "Sending request for manipulation task plan");
      auto result = client_->async_send_request(request, 
                       std::bind(&RobotTaskMarkers::client_cb, this, std::placeholders::_1));
     // Do not wait for result or crash. No nested wait spinning
    }
    break;

  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      //updatePose(goal_pose_, feedback->pose, feedback->header);
    }
    break;

  }

  server_->applyChanges();
}

void RobotTaskMarkers::client_cb(rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedFuture _future)
{
  auto status = _future.wait_for(1s);
  if (status == std::future_status::ready)
  {
      RCLCPP_INFO(nh_->get_logger(), "Status is ready?");
      auto response = _future.get();
      RCLCPP_INFO(nh_->get_logger(), "Got response with %lu solutions", response->solutions.size());

    RCLCPP_INFO(nh_->get_logger(), "Showing %ld solutions", response->solutions.size());
    for(auto si : response->solutions)
    {
      showSolution(si);
      usleep(1.0*1e6);
    }

  }
      
      
}

/**
 * @function updatePose
 */
void RobotTaskMarkers::updatePose(geometry_msgs::msg::PoseStamped &_pose, 
                  const geometry_msgs::msg::Pose &_pos, 
                  const std_msgs::msg::Header &_hed)
{
  _pose.pose = _pos;
  _pose.header = _hed;
}

void RobotTaskMarkers::updatePose(geometry_msgs::msg::PoseStamped &_pose, 
                  const geometry_msgs::msg::Pose &_pos, 
                  const std::string &_frame_id)
{
  _pose.pose = _pos;
  _pose.header.frame_id = _frame_id;
}

/**
 * @function alignMarker
 */
void RobotTaskMarkers::alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
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
void RobotTaskMarkers::make6DofMarker( bool fixed, unsigned int interaction_mode,
				                            const geometry_msgs::msg::Pose &_pose, 
                                    bool show_6dof,
				                            std::string _frame_id,
                                    std::string _marker_name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = _frame_id;
  int_marker.pose = _pose;  
  int_marker.scale = 0.4;

  int_marker.name = _marker_name;
  int_marker.description = _marker_name;

  // insert a marker
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;
/*
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
  }*/

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
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback, this, _1));
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler_.apply( *server_, int_marker.name );
}


/**
 * @function makeMenuMarker
 */
void RobotTaskMarkers::makeMenuMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback,this,_1));
  menu_handler_.apply( *server_, int_marker.name );
}

/**
 * @function makeButtonMarker
 */
void RobotTaskMarkers::makeButtonMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback, this, _1));
}

/**
 * @function makeMovingMarker
 */
void RobotTaskMarkers::makeMovingMarker( const tf2::Vector3& position,
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
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback, this, _1));
}

  
void RobotTaskMarkers::saveMarker( visualization_msgs::msg::InteractiveMarker int_marker )
{
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback, this, _1));
}

/**
 * @function moveBase 
 */
void RobotTaskMarkers::moveBase(const geometry_msgs::msg::PoseStamped &_pose)
{
  auto request = std::make_shared<reachability_msgs::srv::SetRobotPose::Request>();
  request->pose = _pose;

  while (!client_move_base_->wait_for_service(1s)) {
      
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...");
  }

  auto result = client_move_base_->async_send_request(request);
}

/**
 * @function showSolution
 */
void RobotTaskMarkers::showSolution(reachability_msgs::msg::PlaceRobotSolution &_msg)
{
    // Move base
    moveBase(_msg.base_pose);
    // Update arm pose
    for(auto cs : _msg.chain_sols)
    {
      pub_js_->publish(cs);
      usleep(1.0*1e6);
    }
}

/**
 * @function timer_cb
 */
void RobotTaskMarkers::timer_cb()
{
   RCLCPP_INFO(nh_->get_logger(), "Timer check...");
  if(!wait_for_result_)
    return;

  if(client_result_.valid())
  {
    wait_for_result_ = false;  
 
    auto result = client_result_.get();
    RCLCPP_INFO(nh_->get_logger(), "Timer got valid result. Showing %ld solutions", result->solutions.size());
    for(auto si : result->solutions)
    {
      showSolution(si);
      usleep(1.0*1e6);
    }

  }

}