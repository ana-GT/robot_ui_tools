/**
 * @file task_ui_markers.cpp
 */

#include <task_ui/robot_task_markers.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp> 

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @function RobotTaskMarkers
 * @brief Constructor
 */
RobotTaskMarkers::RobotTaskMarkers(const std::string &_server_name) :
  Node("robot_task_markers")
{
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(_server_name,
									   this->get_node_base_interface(),
									   this->get_node_clock_interface(),
									   this->get_node_logging_interface(),
									   this->get_node_topics_interface(),
									   this->get_node_services_interface());
  this->declare_parameter("group", "");
}

void RobotTaskMarkers::stop()
{
  server_.reset();
}

bool RobotTaskMarkers::init()
{
  std::string chain_group;
 
  if( !this->get_parameter("group", chain_group) )
  {
    RCLCPP_ERROR(this->get_logger(), "group parameter was not set!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "group parameter: %s ", chain_group.c_str() );


  // To simulate arm motion and robot base moving
  pub_js_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state_command", 10);
  client_move_base_ = this->create_client<robot_sim_msgs::srv::SetRobotPose>("set_robot_pose");

  // Load
  std::string param_prefix = "robot_task_ui_params." + chain_group;

  std::shared_ptr<robot_task_ui_params::ParamListener> param_listener;
  param_listener = std::make_shared<robot_task_ui_params::ParamListener>(this->shared_from_this(), param_prefix);
  params_ = param_listener->get_params();

  
  // Call derived class specific
  if(!init_(chain_group))
    return false;

  // Add menu handler to hide/show the gimbal
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "View" );
  menu_handler_.insert( sub_menu_handle, "Hide/Show 6D gimbal", std::bind(&RobotTaskMarkers::switchGimbal, this, _1));
 
  // Create markers that represent the task
  // has to come AFTER init_ to attach menus accordingly
  createTaskMarkers();
  
  // Update
  server_->applyChanges();
   
  return true;
}

void RobotTaskMarkers::switchGimbal( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{

  switch ( feedback->event_type )
  {
     case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      if(isGimbalShowing(feedback->marker_name))
        hideGimbal(feedback->marker_name);
      else
        showGimbal(feedback->marker_name);  
     break;
  }
}

/**
 * @function getTaskMarkerName 
 */
std::string RobotTaskMarkers::getTaskMarkerName(int i)
{
  return std::string("task_marker_") + std::to_string(i);
}

/**
 * @brief createTaskMarkers
 */
void RobotTaskMarkers::createTaskMarkers()
{
  parseParams();
  
  // Create reference markers
  geometry_msgs::msg::Pose reference_pose;
  for(auto ri : references_)
  {
    if (!doubleArrayToPose(params_.reference_data.reference_names_map.at(ri.name).pose, reference_pose))
      continue;
      
    make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		    reference_pose, ri.mesh,
		    true, reference_frame_, ri.name);
  } // for ri

  // Create step markers
  geometry_msgs::msg::Pose step_pose;
  for(auto si : steps_)
  {
    marker_names_.push_back(si.name);
    
    int index = getObjectIndex(si.object);
    if(index < 0)
      continue;

    auto step_pose = calculateStepPose(si);
   
    make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		    step_pose, objects_[index].mesh, 
		    true, reference_frame_, si.name);    
  } // for si
  
}

/**
 * @function calculateStepPose
 */
geometry_msgs::msg::Pose RobotTaskMarkers::calculateStepPose(const Step &_step)
{
 Eigen::Isometry3d Tf_ref, Tf_step, Tf_full;
 
 // Get reference pose
 auto ref_pose = getMarkerPose(_step.reference);
 tf2::fromMsg( getMarkerPose(_step.reference), Tf_ref );
 
 // Get relative pose
 tf2::fromMsg(_step.pose, Tf_step);
  
 // multiply and return
 Tf_full = Tf_ref * Tf_step;
 
 geometry_msgs::msg::Pose pose = tf2::toMsg(Tf_full); 
 return pose;
}

/**
 * @function getMarkerPose
 */
geometry_msgs::msg::Pose RobotTaskMarkers::getMarkerPose(const std::string &_name)
{
  visualization_msgs::msg::InteractiveMarker im;  
  if(!server_->get(_name, im))
    RCLCPP_ERROR(this->get_logger(), "Marker with name %s does NOT exist", _name.c_str());
  
  return  im.pose;
}

/**
 * @function getObjectIndex
 */
int RobotTaskMarkers::getObjectIndex(const std::string &_name)
{
   int index = -1;
   for(int i = 0; i < objects_.size(); ++i)
   {
      if(objects_[i].name == _name)
        return i;
   }
   
   return index;
}

/**
 * @function getReferenceIndex
 */
int RobotTaskMarkers::getReferenceIndex(const std::string &_name)
{
   int index = -1;
   for(int i = 0; i < references_.size(); ++i)
   {
      if(references_[i].name == _name)
        return i;
   }
   
   return index;
}

/**
 * @function getStepIndex
 */
int RobotTaskMarkers::getStepIndex(const std::string &_name)
{
   int index = -1;
   for(int i = 0; i < steps_.size(); ++i)
   {
      if(steps_[i].name == _name)
        return i;
   }
   
   return index;
}


/**
 * @function parseParams
 */
void RobotTaskMarkers::parseParams() 
{
   // Parse reference frame
   reference_frame_ = params_.reference_frame; 

   // Parse reference names
   std::vector<std::string> reference_names = params_.reference_names;
      
   // Parse reference data  
   for(auto ri : reference_names)
   {
    Reference ref;
    ref.name = ri;
    ref.mesh = params_.reference_data.reference_names_map.at(ri).mesh;
    
    references_.push_back(ref);
   }
      
   // Parse object names
   std::vector<std::string> object_names = params_.object_names;

   // Parse object data
   for(auto oi : object_names)
   {
    Object obj;
    obj.name = oi;
    obj.mesh = params_.object_data.object_names_map.at(oi).mesh;

    std::vector<double> grasp_offset = params_.object_data.object_names_map.at(oi).grasp_offset;
    if (!doubleArrayToPose(params_.object_data.object_names_map.at(oi).grasp_offset, obj.grasp_offset))
      continue;

    objects_.push_back(obj);
   } // for oi
   
   // Parse step names
   std::vector<std::string> step_names = params_.step_names;

   // Parse step data
   for(auto si : step_names)
   {
    Step step;
    step.name = si;
    step.object = params_.step_data.step_names_map.at(si).object;
    step.reference = params_.step_data.step_names_map.at(si).reference;
    
    if (!doubleArrayToPose(params_.step_data.step_names_map.at(si).pose, step.pose))
      continue;  
          
    steps_.push_back(step);
   } // for si
   
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
visualization_msgs::msg::Marker RobotTaskMarkers::makeMeshMarker(visualization_msgs::msg::InteractiveMarker &msg,
                                                                 const std::string &_mesh )
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker.mesh_resource = _mesh;
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
 * @function makeMeshControl
 */
visualization_msgs::msg::InteractiveMarkerControl& RobotTaskMarkers::makeMeshControl( visualization_msgs::msg::InteractiveMarker &msg,
                                   const std::string &_mesh )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( this->makeMeshMarker(msg, _mesh) ); // makeBox
  msg.controls.push_back( control );

  return msg.controls.back();
}



/**
 * @function alignMarker
 */
void RobotTaskMarkers::alignMarker( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  geometry_msgs::msg::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  RCLCPP_INFO_STREAM( this->get_logger(), feedback->marker_name << ":"
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
				       const std::string &_mesh,
                                       bool show_6dof,
				       std::string _frame_id,
                                       std::string _marker_name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = _frame_id;
  int_marker.pose = _pose;  
  int_marker.scale = params_.gimbal_scale;

  int_marker.name = _marker_name;
  int_marker.description = _marker_name;

  // insert a marker
  makeMeshControl(int_marker, _mesh);
  int_marker.controls[0].interaction_mode = interaction_mode;

  if(show_6dof)
    addGimbal(int_marker);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&RobotTaskMarkers::processFeedback, this, _1));

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler_.apply( *server_, int_marker.name );
}

bool RobotTaskMarkers::isGimbalShowing(const std::string &_name)
{

  for(int i = 0; i < steps_.size(); ++i)
  {
    visualization_msgs::msg::InteractiveMarker im;
    server_->get(_name, im);
    
    if( im.controls.size() > 6 )
      return true;
  }
  
  return false;
}

void RobotTaskMarkers::hideGimbal(const std::string &_name)
{
    visualization_msgs::msg::InteractiveMarker im;
    server_->get(_name, im);
    int num = im.controls.size();
    for(int j = 1; j < num; ++j)
      im.controls.pop_back();
    
    server_->insert(im);
    server_->applyChanges();
}

void RobotTaskMarkers::showGimbal(const std::string &_name)
{
    visualization_msgs::msg::InteractiveMarker im;
    server_->get(_name, im);
    
    addGimbal(im);    
    server_->insert(im);
    server_->applyChanges();

}

void RobotTaskMarkers::addGimbal( visualization_msgs::msg::InteractiveMarker &_im )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    _im.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    _im.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    _im.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    _im.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    _im.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    _im.controls.push_back(control);
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


/**
 * @function processFeedback
 */
void RobotTaskMarkers::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{

  switch ( feedback->event_type )
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      // If it is reference object, update pose of step related to it
      auto marker = feedback->marker_name;
      geometry_msgs::msg::Pose pose_step;
       
      Eigen::Isometry3d Tf_marker, Tf_rel, Tf_step; 
      tf2::fromMsg( getMarkerPose(marker), Tf_marker );      

      int ref_index = getReferenceIndex(marker);
      if(ref_index >= 0)
      {
	for(int i = 0; i < steps_.size(); ++i)
	{
	 if(steps_[i].reference == marker)
	 {
	   tf2::fromMsg(steps_[i].pose, Tf_rel);
	   Tf_step = Tf_marker * Tf_rel;
	   pose_step = tf2::toMsg(Tf_step);
	   // update pose
	   server_->setPose(steps_[i].name, pose_step);
	 } // if steps
	} // for i
      
        break;
      }
      // If it is step object, update stored pose
      int step_index = getStepIndex(marker);
      
      geometry_msgs::msg::Pose pose_ref;
      Eigen::Isometry3d Tf_ref;
      
      if(step_index >= 0)
      {
        // Get pose of step
        pose_step = getMarkerPose(steps_[step_index].name);
        tf2::fromMsg(pose_step, Tf_step);
        
        // Get pose of reference
        pose_ref = getMarkerPose(steps_[step_index].reference);
        tf2::fromMsg(pose_ref, Tf_ref);
                
        // Get Tf_step = Tf_ref * Tf_rel = Tf_ref.inverse() * Tf_step
        Tf_rel = Tf_ref.inverse() * Tf_step;
        steps_[step_index].pose = tf2::toMsg(Tf_rel);
	   	
        break;
      }      

    }
    break;
  }

 // Specific application
 processFeedback_(feedback);
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
  auto request = std::make_shared<robot_sim_msgs::srv::SetRobotPose::Request>();
  request->pose = _pose;

  while (!client_move_base_->wait_for_service(1s)) {
      
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto result = client_move_base_->async_send_request(request);
}

