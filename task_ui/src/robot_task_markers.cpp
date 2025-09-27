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
      if(isGimbalShowing())
        hideGimbal();
      else
        showGimbal();  
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


RCLCPP_INFO(this->get_logger(), "Refs: %lu, objects: %lu, steps: %lu", references_.size(), objects_.size(), steps_.size());

  // Create step markers
  geometry_msgs::msg::Pose step_pose;
  for(auto si : steps_)
  {
    marker_names_.push_back(si.name);
   RCLCPP_INFO(this->get_logger(), "Processing step...");
    if (!doubleArrayToPose(params_.step_data.step_names_map.at(si.name).pose, step_pose))
      continue;  
    
    int index = getObjectIndex(si.object);   RCLCPP_INFO(this->get_logger(), "Getting object index: %d", index);
    if(index < 0)
      continue;
         RCLCPP_INFO(this->get_logger(), "Processing step, mesh: %s", objects_[index].mesh.c_str());
    make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		    step_pose, objects_[index].mesh, 
		    true, reference_frame_, si.name);    
  } // for si
  
}

/**
 * @function getObjectIndex
 */
int RobotTaskMarkers::getObjectIndex(const std::string &_name)
{   RCLCPP_INFO(this->get_logger(), "Getting object index for: %s", _name.c_str());
   int index = -1;
   for(int i = 0; i < objects_.size(); ++i)
   {   RCLCPP_INFO(this->get_logger(), "Object evaluated, type: %s", objects_[i].name.c_str());
      if(objects_[i].name == _name)
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

bool RobotTaskMarkers::isGimbalShowing()
{
  for(int i = 0; i < steps_.size(); ++i)
  {
    visualization_msgs::msg::InteractiveMarker im;
    std::string name_i = steps_[i].name;
    server_->get(name_i, im);
    
    if( im.controls.size() > 6 )
      return true;
  }
  
  return false;
}

void RobotTaskMarkers::hideGimbal()
{
  for(int i = 0; i < steps_.size(); ++i)
  {
    visualization_msgs::msg::InteractiveMarker im;
    std::string name_i = steps_[i].name;
    server_->get(name_i, im);
    int num = im.controls.size();
    for(int j = 1; j < num; ++j)
      im.controls.pop_back();
    
    server_->insert(im);
    server_->applyChanges();
  }  
}

void RobotTaskMarkers::showGimbal()
{
  for(int i = 0; i < steps_.size(); ++i)
  {
    visualization_msgs::msg::InteractiveMarker im;
    std::string name_i = steps_[i].name;
    server_->get(name_i, im);
    
    addGimbal(im);    
    server_->insert(im);
    server_->applyChanges();
  }  

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

