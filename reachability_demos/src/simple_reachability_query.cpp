/**
 * @file simple_placement_solver.cpp
 */
#include <reachability_demos/simple_reachability_query.h>

#include <algorithm>
#include <cfloat>
#include <nlopt.h>

#include <reachability_description/reach_utilities.h>

using namespace std::chrono_literals;

const auto logger = rclcpp::get_logger("simple_reachability_query");

/**
 * Constructor
 */
RobotToTask::RobotToTask() :
rclcpp::Node("robot_to_task")
{
    this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
}

/*
 * Initialize
 */
bool RobotToTask::initialize()
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

     this->get_parameter("chain_group_name", chain_group_);
     
     if(!this->get_parameter("robot_name", robot_name_))
       return false;
       
    // Init Reachability
    rd_.reset( new reachability_description::ReachabilityDescription(this->shared_from_this()));
    
    if (!rd_->initialize(robot_name_))
      return false;
      
    if (!rd_->initializeGroup(chain_group_))
      return false;

    return true;
}

/**
 * Offer service to get pose
 */
bool RobotToTask::setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<reachability_msgs::srv::GenerateReachPoses>("generate_reach_poses", 
                std::bind(&RobotToTask::handleSrv, this, _1, _2));

    return true;
}


/**
 * Handle service
 */
void RobotToTask::handleSrv(const std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Request> req,
               std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Response> res)
{
  RCLCPP_INFO(logger, "Received service to query reachability for this pose");

  // Get X,Y,Z
  double x, y, z;
  x = req->bbox.center.position.x;
  y = req->bbox.center.position.y;
  z = req->bbox.center.position.z;
  
  // Fill the voxel for this location
  reachability_msgs::msg::ReachData reach_data;
  reach_data = rd_->calculateReachabilityPoint(x, y, z, chain_group_);
    
  // Return the EE poses / joint states
  reachability_msgs::msg::ChainInfo ci;
  rd_->getChainInfo(chain_group_, ci);
  
  for(auto si : reach_data.samples)
  {
    geometry_msgs::msg::PoseStamped ps;
    sensor_msgs::msg::JointState js;
    
    ps.pose = si.pose;
    js = vectorToJointState(si.best_config, ci);
    
    res->ee_poses.push_back(ps);
    res->joint_states.push_back(js);
  }

  res->success = res->ee_poses.empty()? false : true; 
}


/**
 * @function getTransform // (-0.062, 0.0, 0.291);
 */
bool RobotToTask::getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx)
{
   geometry_msgs::msg::TransformStamped tfxs;
   try
   {
      tfxs = tf_buffer_->lookupTransform(_source, _target, rclcpp::Time(0), rclcpp::Duration(1, 0));
   }
   catch (tf2::TransformException& ex)
   {
      RCLCPP_ERROR_STREAM(logger, "No transform from " << _source << " to " << _target
                                                       << ".  Error: " << ex.what());
      return false;
   }
   
   _Tfx = tf2::transformToEigen(tfxs);
   return true;
}


////////////////////////////////////

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   std::shared_ptr<RobotToTask> rtt = std::make_shared<RobotToTask>();

  if(!rtt->initialize())
    return 1;

  // Offer service
  rtt->setServices();

  rclcpp::spin(rtt);
  rclcpp::shutdown();
  return 0;    
}
