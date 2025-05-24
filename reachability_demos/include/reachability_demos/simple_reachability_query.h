#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <reachability_msgs/srv/generate_reach_poses.hpp>

#include <reachability_description/reachability_description.h>


/**
 * @class RobotToTask
 **/
class RobotToTask : public rclcpp::Node
{
 public:

  RobotToTask();
  bool initialize(); 
  bool setServices();

 protected:
   bool getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx);
   
   void handleSrv(const std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Request> req,
                  std::shared_ptr<reachability_msgs::srv::GenerateReachPoses::Response> res);
 
   rclcpp::Service<reachability_msgs::srv::GenerateReachPoses>::SharedPtr srv_;


   // Read parameters
   std::string chain_group_;
   std::string robot_name_;
   std::string robot_base_frame_;
            
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   

   std::shared_ptr<reachability_description::ReachabilityDescription> rd_;   
};

