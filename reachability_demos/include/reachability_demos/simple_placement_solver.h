#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <robot_sim_msgs/srv/move_robot_to_task.hpp>
#include <robot_sim_msgs/srv/set_robot_pose.hpp>


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

   bool getFK( const Eigen::VectorXd &_qs, Eigen::Isometry3d &_Tfx);
   bool getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx);
   bool projectConstraints(Eigen::Isometry3d &_Tfs);
   
   void handleSrv(const std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Request> req,
                  std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Response> res);
 
   rclcpp::Service<robot_sim_msgs::srv::MoveRobotToTask>::SharedPtr srv_;

   std::vector<int> higher_indices_;
   double above_ratio_;
   double below_z_comp_ = 0.1;

   //nlopt::opt opt_;

   // Read parameters
   KDL::Chain chain_;

   std::string chain_group_;
   std::string robot_name_;
   std::string robot_base_frame_;
   
   std::string chain_root_link_;
   std::string chain_tip_link_;
      
   std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_;
   std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
   
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

