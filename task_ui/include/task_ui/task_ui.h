#pragma once

/**
 * @file task_ui.h
 */
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <trac_ik/trac_ik.hpp>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>

/**
 * @class TaskUi
 */
class TaskUi {

public:
  TaskUi();
  void init(rclcpp::Node::SharedPtr _nh,
	    std::string _group);

  bool getIK(Eigen::Vector3d _pos,
	     Eigen::Quaterniond _rot,
	     std::vector<double> &_joints,
	     KDL::Twist _bounds = KDL::Twist::Zero());

  bool getFK(std::vector<double> _joints,
	     Eigen::Isometry3d &_pose);

  KDL::JntArray getMidJoint();

  std::string getBaseLink() { return group_base_link_; }
  std::string getTipLink() { return group_tip_link_; }

  bool getMsg(const std::vector<double> &_joints,
	      sensor_msgs::msg::JointState &_msg);
  void getJointNames();
  
protected:
  
  std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_;
  rclcpp::Node::SharedPtr nh_;
  std::string group_;
  std::vector<std::string> joints_;
  int num_joints_;

  urdf::ModelInterfaceSharedPtr urdf_;
  urdf::Model urdf_model_;
  srdf::Model srdf_;
  KDL::Tree kdl_tree_;

  std::string group_base_link_;
  std::string group_tip_link_;
};
