#include <jose/jose.h>
#include <tf2/time.h>

Jose::Jose()
{}

/**
 * @function init
 */
void Jose::init(rclcpp::Node::SharedPtr _nh,
		std::string _group)
{
  nh_ = _nh;
  group_ = _group;
  
  std::string urdf_param = "robot_description";
  std::string srdf_param = "robot_description_semantic";

  std::string urdf_string;
  std::string srdf_string;

  urdf_string = nh_->declare_parameter(urdf_param, std::string(""));
  srdf_string = nh_->declare_parameter(srdf_param, std::string(""));

  if(urdf_string.empty() || srdf_string.empty())
  {
    RCLCPP_FATAL(nh_->get_logger(), "Could not load robot description and/or semantic");
    return;
  }
  
  urdf_ = urdf::parseURDF(urdf_string);
  urdf_model_.initString(urdf_string);

  if(!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failure parsing KDL tree \n");
    return;
  }
  
  srdf_.initString(*urdf_, srdf_string);

  std::string name = srdf_.getName();
  bool found = false;
  for(auto gi : srdf_.getGroups())
  {
    printf("\t * Group : %s. number of chains: %d joints: %d, links: %d \n",
	   gi.name_.c_str(),
	   gi.chains_.size(),
	   gi.joints_.size(),
	   gi.links_.size());
    if(gi.name_ == _group)
    {
      if(gi.chains_.size() == 1)
      {
	group_base_link_ = gi.chains_[0].first;
	group_tip_link_ = gi.chains_[0].second;
	printf("Get chain with base: %s and tip: %s \n",
	       group_base_link_.c_str(),
	       group_tip_link_.c_str());
	getJointNames();
	found = true;
      }
    }
  }
  
  printf("\t * Init jose. Found: %d  \n", found);
  
  trac_ik_.reset( new TRAC_IK::TRAC_IK(nh_, group_base_link_, group_tip_link_,
				       urdf_param) );
		  
}

void Jose::getJointNames()
{
  joints_.clear();

  KDL::Chain chain;
  if(!kdl_tree_.getChain(group_base_link_, group_tip_link_, chain))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failure parsing chain from %s to %s \n",
		 group_base_link_.c_str(),
		 group_tip_link_.c_str());
    return;
  }

  for(unsigned int i = 0; i < chain.segments.size(); i++)
  {
    std::string ji = chain.segments[i].getJoint().getName();
    if(chain.segments[i].getJoint().getType() != KDL::Joint::None)
    {
      joints_.push_back(ji);
      printf("\t * Adding joint %s joint type: %d \n",
	     ji.c_str(), chain.segments[i].getJoint().getType());
    }
    else
      printf("\t *    !!! Skipping joint %s joint type: %d \n",
	     ji.c_str(), chain.segments[i].getJoint().getType());
      
  }

  printf("\t ** Joints of size: %d \n", joints_.size());
  return;
}

bool Jose::getIK(Eigen::Vector3d _pos,
		 Eigen::Quaterniond _rot,
		 std::vector<double> &_joints,
		 KDL::Twist _bounds)
{
  KDL::Frame p_in;
  KDL::JntArray q_out;
  KDL::JntArray q_init;

  q_init = getMidJoint();
  
  p_in.p = KDL::Vector(_pos.x(), _pos.y(), _pos.z());
  p_in.M = KDL::Rotation::Quaternion(_rot.x(), _rot.y(), _rot.z(), _rot.w());
  
  int res = trac_ik_->CartToJnt(q_init, p_in, q_out, _bounds);

  _joints.clear();
  if(res > 0)
  {
    for(int i = 0; i < q_out.data.size(); ++i)
      _joints.push_back(q_out.data(i));
    
    return true;
  }

  return false;
}


bool Jose::getFK(std::vector<double> _joints,
		 Eigen::Isometry3d &_pose)
{

}

KDL::JntArray Jose::getMidJoint()
{
  KDL::JntArray lb, ub, mid;
  trac_ik_->getKDLLimits(lb, ub);

  mid.data.resize(lb.data.size());
  for(int i = 0; i < lb.data.size(); ++i)
  {
    mid.data(i) = (lb.data(i) + ub.data(i))*0.5;
  }

  return mid;
}

bool Jose::getMsg(const std::vector<double> &_joint_vals,
		  sensor_msgs::msg::JointState &_msg)
{
  if(_joint_vals.size() != joints_.size())
  {
    printf("Mismatch of sizes: input: %d expected: %d \n",
	   _joint_vals.size(), joints_.size());
    return false;
  }

  _msg.header.stamp = nh_->get_clock()->now();
  _msg.header.frame_id = group_base_link_;
  _msg.name = joints_;
  _msg.position = _joint_vals;
  return true;
}
