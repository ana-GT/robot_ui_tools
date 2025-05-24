/**
 * @file simple_placement_solver.cpp
 */
#include <reachability_demos/simple_placement_solver.h>

#include <algorithm>
#include <cfloat>
#include <nlopt.h>

//#include <reachability_description/reachability_description.h>
//#include <reachability_description/reach_utilities.h>

using namespace std::chrono_literals;

/**
 * @function manip_comp
 */
bool dist_comp ( std::pair<int, double> _i,
                 std::pair<int, double> _j ) {
  return (_i.second < _j.second );
}

bool samp_comp ( std::pair<int, int> _i,
                 std::pair<int, int> _j ) {
  return (_i.second < _j.second );
}

struct GS
{
  Eigen::Isometry3d G;
  Eigen::Isometry3d S;
};

double calc_error(Eigen::Isometry3d _S, Eigen::Isometry3d _G, std::vector<double> _x)
{

  double fx, fy, zx, zy;
  double G03, G13, G02, G12;
  double S03, S13, S02, S12;

  G03 = _G.matrix()(0,3);
  G13 = _G.matrix()(1,3);
  G02 = _G.matrix()(0,2);
  G12 = _G.matrix()(1,2);
  S03 = _S.matrix()(0,3);
  S13 = _S.matrix()(1,3);
  S02 = _S.matrix()(0,2);
  S12 = _S.matrix()(1,2);
  fx = G03 - _x[0] - cos(_x[2])*S03 - sin(_x[2])*S13;
  fy = G13 - _x[1] - sin(_x[2])*S03 - cos(_x[2])*S13;
  zx = G02 - cos(_x[2])*S02 - sin(_x[2])*S12;
  zy = G12 - sin(_x[2])*S02 - cos(_x[2])*S12;


  return (fx*fx + fy*fy + zx*zx  + zy*zy);
}

double min_func(const std::vector<double> &x, std::vector<double>& grad, void* data)
{
  // x
  // tx = x[0] ty = x[1] theta = x[2]
  GS* c = (GS*) data;

  Eigen::Isometry3d S, G; // sample and goal
  S = c->S;
  G = c->G;
  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double res = calc_error(S, G, x);
  
  if (!grad.empty())
  {
    double v1;
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      v1 = calc_error(S, G, vals);

      vals[i] = original;
      grad[i] = (v1 - res) / (2 * jump);
    }
  }

  return res;
}

bool withinZThresh(Eigen::Isometry3d _Tfx, double _x, double _y, double _z, double _thresh)
{
    return ( fabs(_z - _Tfx.translation()(2)) <= _thresh );
}


/**
 * Constructor
 */
RobotToTask::RobotToTask() :
rclcpp::Node("robot_to_task")
{
    //this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
    this->declare_parameter("chain_root_link", std::string(""));
    this->declare_parameter("chain_tip_link", std::string(""));
    this->declare_parameter("robot_base_frame", std::string(""));    
}

/*
 * Initialize
 */
bool RobotToTask::initialize()
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     this->get_parameter("chain_group_name", chain_group_);
     if(!this->get_parameter("robot_name", robot_name_))
       return false;
       
     if(!this->get_parameter("chain_root_link", chain_root_link_))
       return false;
       
     if(!this->get_parameter("chain_tip_link", chain_tip_link_))
       return false;

     if(!this->get_parameter("robot_base_frame", robot_base_frame_))
       return false;
            
     if( chain_root_link_.empty() || chain_tip_link_.empty() || robot_base_frame_.empty() )
       return false;

    double ik_max_time = 0.005;
    double ik_epsilon = 0.001;
    TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Speed;
     
    ik_solver_.reset( new TRAC_IK::TRAC_IK(this->shared_from_this(), chain_root_link_, chain_tip_link_, 
                   "robot_description", 
                   ik_max_time, ik_epsilon, ik_type));

    // Create IK and FK solver
    ik_solver_->getKDLChain(chain_);
    unsigned int num_joints = chain_.getNrOfJoints();
    unsigned int num_segments = chain_.getNrOfSegments();
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));


    //rd_->addKinematicSolvers(_chain_group);


     //above_ratio_ = 0.35;
     //below_z_comp_ = 0.15; // (nx, ny, nz) vs (mx, my, mz), this factor is diff between nz and mx, at most it can be 2

     //opt_ = nlopt::opt(nlopt::LD_SLSQP, 3); // nlopt::LD_SLSQP
     //opt_.set_xtol_abs(0.01);
     //opt_.set_maxtime(0.005);
    
     //std::vector<double> x_lower_bounds = {-10.0, -10.0, -M_PI};
     //std::vector<double> x_upper_bounds = {10.0, 10.0, M_PI};

     //opt_.set_lower_bounds(x_lower_bounds);
     //opt_.set_upper_bounds(x_upper_bounds);

    return true;
}

/**
 * Offer service to get pose
 */
bool RobotToTask::setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<robot_sim_msgs::srv::MoveRobotToTask>("robot_to_task", 
                std::bind(&RobotToTask::handleSrv, this, _1, _2));

    return true;
}


/**
 * Handle service
 */
void RobotToTask::handleSrv(const std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Request> req,
               std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received service to send robot to task!!!");

  if(req->tcp_poses.empty())
    res->success = false;

  // 1. Get all voxels that have a Z value in a threshold of this z
  // Only use 1 for now
  Eigen::Isometry3d Tf_world_ee_goal;
  tf2::fromMsg(req->tcp_poses[0].pose, Tf_world_ee_goal);

  std::vector<std::pair<int, int> > distances;
      
   Eigen::Isometry3d Tf_base_root;   
   getTransform(robot_base_frame_, chain_root_link_, Tf_base_root);   
   
   Eigen::Isometry3d Tf_world_base_guess, Tf_root_ee;
   geometry_msgs::msg::Pose msg_world_base_guess;

   // FK()
   // "torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
   Eigen::VectorXd js(8);
   js << 0.0, 0.27, -0.165, -1.827, 2.07, 1.18, -0.65, -1.04;
   getFK(js, Tf_root_ee);
   
    // First guess: 
   Tf_world_base_guess = Tf_world_ee_goal * Tf_root_ee.inverse() * Tf_base_root.inverse();
   
   // Project
   projectConstraints(Tf_world_base_guess);
   // Try IK
   //FK  = Tf_base_root.inverse()* Tf_world_base_guess.inverse() * Tf_world_ee_goal;
   //ik_solver_->
   
   msg_world_base_guess = tf2::toMsg(Tf_world_base_guess);
   
   res->success = true;
   robot_sim_msgs::msg::PlaceRobotSolution sol;
   sol.base_pose.pose = msg_world_base_guess;
   //sol.chain_sols = ;
   res->solutions.push_back(sol);
 
}

bool RobotToTask::projectConstraints(Eigen::Isometry3d &_Tfs)
{
   Eigen::Vector3d zs, zg;
   
   // 1. Get Z current
   zs = _Tfs.linear().col(2);
   zg = Eigen::Vector3d(0,0,1);
   
   // 2. Calculate rotation to Z(1,0,0)
   Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(zs, zg);
   
   // 3. Apply rotation
   _Tfs.linear() = q * _Tfs.linear();
   
   // 4. Project to floor
   _Tfs.translation()(2) = 0.0;
   return true;   
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from " << _source << " to " << _target
                                                       << ".  Error: " << ex.what());
      return false;
   }
   
   _Tfx = tf2::transformToEigen(tfxs);
   return true;
}

/** 
 * @function getFK
 */
bool RobotToTask::getFK( const Eigen::VectorXd &_qs, Eigen::Isometry3d &_Tfx)
{
   if(_qs.rows() != chain_.getNrOfJoints())
   {  
      RCLCPP_ERROR(this->get_logger(), "FK input argument has wrong size: %d, should be %d", _qs.rows(), chain_.getNrOfJoints());
      return false;
   }
   
   KDL::JntArray q;
   q.data = _qs;
   
   KDL::Frame Tf_kdl;
   if( fk_solver_->JntToCart(q, Tf_kdl) < 0 )
     return false;
     
   tf2::transformKDLToEigen(Tf_kdl, _Tfx);
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
