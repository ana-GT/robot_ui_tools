/**
 * @file simple_placement_solver.cpp
 */
#include <rclcpp/rclcpp.hpp>
#include <robot_sim_msgs/srv/move_robot_to_task.hpp>
#include <robot_sim_msgs/srv/set_robot_pose.hpp>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <cfloat>
#include <nlopt.h>
#include <trac_ik/trac_ik.hpp>

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

/**
 * @class RobotToTask
 **/
class RobotToTask : public rclcpp::Node
{
 public:

  // Constructor
  RobotToTask() :
    rclcpp::Node("robot_to_task")
  {
    //this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
    this->declare_parameter("chain_root_link", std::string(""));
    this->declare_parameter("chain_tip_link", std::string(""));    
  }

  // Initialize
  bool initialize()
  {
//     this->get_parameter("chain_group_name", chain_group_);
     if(!this->get_parameter("robot_name", robot_name_))
       return false;
       
     if(!this->get_parameter("chain_root_link", chain_root_link_))
       return false;
       
     if(!this->get_parameter("chain_tip_link", chain_tip_link_))
       return false;
            
     //if( chain_group_.empty() || robot_name_.empty() )
     //  return false;

    double ik_max_time = 0.005;
    double ik_epsilon = 0.001;
    TRAC_IK::SolveType ik_type = TRAC_IK::SolveType::Speed;
     
    ik_solver.reset( new TRAC_IK::TRAC_IK(this->shared_from_this(), chain_root_link_, chain_tip_link_, 
                   "robot_description", 
                   ik_max_time, ik_epsilon, ik_type));

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


// Offer service to get pose
bool setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<robot_sim_msgs::srv::MoveRobotToTask>("robot_to_task", 
                std::bind(&RobotToTask::handleSrv, this, _1, _2));

    return true;
}

bool withinZThresh(Eigen::Isometry3d _Tfx, double _x, double _y, double _z, double _thresh)
{
    return ( fabs(_z - _Tfx.translation()(2)) <= _thresh );
}


// Handle service
void handleSrv(const std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Request> req,
               std::shared_ptr<robot_sim_msgs::srv::MoveRobotToTask::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received service to send robot to task!!!");

  if(req->tcp_poses.empty())
    res->success = false;

  // 1. Get all voxels that have a Z value in a threshold of this z
  // Only use 1 for now
  Eigen::Isometry3d Tfx;
  tf2::fromMsg(req->tcp_poses[0].pose, Tfx);
  double z_task = Tfx.translation()(2);

   std::vector<std::pair<int, int> > distances;
   
   // 
   res->success = true;
   robot_sim_msgs::msg::PlaceRobotSolution sol;
   sol.base_pose = req->tcp_poses[0];
   //sol.chain_sols = ;
   res->solutions.push_back(sol);
 
}

 protected:
 
 rclcpp::Service<robot_sim_msgs::srv::MoveRobotToTask>::SharedPtr srv_;

 std::vector<int> higher_indices_;
 double above_ratio_;
 double below_z_comp_ = 0.1;

 //nlopt::opt opt_;

   // Read parameters
   std::string chain_group_;
   std::string robot_name_;
   
   std::string chain_root_link_;
   std::string chain_tip_link_;
      
   std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
};


////////////////////////////////////

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   std::shared_ptr<RobotToTask> rtt = std::make_shared<RobotToTask>();
   RCLCPP_INFO(rtt->get_logger(), "Start node...");

  RCLCPP_INFO(rtt->get_logger(), "Initializing...");
  if(!rtt->initialize())
    return 1;
  RCLCPP_INFO(rtt->get_logger(), "Set services...");
  // Offer service
  rtt->setServices();
  RCLCPP_INFO(rtt->get_logger(), "Start spinning...");
  rclcpp::spin(rtt);
  rclcpp::shutdown();
  return 0;    
}
