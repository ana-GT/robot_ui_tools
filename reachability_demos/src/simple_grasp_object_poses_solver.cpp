/**
 * @file simple_grasp_object_poses_solver.cpp
 */
#include <reachability_demos/simple_grasp_object_poses_solver.h>

#include <algorithm>
#include <cfloat>
#include <nlopt.h>

#include <reachability_description/reach_utilities.h>
#include <reachability_description/reach_graph.h>

using namespace std::chrono_literals;

const auto logger = rclcpp::get_logger("grasp");

/**
 * Constructor
 */
GraspObjectPosesSolver::GraspObjectPosesSolver() :
rclcpp::Node("grasp_object_placement_solver")
{
    this->declare_parameter("chain_group_name", std::string(""));
    this->declare_parameter("robot_name", std::string(""));
    
    pub_cloud_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug_object_poses", 10);
}

/*
 * Initialize
 */
bool GraspObjectPosesSolver::initialize()
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

    if(!rd_->loadDescription(chain_group_))
      return false;

   // Get chain info
   rd_->getChainInfo(chain_group_, ci_);

    return true;
}

/**
 * Offer service to get pose
 */
bool GraspObjectPosesSolver::setServices()
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<reachability_msgs::srv::GetGraspObjectPoses>("get_grasp_object_poses", 
                std::bind(&GraspObjectPosesSolver::handleSrv, this, _1, _2));

    return true;
}

bool compare_samples(reachability_msgs::msg::ReachData a, reachability_msgs::msg::ReachData b)
{
  return a.samples.size() > b.samples.size();
}

/**
 * Handle service
 */
void GraspObjectPosesSolver::handleSrv(const std::shared_ptr<reachability_msgs::srv::GetGraspObjectPoses::Request> req,
               std::shared_ptr<reachability_msgs::srv::GetGraspObjectPoses::Response> res)
{
  RCLCPP_INFO(logger, "Received service to query object poses for the given grasp");

  // Return the EE poses / joint states

  // Variables
  Eigen::Isometry3d Tf_base, Tf_object, Tf_ee_offset, Tf_ee;
  Eigen::Vector3d z_dir;

  // Get pose of object and offset
  tf2::fromMsg(req->object_pose, Tf_object);
  tf2::fromMsg(req->grasp_offset, Tf_ee_offset);
  
  // Transform object pose to chain.root_link, if needed
  getTransform(ci_.root_link, req->frame_id, Tf_base);

  Tf_ee = Tf_base * Tf_object * Tf_ee_offset;
  
  // Get the Z value of the TF
  double z = Tf_ee.translation()(2);
  z_dir = Tf_ee.linear().col(2);
  z_dir.normalize();
  // Select the reachability voxel values at this Z (approx)
  std::shared_ptr<reachability_description::ReachGraph> rg = rd_->getReachGraph(chain_group_);
  std::vector<reachability_msgs::msg::ReachData> neighbors, aligned;
  
  unsigned int min_samples, max_samples;
  rg->getMinMaxSamples(min_samples, max_samples);
  
  for(int i = 0; i < rg->getNumPoints(); ++i)
  {
    auto rdi = rg->getState(i);
    if(rdi.state == reachability_msgs::msg::ReachData::FILLED)
    {
       if( fabs( rdi.pose.position.z - z ) < rg->getResolution() )
          neighbors.push_back(rdi);       
    }
  }
    
  RCLCPP_INFO(logger, "Found %lu neighbors within a resolution distance from %f. Samples range from %d to %d", neighbors.size(), z, min_samples, max_samples);    
  
    
  // Start by selecting the nodes with samples ratio > 20%
  // For rotation around Z axis  from 0 to 360, if angle is within a range + axis close to Z, store (rotation)
  
  // If the above produces non-empty voxels, calculate IK for these, using the stored joint values as reference
  
  // Return Tf = Translation of voxel, rotation from the while
  int good_quality = 0;
  do {
  
    double ratio = 0.20;
    int min_aligned, max_aligned;
    min_aligned = 10000;
    max_aligned = 0;
    for(auto ni : neighbors)
    {
  	if(ni.samples.size() > (1.0 - ratio) * max_samples)
  	{
  	  good_quality++;
  	  
  	  int parallel = 0;
          auto pi = ni;
          pi.samples.clear();
  	  for(auto si : ni.samples)
  	  {
  	     Eigen::Quaterniond qs(si.pose.orientation.w, si.pose.orientation.x, si.pose.orientation.y, si.pose.orientation.z);
  	     Eigen::Matrix3d rot_s; Eigen::Vector3d zs; Eigen::Quaterniond q_theta;
  	     rot_s = qs.toRotationMatrix();
  	     zs = rot_s.col(2);
  	     zs.normalize();
  	     q_theta.setFromTwoVectors(z_dir, zs);
  	     Eigen::AngleAxisd aa(q_theta);
  	     Eigen::Vector3d ax; ax = aa.axis();
  	     if( fabs( ax.dot(Eigen::Vector3d(0,0,1) ) ) > cos(30.0/180.0 *3.1416) )
  	     {
//  	    RCLCPP_INFO(logger, " From sample (%.3f %.3f %.3f) to dir (%.3f %.3f %.3f), axis:  %.3f %.3f %.3f -- angle: %.3f", zs(0), zs(1), zs(2), z_dir(0), z_dir(1), z_dir(2), aa.axis()(0), aa.axis()(1), aa.axis()(2), aa.angle()*180.0/3.1416);
               pi.samples.push_back(si);
  	       parallel++;
  	     }
 	     
  	  } // for si : ni.samples

          if(parallel > 0)
          {
  //          RCLCPP_INFO(logger, " Sample %f %f %f with seemingly parallel approx directions. # of parallels: %d/%lu ", ni.pose.position.x, ni.pose.position.y, ni.pose.position.z, parallel, ni.samples.size());
            if(pi.samples.size() > max_aligned)
              max_aligned = pi.samples.size();
            if(pi.samples.size() < min_aligned)
              min_aligned = pi.samples.size();
                
            aligned.push_back(ni); // pi
          }
  	} // if
    }
    
    publishCloud(aligned, min_aligned, max_aligned);
    
    RCLCPP_INFO_STREAM(logger, "Tf: \n" << Tf_ee.matrix());
    RCLCPP_INFO(logger, "Samples with good quality (top %f): %d, aligned: %lu. Min: %d max: %d samples", ratio, good_quality, aligned.size(), min_aligned, max_aligned);
    break;
  
  } while(true);


  // Order based on num samples
  std::sort(aligned.begin(), aligned.end(), compare_samples);
  RCLCPP_INFO(logger, " Debugging, samples for first 4 ordered: %lu, %lu, %lu, %lu", aligned[0].samples.size(), aligned[1].samples.size(), aligned[2].samples.size(), aligned[3].samples.size() );
  // First
  RCLCPP_INFO(logger, "Pose initialization must be: %.3f %.3f %.3f", aligned[0].pose.position.x, aligned[0].pose.position.y, aligned[0].pose.position.z);
    
   geometry_msgs::msg::PoseStamped object;
   object.pose = aligned[0].pose;
   object.header.frame_id = ci_.root_link;
   res->object_poses.push_back(object);
   //res->joint_states.push_back(aligned[0]);
   res->success = true; 
  /*
  for(auto si : reach_data.samples)
  {
    geometry_msgs::msg::PoseStamped ps;
    sensor_msgs::msg::JointState js;
    
    ps.pose = si.pose;
    js = vectorToJointState(si.best_config, ci);
    
    //res->ee_poses.push_back(ps);
    //res->joint_states.push_back(js);
  }
*/
  //res->success = res->ee_poses.empty()? false : true; 
  
}

/**
 * @function publishCloud
 */
void GraspObjectPosesSolver::publishCloud(const std::vector<reachability_msgs::msg::ReachData> &_rdata, const int &_min_samples, const int &_max_samples)
{

  // Get pointcloud to visualize
  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.frame_id = ci_.root_link;
  cloud.width = _rdata.size();
  cloud.height = 1;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_rdata.size());

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");


  for( auto rdi : _rdata ) {
    
    *out_x = rdi.pose.position.x;
    *out_y = rdi.pose.position.y;
    *out_z = rdi.pose.position.z;
    
    if(rdi.samples.size() == _min_samples) {
      *out_r = 255; *out_g = 0; *out_b = 0;
    } else if(rdi.samples.size() >= _max_samples) {
      *out_r = 0; *out_g = 255; *out_b = 0;  
    } else {
      *out_r = 255; *out_g = 255; *out_b = 0;    
    }
    ++out_x; ++out_y; ++out_z;
    ++out_r; ++out_g; ++out_b;
  }

  pub_cloud_debug_->publish(cloud); 
}

/**
 * @function getTransform // (-0.062, 0.0, 0.291);
 */
bool GraspObjectPosesSolver::getTransform(const std::string &_source, const std::string &_target, Eigen::Isometry3d &_Tfx)
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
   std::shared_ptr<GraspObjectPosesSolver> rtt = std::make_shared<GraspObjectPosesSolver>();

  if(!rtt->initialize())
    return 1;

  // Offer service
  rtt->setServices();

  rclcpp::spin(rtt);
  rclcpp::shutdown();
  return 0;    
}
