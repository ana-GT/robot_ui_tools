#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <robot_sim_msgs/srv/set_robot_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @class MoveBase
 */
class MoveBase : public rclcpp::Node
{
public:
  MoveBase() : 
  rclcpp::Node("simulate_robot_base_motion") 
  {
     this->declare_parameter("ref_frame", std::string(""));
     this->declare_parameter("robot_frame", std::string(""));

     this->declare_parameter("init_x", 0.0);
     this->declare_parameter("init_y", 0.0);
     this->declare_parameter("init_z", 0.0);
     this->declare_parameter("init_roll", 0.0);
     this->declare_parameter("init_pitch", 0.0);
     this->declare_parameter("init_yaw", 0.0);
                              
  }

  bool initialize()
  {  
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());

     this->get_parameter("ref_frame", ref_frame_);
     this->get_parameter("robot_frame", robot_frame_);

     RCLCPP_INFO(this->get_logger(), "Ref frame: %s, robot frame: %s", ref_frame_.c_str(), robot_frame_.c_str());

     // Set translation
     geometry_msgs::msg::Pose pose;
     this->get_parameter("init_x", pose.position.x);
     this->get_parameter("init_y", pose.position.y);
     this->get_parameter("init_z", pose.position.z);

     RCLCPP_INFO(this->get_logger(), "Initial position: %f %f %f", pose.position.x, pose.position.y, pose.position.z);

     // Set orientation     
     double roll, pitch, yaw;
     this->get_parameter("init_roll", roll);
     this->get_parameter("init_pitch", pitch);
     this->get_parameter("init_yaw", yaw);

     RCLCPP_INFO(this->get_logger(), "Initial RPY: %f %f %f", roll, pitch, yaw);

     tf2::Quaternion q;
     q.setRPY(roll, pitch, yaw);
     q.normalize();
     pose.orientation = tf2::toMsg(q);

     if(ref_frame_.empty() || robot_frame_.empty())
     {
        RCLCPP_ERROR(this->get_logger(), "Reference frame or robot frame are empty!");
        return false;
     }

     updatePose(pose, ref_frame_);

    // Set service server
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv_ = this->create_service<robot_sim_msgs::srv::SetRobotPose>("set_robot_pose", 
                std::bind(&MoveBase::handleSrv, this, _1, _2));

    return true;
  }

void handleSrv(const std::shared_ptr<robot_sim_msgs::srv::SetRobotPose::Request> req,
               std::shared_ptr<robot_sim_msgs::srv::SetRobotPose::Response> res)
  {
    // Update
    RCLCPP_INFO(this->get_logger(), "Got service call to move base to %f %f %f", req->pose.pose.position.x,
                req->pose.pose.position.y, req->pose.pose.position.z); 
    updatePose(req->pose.pose, req->pose.header.frame_id);
    res->success = true;
  }

void updatePose(geometry_msgs::msg::Pose _pose, std::string _frame_id)
{
    mux_.lock();
    
    tfx_.transform.translation.x = _pose.position.x;
    tfx_.transform.translation.y = _pose.position.y;
    tfx_.transform.translation.z = _pose.position.z;
    tfx_.transform.rotation = _pose.orientation;
    tfx_.header.stamp = this->now();

    tfx_.header.frame_id = _frame_id;
    if(_frame_id.empty())
        tfx_.header.frame_id = ref_frame_;
    tfx_.child_frame_id = robot_frame_;
    
    mux_.unlock();
}

void publishPose()
{
    mux_.lock();
    
    tfx_.header.stamp = this->now();
    tf_static_broadcaster_->sendTransform(tfx_);
    
    mux_.unlock();
}

protected:
   rclcpp::Service<robot_sim_msgs::srv::SetRobotPose>::SharedPtr srv_;
   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
   std::string ref_frame_;
   std::string robot_frame_;
   std::mutex mux_;

   geometry_msgs::msg::TransformStamped tfx_;

  };

/////////////////////////////////////////
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveBase>();

  // Hand to user app
  if(!node->initialize())
    return 1;

  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->publishPose();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;    

}
