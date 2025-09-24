
#include <task_ui/markers_get_grasp_object_poses.h>
//#include <vision_msgs/msg/bounding_box3_d.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkersGetGraspObjectPoses::MarkersGetGraspObjectPoses(const std::string &_server_name) :
RobotTaskMarkers(_server_name)
{

}

/**
 * @function init
 */
bool MarkersGetGraspObjectPoses::init_(const std::string &_chain_group)
{
  // Services to use to call reachability-related queries
  group_ = _chain_group;
  client_ = this->create_client<reachability_msgs::srv::GetGraspObjectPoses>("get_grasp_object_poses");


  // Interactive marker stuff
  menu_handler_.insert( "Get Grasp Object poses", std::bind(&MarkersGetGraspObjectPoses::processFeedback, this, _1));
  //interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  //menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));
  //menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));

  return true;
}


// %Tag(processFeedback)%
void MarkersGetGraspObjectPoses::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{

  switch ( feedback->event_type )
  {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
       RCLCPP_ERROR(this->get_logger(), "Processing feeback from markers_get_reach_poses's menu select. Marker size: %ld", marker_names_.size());
       auto request = std::make_shared<reachability_msgs::srv::GetGraspObjectPoses::Request>();

      // Get feedback poses
      if( marker_names_.size() != 1)
      {
         return;
      }
	
      int idx = 0;
      visualization_msgs::msg::InteractiveMarker im;
      if(!server_->get(marker_names_[idx], im))
          return;
  
      //request->group_name = group_;
      //request->init_joint_state; // empty: Will use the current joint state
      request->object_pose = im.pose;
      doubleArrayToPose(params_.grasp_offset_0, request->grasp_offset);
      request->frame_id = im.header.frame_id;
      
      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
      }
    
      RCLCPP_INFO(this->get_logger(), "Sending request for manipulation task plan");
      auto result = client_->async_send_request(request, 
                       std::bind(&MarkersGetGraspObjectPoses::client_cb, this, std::placeholders::_1));
     // Do not wait for result or crash. No nested wait spinning
    }
    break;

  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      //updatePose(goal_pose_, feedback->pose, feedback->header);
    }
    break;

  }

  server_->applyChanges();
}

/**
 * @function client_cb
 */
void MarkersGetGraspObjectPoses::client_cb(rclcpp::Client<reachability_msgs::srv::GetGraspObjectPoses>::SharedFuture _future)
{
  auto status = _future.wait_for(1s);
  if (status == std::future_status::ready)
  {
      RCLCPP_INFO(this->get_logger(), "Status is ready?");
      auto response = _future.get();

    
    if(response->success)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully gotten solution");
        //RCLCPP_INFO(this->get_logger(), "Number of solutions: %ld", response->solutions.size());
        
        for(auto op : response->object_poses)
        {
          visualization_msgs::msg::InteractiveMarker im;
          //server_->get(marker_names_[0], im));
          server_->setPose(marker_names_[0], op.pose, op.header);
          server_->applyChanges();
          //pub_js_->publish(si.arm_config);
          //moveBase(si.base_pose);
          //usleep(1.0*1e6);
        }
    }
  }     
      
}


