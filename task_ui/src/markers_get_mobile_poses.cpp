
#include <task_ui/markers_get_mobile_poses.h>
#include <vision_msgs/msg/bounding_box3_d.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkersGetMobilePoses::MarkersGetMobilePoses(const std::string &_server_name) :
RobotTaskMarkers(_server_name)
{

}

/**
 * @function init
 */
bool MarkersGetMobilePoses::init_(const std::string &_chain_group)
{
  // Services to use to call reachability-related queries
  group_ = _chain_group;
  client_ = this->create_client<reachability_msgs::srv::GetMobilePoses>("get_mobile_poses");


  // Interactive marker stuff
  menu_handler_.insert( "Get Mobile poses", std::bind(&MarkersGetMobilePoses::processFeedback, this, _1));
  //interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  //menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));
  //menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));

  return true;
}


// %Tag(processFeedback)%
void MarkersGetMobilePoses::processFeedback_( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{

  switch ( feedback->event_type )
  {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
       auto request = std::make_shared<reachability_msgs::srv::GetMobilePoses::Request>();

      // Get feedback poses
      if( marker_names_.size() != 1)
      {
         RCLCPP_ERROR(this->get_logger(), "Marker size should be 1, it is: %ld", marker_names_.size());
         return;
      }

      int idx = 0;
      visualization_msgs::msg::InteractiveMarker im;
      if(!server_->get(marker_names_[idx], im))
          return;
  
      request->group_name = group_;
      //request->init_joint_state; // empty: Will use the current joint state
      request->goal_pose.pose = im.pose;
      request->goal_pose.header = im.header;
      
      int index = getObjectIndex(steps_[0].object);
      if(index < 0)
        return;

      request->grasp_offset = objects_[index].grasp_offset;

      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
      }

      auto result = client_->async_send_request(request, 
                       std::bind(&MarkersGetMobilePoses::client_cb, this, std::placeholders::_1));
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
void MarkersGetMobilePoses::client_cb(rclcpp::Client<reachability_msgs::srv::GetMobilePoses>::SharedFuture _future)
{
  auto status = _future.wait_for(1s);
  if (status == std::future_status::ready)
  {
      RCLCPP_INFO(this->get_logger(), "Status is ready?");
      auto response = _future.get();

    
    if(response->success)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully gotten solution");
        RCLCPP_INFO(this->get_logger(), "Number of solutions: %ld", response->solutions.size());
        
        for(auto si : response->solutions)
        {
          pub_js_->publish(si.arm_config);
          moveBase(si.base_pose);
          usleep(1.0*1e6);
        }
    }
  }     
      
}


