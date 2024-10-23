
#include <task_ui/markers_get_robot_base.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkersGetRobotBase::MarkersGetRobotBase(rclcpp::Node::SharedPtr _node) :
RobotTaskMarkers(_node)
{

}


/**
 * @function init
 */
void MarkersGetRobotBase::init_(const std::string &_chain_group)
{
  // Services to use to call reachability-related queries
  client_ = node_->create_client<reachability_msgs::srv::MoveRobotToTask>("robot_to_task");


  // Interactive marker stuff
  menu_handler_.insert( "Get Robot pose", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));
  menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));

}


// %Tag(processFeedback)%
void MarkersGetRobotBase::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  std::ostringstream s;
  s << "* Feedback from marker '" << feedback->marker_name;

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
      RCLCPP_INFO_STREAM( node_->get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked");      

       auto request = std::make_shared<reachability_msgs::srv::MoveRobotToTask::Request>();

      // Get feedback poses
      for(int i = 0; i < marker_names_.size(); ++i)
      {
        visualization_msgs::msg::InteractiveMarker im;
        if(!server_->get(marker_names_[i], im))
          return;
        geometry_msgs::msg::PoseStamped pi;
        pi.pose = im.pose;
        pi.header.frame_id = reference_frame_;
  
        request->tcp_poses.push_back(pi);
      }

       request->num_robot_placements = params_.num_robot_placements;
      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(node_->get_logger(), "service not available, waiting again...");
      }
    
      RCLCPP_INFO(node_->get_logger(), "Sending request for manipulation task plan");
      auto result = client_->async_send_request(request, 
                       std::bind(&MarkersGetRobotBase::client_cb, this, std::placeholders::_1));
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

void MarkersGetRobotBase::client_cb(rclcpp::Client<reachability_msgs::srv::MoveRobotToTask>::SharedFuture _future)
{
  auto status = _future.wait_for(1s);
  if (status == std::future_status::ready)
  {
      RCLCPP_INFO(node_->get_logger(), "Status is ready?");
      auto response = _future.get();
      RCLCPP_INFO(node_->get_logger(), "Got response with %lu solutions", response->solutions.size());

    RCLCPP_INFO(node_->get_logger(), "Showing %ld solutions", response->solutions.size());
    for(auto si : response->solutions)
    {
      showSolution(si);
      usleep(1.0*1e6);
    }

  }
      
      
}


/**
 * @function showSolution
 */
void MarkersGetRobotBase::showSolution(reachability_msgs::msg::PlaceRobotSolution &_msg)
{
    // Move base
    //RCLCPP_INFO(node_->get_logger(), "Showing solutions: moving base and showing %d arm sols", _msg.chain_sols.size());
    moveBase(_msg.base_pose);
    // Update arm pose
    for(auto cs : _msg.chain_sols)
    {
      pub_js_->publish(cs);
      usleep(1.0*1e6);
    }
}


