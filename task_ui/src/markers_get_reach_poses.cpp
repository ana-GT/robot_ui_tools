
#include <task_ui/markers_get_reach_poses.h>
#include <vision_msgs/msg/bounding_box3_d.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkersGetReachPoses::MarkersGetReachPoses(rclcpp::Node::SharedPtr _node) :
RobotTaskMarkers(_node)
{

}

/**
 * @function init
 */
void MarkersGetReachPoses::init_(const std::string &_chain_group)
{
  // Services to use to call reachability-related queries
  client_ = node_->create_client<reachability_msgs::srv::GenerateReachPoses>("generate_reach_poses");
                             
  publisher_marker_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("show_ee_poses", 10);

  // Interactive marker stuff
  menu_handler_.insert( "Get Robot pose", std::bind(&MarkersGetReachPoses::processFeedback, this, _1));
  //interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  //menu_handler_.insert( sub_menu_handle, "First Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));
  //menu_handler_.insert( sub_menu_handle, "Second Entry", std::bind(&MarkersGetRobotBase::processFeedback, this, _1));

}


// %Tag(processFeedback)%
void MarkersGetReachPoses::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{

  switch ( feedback->event_type )
  {
  case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {
          RCLCPP_ERROR(node_->get_logger(), "Processing feeback from markers_get_reach_poses's menu select. Marker size: %d", marker_names_.size());
       auto request = std::make_shared<reachability_msgs::srv::GenerateReachPoses::Request>();

      // Get feedback poses
      if( marker_names_.size() != 1)
      {
         return;
      }

      int idx = 0;
      visualization_msgs::msg::InteractiveMarker im;
      if(!server_->get(marker_names_[idx], im))
          return;
  
      vision_msgs::msg::BoundingBox3D bbox;
      bbox.center = im.pose;
      bbox.size.x = 0.15;
      bbox.size.y = 0.15;
      bbox.size.z = 0.15;
      request->bbox = bbox;
      request->frame_id = feedback->header.frame_id; //reference_frame_;
      request->num_poses = 8; // TODO Make it parameter, no hard-code

      while (!client_->wait_for_service(1s)) {
      
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(node_->get_logger(), "service not available, waiting again...");
      }
    
      RCLCPP_INFO(node_->get_logger(), "Sending request for manipulation task plan");
      auto result = client_->async_send_request(request, 
                       std::bind(&MarkersGetReachPoses::client_cb, this, std::placeholders::_1));
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
 *
 */
void MarkersGetReachPoses::client_cb(rclcpp::Client<reachability_msgs::srv::GenerateReachPoses>::SharedFuture _future)
{
  auto status = _future.wait_for(1s);
  if (status == std::future_status::ready)
  {
      RCLCPP_INFO(node_->get_logger(), "Status is ready?");
      auto response = _future.get();
      RCLCPP_INFO(node_->get_logger(), "Got response with %lu solutions", response->ee_poses.size());

    visualization_msgs::msg::MarkerArray md;
    visualization_msgs::msg::Marker mdi;
    mdi.action = visualization_msgs::msg::Marker::DELETEALL;
    md.markers.push_back(mdi);
    publisher_marker_->publish(md);
    usleep(0.1*1e6);

    //RCLCPP_INFO(node_->get_logger(), "Showing %ld solutions", response->solutions.size());
    int id = 0;
    visualization_msgs::msg::MarkerArray ma;
    for(auto pi : response->ee_poses)
    {
      //RCLCPP_INFO(node_->get_logger(), "EE Pose: %f %f %f", pi.pose.position.x, pi.pose.position.y, pi.pose.position.z);

      visualization_msgs::msg::Marker mi;
      createArrowMarker(mi, id, pi);
      id++;      
      ma.markers.push_back(mi);
    }

    publisher_marker_->publish(ma);
    // Show arm poses
    for(auto jsi : response->joint_states)
    {
      pub_js_->publish(jsi);
      usleep(1.0*1e6);
    }
  }     
      
}

void MarkersGetReachPoses::createArrowMarker(visualization_msgs::msg::Marker &_marker, 
                                             const int &_id, 
                                             const geometry_msgs::msg::PoseStamped &_pi)
{  RCLCPP_WARN(node_->get_logger(), "arrow with header: %s", _pi.header.frame_id.c_str());
   _marker.header.frame_id = _pi.header.frame_id;
   _marker.header.stamp = node_->now();
   _marker.ns = "ee_pose";
   _marker.id = _id;
   _marker.type = visualization_msgs::msg::Marker::ARROW;
   _marker.action = visualization_msgs::msg::Marker::ADD;
   _marker.pose = _pi.pose;
  _marker.scale.x = 0.01; // shaft diameter
  _marker.scale.y = 0.015; // head diameter
  _marker.scale.z = 0.01; // arrow height
  _marker.color.a = 1.0; // Don't forget to set the alpha!
  _marker.color.r = 0.2;
  _marker.color.g = 1.0;
  _marker.color.b = 0.2;
  
  geometry_msgs::msg::Point ps, pe;
  pe.x = 0.0; //_marker.pose.position.x;
  pe.y = 0.0; //_marker.pose.position.y;
  pe.z = 0.0; //_marker.pose.position.z;
  
  Eigen::Quaterniond q(_pi.pose.orientation.w, _pi.pose.orientation.x, _pi.pose.orientation.y, _pi.pose.orientation.z);
  Eigen::Matrix3d m; m = q.matrix();
  Eigen::Vector3d zdir; zdir = m.col(2);
  
  double length = 0.04;
  ps.x = pe.x - 0.0*length;
  ps.y = pe.y - 0.0*length;
  ps.z = pe.z - 1.0*length;
  //RCLCPP_WARN(node_->get_logger(), "Point s: %f %f %f, point g: %f %f %f", ps.x, ps.y, ps.z, pe.x, pe.y, pe.z);
  
  _marker.points.push_back(ps);
  _marker.points.push_back(pe);
    
}



