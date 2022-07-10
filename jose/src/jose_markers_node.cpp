#include <jose/jose_markers.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jose_markers_node");
  ros::NodeHandle pnh("~");

  std::string group;
  
  if(!pnh.getParam("group", group))
  {
    ROS_ERROR("group parameter was not read!");
    return 0;
  }
  ROS_WARN("group parameter: %s \n", group.c_str() );
  
  JoseMarkers jm(pnh);
  jm.init(group);
  ros::spin();
  jm.stop();
}
// %EndTag(main)%

