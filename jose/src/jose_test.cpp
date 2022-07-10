#include <jose/jose.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "jose_test");
  ros::NodeHandle nh;

  Jose jose;
  jose.init("arm");

  ros::spin();
  return 0;
}
