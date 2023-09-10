#include <task_ui/task_ui.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "task_ui_test");
  ros::NodeHandle nh;

  TaskUi tu;
  tu.init("arm");

  ros::spin();
  return 0;
}
