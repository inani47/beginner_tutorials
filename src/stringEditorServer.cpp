#include "ros/ros.h"
#include "beginner_tutorials/stringEditor.h"


bool editor(beginner_tutorials::stringEditor::Request  &req,
       beginner_tutorials::stringEditor::Response &res) {
  res.output = "counting to 100: ";
  ROS_INFO("sending back response");
  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "stringEditorServer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("editBaseString", editor);
  ros::spin();

  return 0;
}

