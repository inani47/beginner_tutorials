#include <sstream>
#include <iomanip>
#include "ros/ros.h"
#include <gtest/gtest.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/stringEditor.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

TEST(TESTTalker,stringEditorService) {
  ros::NodeHandle n;
  ros::service::waitForService("editBaseString");
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::stringEditor
      > ("editBaseString");
  beginner_tutorials::stringEditor str;
  std::string baseStr = "The count is: ";
  str.request.input = baseStr;
  client.call(str);

  EXPECT_EQ("counting to 100: ", str.response.output);
}

TEST(TESTTalker, tf) {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/talk", "/world", ros::Time(0), ros::Duration(10.0) );
  listener.lookupTransform("/talk", "/world", ros::Time(0), transform);
  EXPECT_NEAR( -1.5384 , static_cast<float>(transform.getOrigin().x()), 0.1);
}

  
int main(int argc, char **argv) {
  ros::init(argc, argv, "testsTalker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

