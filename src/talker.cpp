/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2017, Pranav Inani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file talker.cpp
 *
 *  @brief A publisher node
 *
 *  Publishes 100 messages on topic chatter
 *  then stops
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/stringEditor.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  ros::Rate loop_rate(10);
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::stringEditor
      > ("editBaseString");
  beginner_tutorials::stringEditor str;
  std::string baseStr = "The count is: ";
  str.request.input = baseStr;
  client.call(str);
  baseStr = str.response.output;
  int count = 0;
  /**
   *  This loop receives a modified string from
   *  the string edittor service then it publishes
   *  100 messages on topic chatter at 10 hz
   */
  while (ros::ok()) {
    ROS_DEBUG_STREAM_ONCE("ROS is OK");
    std_msgs::String msg;
    std::stringstream ss;
    ss << baseStr << count;
    msg.data = ss.str();
    if (count == 50) {
      ROS_WARN_STREAM("Reached half way" << std::endl << msg.data);
    } else if (count == 90) {
      ROS_ERROR_STREAM(" 10 messages to go" << std::endl << msg.data);
    } else if (count == 100) {
      ROS_FATAL_STREAM("100 messages printed" << std::endl << msg.data);
      break;
    } else {
    ROS_INFO_STREAM(msg.data);
    }
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

