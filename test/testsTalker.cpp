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
 *  @file testsTalker.cpp
 *
 *  @brief Contains tests for the talker node
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include <sstream>
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/stringEditor.h"

/**
 * @brief Unit test for string editor service
 *
 * Checks if the service outputs the desired
 * constant output string for any input string
 */

TEST(TESTTalker, stringEditorService) {
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

/**
 * @brief Unit test for tf transform
 *
 * Checks if the x coordinate of the origin
 * of the transform is as expected.
 */

TEST(TESTTalker, tf) {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/talk", "/world", ros::Time(0),
                            ros::Duration(10.0));
  listener.lookupTransform("/talk", "/world", ros::Time(0), transform);
  EXPECT_NEAR(-1.5384, static_cast<float>(transform.getOrigin().x()), 0.1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testsTalker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

