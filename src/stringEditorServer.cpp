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
 *  @file stringEditorServer.cpp
 *
 *  @brief Server for string editor service
 *
 *  This service modifies a given string
 *  to a constant output string
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include "ros/ros.h"
#include "beginner_tutorials/stringEditor.h"
/**
 * @brief callback function for the server
 *
 * @param req is the string change request sent by the client
 * @param res is the response string sent by the server
 *
 * @return true if the response is successful

 */
bool editor(beginner_tutorials::stringEditor::Request &req,
            beginner_tutorials::stringEditor::Response &res) {
  res.output = "counting to 100: ";
  ROS_INFO_STREAM("sending back response");
  return true;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "stringEditorServer");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("editBaseString", editor);
  ros::spin();
  return 0;
}
