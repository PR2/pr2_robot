/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pr2_msgs/PowerBoardState.h>
#include <std_srvs/Empty.h>
#include <pr2_power_board/PowerBoardCommand.h>


static bool last_e_stop_state_ = true;

void powerStateCallback(const pr2_msgs::PowerBoardStateConstPtr& msg)
{
  if (!last_e_stop_state_ && msg->run_stop){
    ROS_INFO("Run stop re-enabled. Bringing up power and resetting motors.");

    // enable power
    if (!ros::service::waitForService("power_board/control", ros::Duration(5.0))){
      ROS_ERROR("Could not find power board control service");
      return;
    }
    pr2_power_board::PowerBoardCommand power_board_cmd;
    power_board_cmd.request.serial_number = msg->serial_num;
    power_board_cmd.request.command = "start";
    for (unsigned int i=0; i<3; i++){
      power_board_cmd.request.breaker_number = i;
      ros::service::call("power_board/control", power_board_cmd);
      ROS_INFO("  - Enabling breaker %i: %i",i, power_board_cmd.response.retval);
    }

    // reset motors
    ros::Duration(2.0).sleep();  // give motors time to detect power up
    if (!ros::service::waitForService("pr2_ethercat/reset_motors", ros::Duration(5.0))){
      ROS_ERROR("Could not find reset motors service");
      return;
    }
    std_srvs::Empty empty_cmd;
    ros::service::call("pr2_ethercat/reset_motors", empty_cmd);
    ROS_INFO("  - Reset motors");
  }

  last_e_stop_state_ =  msg->run_stop;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_stop_helper");
  ros::NodeHandle node;

  // subscribe to state of power board
  ros::Subscriber power_state_sub = node.subscribe("power_board/state", 1, powerStateCallback);

  ros::spin();

  return 0;
}
