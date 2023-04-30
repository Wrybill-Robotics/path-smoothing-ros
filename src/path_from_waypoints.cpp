/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Heath Ascott-Evans
*********************************************************************/
/* 
  Generate a path from waypoints
*/
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher pathPub,;
ros::Subscriber waypoint_sub, path_sub;

geometry_msgs::PoseStamped pose;
nav_msgs::Path path;

void path_from_pose_generator_callback(const geometry_msgs::PoseWithCovarianceStamped wp)
{
  path.header.frame_id = "map"; //use wp frame_id ?
  pose.header.frame_id = "map"; //use wp frame_id?
  pose.pose.position=wp.pose.pose.position;
  pose.pose.orientation = wp.pose.pose.orientation;
  path.poses.push_back(pose);
  pathPub.publish(path);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_path");
  ros::NodeHandle nh;

  waypoint_sub = nh.subscribe("/input", 10, path_from_pose_generator_callback);
  pathPub = nh.advertise<nav_msgs::Path>("initial_path", 1, true);

  ros::Rate rate(10);
  while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      // ros::spin();

    }
  return 0;
}