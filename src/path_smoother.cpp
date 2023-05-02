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
/* Generate a smooth path from inear path
TODO service for path reset/reload
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <tf/tf.h>
#include <std_srvs/Trigger.h>


ros::Publisher initialPosePub,finalPosePub,smoothedPathPub;
ros::Subscriber path_sub;

geometry_msgs::PoseStamped pose;
nav_msgs::Path path;

//spline interpolator variables
double pointsPerUnit;
int skipPoints;
bool useEndConditions,useMiddleConditions,path_input; 



void path_from_path_generator_callback(const nav_msgs::Path inputpath)
{
  // create a cubic spline interpolator if path >1 point
  nav_msgs::Path smoothedPath;
  smoothedPath.header.frame_id = inputpath.header.frame_id;
  if(inputpath.poses.size() >1)
  {
    //creat object csi


    path_smoothing::CubicSplineInterpolator csi(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    csi.interpolatePath(inputpath, smoothedPath);
    initialPosePub.publish(smoothedPath.poses.front());
    finalPosePub.publish(smoothedPath.poses.back());
  }
  else
  {
    ROS_INFO_STREAM("Path does not have enough points");
  }
  smoothedPathPub.publish(smoothedPath);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing");
  ros::NodeHandle nh;
  nh.param<double>("points_per_unit", pointsPerUnit, 5.0);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);

  initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
  finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
  smoothedPathPub = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);

  path_sub = nh.subscribe("/initial_path", 10, path_from_path_generator_callback);

ros::Rate rate(10);
while(ros::ok())
{
  ros::spinOnce();
  rate.sleep();
}
return 0;
}