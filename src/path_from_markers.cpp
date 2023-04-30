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
* Author:  Heath AScott-Evans
*********************************************************************/
/*WIP
Generate a smooth path between a set of markers

*/
#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs::Bool.h>
ros::Publisher initialPosePub,finalPosePub,pathPub,smoothedPathPub;
ros::Subscriber markers_sub, generate_path_sub;

geometry_msgs::PoseStamped pose;
nav_msgs::Path path, smoothedPath;
visualization_msgs::MarkerArray markers
double pointsPerUnit;
int skipPoints;
bool useEndConditions,useMiddleConditions,path_input; 

geometry_msgs::Point mid_point_calc (geometry_msgs::Point left, geometry_msgs::Point right)
{
  geometry_msgs::Point mid_point;
  mid_point.x = (left.x+right.x)/2;
  mid_point.y = (left.y+right.y)/2;
  mid_point.z = 0;
  return mid_point
}
void markers_callback(visualization_msgs::MarkerArray marker_input) //marker_array load to global variable
{
    markers.markers = marker_input.markers;
}

// make a service
void generate_path(std_msgs::Bool data)
{
    if (data.data)    //iterate marker_array odds = left evens = right
    {
        for (i=0 ; i <= markers.markers.size(); i++) //needs to iterate by 2's
        {
            geometry_msgs::Point left, right, mid;
            right.x = markers[i].markers.pose.position.x
            right.y = markers[i].markers.pose.position.y
            left.x = markers[i+1].markers.pose.position.x
            left.y = markers[i+1].markers.pose.position.y
            mid = mid_point_calc(left,right);     //mid_point calc -> mid_path

            path.poses.push_back(mid);
        }
            //mid_path -> smoothed path

    }
}

void path_from_path_generator_callback(const nav_msgs::Path inputpath)
{
  // create a cubic spline interpolator if path >1 point
  if(inputpath.poses.size() >1)
  {
    //creat object csi
    smoothedPath.header.frame_id = inputpath.header.frame_id;
    path_smoothing::CubicSplineInterpolator csi(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    csi.interpolatePath(inputpath, smoothedPath);
    initialPosePub.publish(smoothedPath.poses.front());
    finalPosePub.publish(smoothedPath.poses.back());
    smoothedPathPub.publish(smoothedPath);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_ros_wrapper");
  ros::NodeHandle nh("~");
  //ROS_INFO_STREAM("Namespace:" << nh.getNamespace());
  nh.param<double>("points_per_unit", pointsPerUnit, 5.0);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);


  initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
  finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
  smoothedPathPub = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);

  markers_sub = nh.subscribe("/marker_array" , 10, markers_callback);
  generate_path_sub = nh.subscribe("/generate_path", 10, generate_path_callback);

ros::Rate rate(100);
while(ros::ok())
{
  ros::spinOnce();
  rate.sleep();
  // ros::spin();

}
return 0;
}