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
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
ros::Publisher pathPub;
ros::Subscriber marker_sub, generate_path_sub;

geometry_msgs::PoseStamped pose;
nav_msgs::Path path;
//visualization_msgs::Marker marker;

geometry_msgs::Point mid_point_calc (geometry_msgs::Point left, geometry_msgs::Point right)
{
  geometry_msgs::Point mid_point;
  mid_point.x = (left.x+right.x)/2;
  mid_point.y = (left.y+right.y)/2;
  mid_point.z = 0;
  return mid_point;
}

geometry_msgs::Quaternion heading_calc(geometry_msgs::Point old_point, geometry_msgs::Point new_point)
{
  float delta_x,delta_y,yaw;
  delta_x=new_point.x-old_point.x;
  delta_y=new_point.y-old_point.y;
  yaw=atan2(delta_y,delta_x);

  tf::Matrix3x3 rot_euler;
  tf::Quaternion rot_quat;

  rot_euler.setEulerYPR(yaw, 0.0, 0.0);
  rot_euler.getRotation(rot_quat);
  rot_quat.normalize();

  geometry_msgs::Quaternion heading_q;
  heading_q.x = rot_quat.getX();
  heading_q.y = rot_quat.getY();
  heading_q.z = rot_quat.getZ();
  heading_q.w = rot_quat.getW();
  return heading_q;
}

void marker_callback(visualization_msgs::Marker marker) //marker_array load to global variable TODO make a class with data
{
    //marker = marker_input;
    geometry_msgs::Point left, right, mid, mid_prev;
      // Specify heading goal using current goal and next goal (final goal orientation straight line from starting point of goal)
      mid_prev.x=0.0;
      mid_prev.y=0.0;
      path.header.frame_id = "map"; //use wp frame_id ?
      pose.header.frame_id = "map"; //use wp frame_id?

      for (int i=0 ; i <= marker.points.size()-1; i++) //iterate through array finding midpoint between points
      {
        
          //geometry_msgs::Point left, right, mid;
          // left.x = marker.marker[i].pose.position.x;
          // left.y = marker.marker[i].pose.position.y;
          // right.x = marker.marker[i+1].pose.position.x;
          // right.y = marker.marker[i+1].pose.position.y;
          left=marker.points[i];
          right=marker.points[i+1];
          //mid_point calc -> mid_path
          mid = mid_point_calc(left,right);
          pose.pose.position=mid;
          //calculate heading based on previous pose
          pose.pose.orientation = heading_calc(mid_prev,mid);
          mid_prev=mid;
          path.poses.push_back(pose);
      }
      pathPub.publish(path);
}


// TODO make as service
// void generate_path_callback(std_msgs::Bool data)
// {
//     if (data.data)    //iterate marker_array assume zigzag
//     {            
//       geometry_msgs::Point left, right, mid, mid_prev;
//       // Specify heading goal using current goal and next goal (final goal orientation straight line from starting point of goal)
//       mid_prev.x=0.0;
//       mid_prev.y=0.0;
//       path.header.frame_id = "map"; //use wp frame_id ?
//       pose.header.frame_id = "map"; //use wp frame_id?

//       for (int i=0 ; i <= marker.points.size()-1; i++) //iterate through array finding midpoint between points
//       {
        
//           //geometry_msgs::Point left, right, mid;
//           // left.x = marker.marker[i].pose.position.x;
//           // left.y = marker.marker[i].pose.position.y;
//           // right.x = marker.marker[i+1].pose.position.x;
//           // right.y = marker.marker[i+1].pose.position.y;
//           left=marker.points[i];
//           right=marker.points[i+1];
//           //mid_point calc -> mid_path
//           mid = mid_point_calc(left,right);
//           pose.pose.position=mid;
//           //calculate heading based on previous pose
//           pose.pose.orientation = heading_calc(mid_prev,mid);
//           mid_prev=mid;
//           path.poses.push_back(pose);
//       }
//       pathPub.publish(path);
//     }
// }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_ros_wrapper");
  ros::NodeHandle nh;
  // subscribers
  marker_sub = nh.subscribe("/marker" , 10, marker_callback);
  // generate_path_sub = nh.subscribe("/generate_path", 10, generate_path_callback);
  // publishers
  pathPub = nh.advertise<nav_msgs::Path>("/initial_path", 1, true);

  //loop
  ros::Rate rate(10);
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();

    }
    return 0;
}