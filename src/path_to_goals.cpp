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

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
class PathSubscriber
{
public:
  PathSubscriber()
  {
    // Initialize the ROS node handle
    // nh_ = ros::NodeHandle("~");
    ros::NodeHandle nh_;    

    // Create a service server to start the path processing
    start_service_ = nh_.advertiseService("path_to_waypoints", &PathSubscriber::startProcessingCallback, this);

    // Create a subscriber to the path topic
    path_subscriber_ = nh_.subscribe("path", 1, &PathSubscriber::pathCallback, this);

    // Create a publisher for the individual poses
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("waypoint_pose", 1);

  }

  bool startProcessingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    res.success = true;
    res.message = "Processing started";
    geometry_msgs::PoseWithCovarianceStamped pose_cov;
    pose_cov.header.frame_id = input_path_.header.frame_id;
    for (const geometry_msgs::PoseStamped& pose : input_path_.poses)
    {   

        pose_cov.pose.pose = pose.pose;
        pose_publisher_.publish(pose_cov);
        ros::Duration(0.1).sleep();  // Simulate processing time
    }
    return true;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
  {
    input_path_.header=path_msg->header;
    input_path_.poses=path_msg->poses;
  }

private:
//   ros::NodeHandle nh_;
  ros::ServiceServer start_service_;
  ros::Subscriber path_subscriber_;
  ros::Publisher pose_publisher_;
  bool processing_started_;
  nav_msgs::Path input_path_;
};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "path_to_waypoints");

  // Create an instance of PathSubscriber
  PathSubscriber path_subscriber;

  // ROS main loop
  ros::spin();

  return 0;
}
