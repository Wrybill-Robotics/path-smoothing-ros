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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <nav_msgs/Path.h>

//globals
ros::Publisher pathPub;

bool save_waypoint(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
    /*  look up TF base_link->map
        write to file

    */
    geometry_msgs::PoseWithCovarianceStamped waypoint;
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    // ros::Time time_now = ros::Time::now();
        try
        {
            transformStamped= tfBuffer.lookupTransform("map","base_link",ros::Time(0),ros::Duration(3.0));
            //wait for transform base_link to map
            // listener.waitForTransform("base_link","map",time_now,ros::Duration(3.0));
            // listener.transformPoint("map",)
            waypoint.pose.pose.position.x=transformStamped.transform.translation.x;
            waypoint.pose.pose.position.y=transformStamped.transform.translation.y;
            waypoint.pose.pose.position.z=0;
            waypoint.pose.pose.orientation=transformStamped.transform.rotation;


            res.success=true;
            res.message="waypoint_saved x:" + std::to_string(waypoint.pose.pose.position.x) + " y:" + std::to_string(waypoint.pose.pose.position.y);
            
            //write to file 
            std::ofstream file;
            file.open("position.csv",std::ios::app);
            file << waypoint.pose.pose.position.x << "," << waypoint.pose.pose.position.y << "," << waypoint.pose.pose.orientation.x<< "," << waypoint.pose.pose.orientation.y<< "," << waypoint.pose.pose.orientation.z<< "," << waypoint.pose.pose.orientation.w<<"\n";
            file.close();
        }
        catch (tf2::TransformException &ex)
        {
                    ROS_WARN("%s", ex.what());

            res.success=false;
            res.message="failed to save waypoint";
        }
    return true;
}

bool load_waypoint(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
    /*  read waypoints.csv
        iterate waypoints and publish to /input        
    */
    std::ifstream file("position.csv");
        if (!file.is_open()) {
            ROS_ERROR("Failed to open the CSV file");
            res.success=false;
            res.message="failed to open csv file";
            return true;
        }

        std::string line;
        nav_msgs::Path path;
        path.header.frame_id = "map"; //use wp frame_id ?


        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string x_str, y_str,qx_str,qy_str,qz_str,qw_str;

            // Parse the comma-separated values
            if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',') && std::getline(iss, qx_str, ',') && std::getline(iss, qy_str, ',') && std::getline(iss, qz_str, ',') && std::getline(iss, qw_str, ',')) {
                // Convert the string values to doubles
                double x, y,qx,qy,qz,qw;
                try {
                    x = std::stod(x_str);
                    y = std::stod(y_str);
                    qx = std::stod(qx_str);
                    qy = std::stod(qy_str);
                    qz = std::stod(qz_str);
                    qw = std::stod(qw_str);

                } catch (const std::exception& ex) {
                    ROS_WARN_STREAM("Failed to parse values for line: " << line);
                    continue;
                }

                // Create a PoseStamped message and populate it with the loaded point
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.header.frame_id = "map";
                poseStamped.pose.position.x = x;
                poseStamped.pose.position.y = y;
                poseStamped.pose.orientation.x = qx;
                poseStamped.pose.orientation.y = qy;
                poseStamped.pose.orientation.z = qz;
                poseStamped.pose.orientation.w = qw;

                path.poses.push_back(poseStamped);
            }
        }
        pathPub.publish(path);

        res.success=true;
        res.message="Points loaded";
        return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"waypoint_saver");
    ros::NodeHandle nh;

    //services
    ros::ServiceServer saver = nh.advertiseService("save_waypoint", &save_waypoint);
    ros::ServiceServer loader = nh.advertiseService("load_waypoint", &load_waypoint);
    //publishers
    pathPub = nh.advertise<nav_msgs::Path>("initial_path", 1, true);


    //spin
    ros::spin();

    return 0;
}