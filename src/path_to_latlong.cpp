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
//TODO SAVE PATH TO FILE
//TODO SAVE PATH TO FILE

#include <ros/ros.h>
#include <navsat_transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <nav_msgs/Path.h>








        bool loadPathCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
        {
            // Clear the existing path
            path_.poses.clear();

            // Open the CSV file
            std::ifstream file("path.csv");
            if (!file)
            {
            res.success = false;
            res.message = "Failed to open CSV file";
            return true;
            }

            // Read the CSV file line by line
            std::string line;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";

            while (std::getline(file, line))
            {
            // Parse the line into x, y, and yaw values
            std::istringstream iss(line);
            std::string x_str, y_str, yaw_str;
            if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',') && std::getline(iss, yaw_str, ','))
            {
            // Create a PoseStamped message and add it to the path
                double x, y,yaw;
                try {
                    x = std::stod(x_str);
                    y = std::stod(y_str);
                    yaw = std::stod(yaw_str);

                } catch (const std::exception& ex) {
                    ROS_WARN_STREAM("Failed to parse values for line: " << line);
                    continue;
                }
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.orientation = yawToQuaternion(yaw);
                path_.poses.push_back(pose);
            }


            else
            {
                ROS_WARN_STREAM("Failed to parse line: " << line);
                continue;}

            }

            file.close();

            // Publish the loaded path
            pathPub_.publish(path_);

            res.success = true;
            res.message = "Path loaded and published";
            return true;
        }