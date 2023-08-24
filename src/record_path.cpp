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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/impl/convert.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class PathRecorder
{
    public:
        double distance_threshold;
        nav_msgs::Path path_;
        bool is_recording;

        PathRecorder()  //default constructor
        {
            ros::NodeHandle nh;    
            nh.param<double>("minimum_distance" , distance_threshold, 0.5);
            nh.param<std::string>("path_file",file_location_,"path.csv");
            ROS_INFO_STREAM("file path is: " <<file_location_);
            is_recording=false;

            path_.header.frame_id="map";
            recorder= nh.advertiseService("record_path", &PathRecorder::record_srv,this);
            saver= nh.advertiseService("save_path", &PathRecorder::save_to_file,this);
            loader= nh.advertiseService("load_path",&PathRecorder::loadPathCallback,this);
            clearCSV = nh.advertiseService("clear_saved_path",&PathRecorder::clearCSVCallback,this);
            pathPub_ = nh.advertise<nav_msgs::Path>("path", 1, true);

        }  

        void record_path()
        {
            //load current pose
            //check pose is safe
            //if safe add to path
            if (isPoseWithinDistance(grab_pose()))
            {
                addPose(waypoint);
            }
        }

        void addPose(const geometry_msgs::PoseStamped& pose)
        {
            // Add the pose to the path
            path_.poses.push_back(pose);
        }

        bool record_srv(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res)
        {
            if (req.data)
            {
                //start recording path
                is_recording=req.data;
                res.success=true;
                res.message="started recording path";
            }
            else
            {        
                //stop recording
                is_recording=req.data;
                res.success=true;
                res.message="stopped recording path";
            }
            return true;
        }

        bool save_to_file(std_srvs::Trigger::Request &treq,std_srvs::Trigger::Response &tres)
        {
            // Check if the path is empty
            if (path_.poses.empty())
            {
                tres.success = false;
                tres.message = "Path is empty";
                return true;
            }

            // Iterate over the poses in the path and save x, y, and yaw values
            for (const geometry_msgs::PoseStamped& pose : path_.poses)
            {
                double x = pose.pose.position.x;
                double y = pose.pose.position.y;
                double yaw = quaternionToYaw(pose.pose.orientation);

                // Save the x, y, and yaw values to the CSV file
                saveToCSV(x, y, yaw);
            }
            ROS_INFO_STREAM("saved path to file: " << file_location_);

            tres.success = true;
            tres.message = "Path saved to CSV";
            return true;
        
        }

        bool loadPathCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
        {
            // Clear the existing path
            path_.poses.clear();
            ROS_INFO_STREAM("opening file: " << file_location_);
            // Open the CSV file
            std::ifstream file(file_location_);
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
        
        bool clearCSVCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
        {
            // Open the CSV file in truncation mode (clearing the data)
            std::ofstream file(file_location_, std::ios::trunc);
            if (!file)
            {
            res.success = false;
            res.message = "Failed to clear CSV file";
            return true;
            }

            file.close();

            res.success = true;
            res.message = "CSV file cleared";
            return true;
        }
    private:
    geometry_msgs::PoseStamped waypoint;
    ros::ServiceServer recorder;
    ros::ServiceServer saver;
    ros::ServiceServer loader;
    ros::ServiceServer clearCSV;
    ros::Publisher pathPub_;
    std::string file_location_;
    //look up pose in map
    geometry_msgs::PoseStamped grab_pose()
    {
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
                waypoint.header.frame_id="map";
                waypoint.pose.position.x=transformStamped.transform.translation.x;
                waypoint.pose.position.y=transformStamped.transform.translation.y;
                waypoint.pose.position.z=transformStamped.transform.translation.z;
                waypoint.pose.orientation=transformStamped.transform.rotation;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
        return waypoint;
    }

    //check pose is safe distance
    bool isPoseWithinDistance(const geometry_msgs::PoseStamped& input_pose)
    {
        // Check if the path is empty, add first pose as current pose
        if (path_.poses.empty())
        {
            path_.poses.push_back(input_pose);
            return false;

        }

        // Get the last pose in the path
        const geometry_msgs::PoseStamped& last_pose = path_.poses.back();

        // Calculate the Euclidean distance between the input pose and the last pose
        double dx = input_pose.pose.position.x - last_pose.pose.position.x;
        double dy = input_pose.pose.position.y - last_pose.pose.position.y;
        double dz = input_pose.pose.position.z - last_pose.pose.position.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        // Check if the distance is within the threshold
        return distance >= distance_threshold;
    }

    double quaternionToYaw(const geometry_msgs::Quaternion &quaternion)
    {
        tf2::Quaternion tfQuaternion;
        tf2::convert(quaternion, tfQuaternion);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tfQuaternion).getRPY(roll, pitch, yaw);

        return yaw;
    }

    geometry_msgs::Quaternion yawToQuaternion(double yaw)
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);

        geometry_msgs::Quaternion msgQuaternion;
        tf2::convert(quaternion, msgQuaternion);

        return msgQuaternion;
    }

      void saveToCSV(double x, double y, double yaw)
    {
        // Open the CSV file in append mode
        std::ofstream file(file_location_, std::ios::app);

        // Write the x, y, and yaw values to the CSV file
        file << x << "," << y << "," << yaw << std::endl;

        // Close the CSV file
        file.close();
    }

};




int main(int argc, char** argv)
{
    ros::init(argc,argv,"waypoint_saver");
    PathRecorder path_recorder;

    //services
    //publishers

    ros::Rate rate(5.0);

    //spin at rate
    while (ros::ok())
    {
        if (path_recorder.is_recording)
        {
            path_recorder.record_path();
        }
        // pathPub.publish(path_recorder.path_);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}