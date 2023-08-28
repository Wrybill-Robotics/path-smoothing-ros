#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <std_srvs/Trigger.h>

// Define a struct to hold GPS coordinates

int uR = 6378137;


struct GPSPoint {
    double latitude;
    double longitude;
};

class CSVToLatLongConverter {
private:

    std::string csv_file_path_;
    ros::ServiceServer trigger_service_;
public:
    GPSPoint datum_;

    CSVToLatLongConverter() {
            ros::NodeHandle nh_;

        nh_.param<std::string>("csv_file_path", csv_file_path_, "path.csv");
        nh_.param<double>("datum_latitude", datum_.latitude, -40.37978520);
        nh_.param<double>("datum_longitude", datum_.longitude, 175.61290360);
        trigger_service_ = nh_.advertiseService("trigger_conversion", &CSVToLatLongConverter::triggerConversion, this);
        // Subscribe to ROS topics, advertise services, etc.
    }

    GPSPoint convertToLatLong(const GPSPoint& point) {
        GPSPoint converted_point;
        // Convert point to latitude and longitude using datum
        // You need to implement the conversion logic here
        // Example: 
        // converted_point.latitude = datum_.latitude + point.latitude;
        // converted_point.longitude = datum_.longitude + point.longitude;
        converted_point.latitude=datum_.latitude + point.latitude/111111;
        converted_point.longitude=datum_.longitude + point.longitude/111111;
        // std::cout << converted_point.latitude << " " << converted_point.longitude << std::endl;
        return converted_point;
    }

    bool triggerConversion(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) 
    {   
        // std::cout << "trigger called" << std::endl;
        // Read and convert CSV here
        std::ifstream csv_file(csv_file_path_);
        if (!csv_file)
            {
            res.success = false;
            res.message = "Failed to open CSV file";
            return true;
            }
        std::string line;

        std::string new_csv_file_path = "gps_"+csv_file_path_ ; // Initialize with the same path

        std::ofstream new_csv_file(new_csv_file_path,std::ios::trunc); //open in trunc mode to clear file before writing
        new_csv_file << std::setprecision(10);
        // std::cout << "opened" <<std::endl;
        while (std::getline(csv_file, line)) {
            // std::cout << "while getline" << std::endl;
            std::istringstream iss(line);
            std::string x_str, y_str, yaw_str;
            GPSPoint point;

            if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',') && std::getline(iss, yaw_str, ','))
            {
                // std::cout << x_str << " " << y_str <<std::endl;
            // Create a PoseStamped message and add it to the path
                try {
                    point.latitude = std::stod(x_str);
                    point.longitude = std::stod(y_str);

                } catch (const std::exception& ex) {
                    ROS_WARN_STREAM("Failed to parse values for line: " << line);
                    continue;
                }
            }
            GPSPoint converted_point = convertToLatLong(point);
            // Process the converted point, e.g., save it to the new CSV file
            new_csv_file << converted_point.latitude << "," << converted_point.longitude << std::endl;
        }

        csv_file.close();
        new_csv_file.close();
        res.success = true;
        res.message = "converted and saved";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "csv_to_latlong_converter_node");

    CSVToLatLongConverter converter;
    std::cout << std::setprecision(10);
    std::cout << "datum: latitude =" << converter.datum_.latitude << " longitude =" << converter.datum_.longitude << std::endl;
    ros::spin();  // Keep the node running

    return 0;
}
