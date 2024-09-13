#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <cmath>

// Helper function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to convert yaw to quaternion
void yawToQuaternion(double yaw, geometry_msgs::Quaternion& quat) {
    double half_yaw = yaw / 2.0;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = sin(half_yaw);
    quat.w = cos(half_yaw);
}

// Function to load waypoints from a JSON file
std::vector<visualization_msgs::Marker> loadWaypoints(const std::string& filepath) {
    std::vector<visualization_msgs::Marker> markers;
    Json::Value root;
    std::ifstream file(filepath, std::ifstream::binary);

    // Check if the file opened successfully
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filepath.c_str());
        return markers;
    }

    // Try to parse the JSON file
    file >> root;

    // Check for JSON parsing errors
    if (root.isNull() || !root.isMember("locations")) {
        ROS_ERROR("Failed to parse JSON file: %s", filepath.c_str());
        return markers;
    }

    int marker_id = 0; // ID counter for markers

    for (const auto& loc : root["locations"]) {
        // Create an arrow marker for the waypoint
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "waypoints";
        arrow_marker.id = marker_id++; // Unique ID for each marker
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.pose.position.x = loc["Position_X"].asDouble();
        arrow_marker.pose.position.y = loc["Position_Y"].asDouble();
        arrow_marker.pose.position.z = 0.0;

        // Set orientation based on Yaw
        double yaw = degreesToRadians(loc["Yaw"].asDouble());
        yawToQuaternion(yaw, arrow_marker.pose.orientation);

        // Dimensions for the arrow
        arrow_marker.scale.x = 0.5;  // Length of the arrow
        arrow_marker.scale.y = 0.1;  // Arrow width
        arrow_marker.scale.z = 0.1;  // Arrow height

        // Color for the arrow
        arrow_marker.color.a = 1.0;  
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 1.0;
        arrow_marker.color.b = 0.0;

        markers.push_back(arrow_marker);

        // Create a text marker for the waypoint name
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "waypoints";
        text_marker.id = marker_id++; 
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = loc["Position_X"].asDouble();
        text_marker.pose.position.y = loc["Position_Y"].asDouble();
        text_marker.pose.position.z = 1.0; 
        text_marker.pose.orientation.w = 1.0; 

        text_marker.scale.z = 0.6; 
        text_marker.color.a = 1.0;
        text_marker.color.r = 0.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0;
        text_marker.text = loc["Name"].asString();
        markers.push_back(text_marker);
    }

    return markers;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoints_marker_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // Get the file path from the parameter server
    std::string file_path;
    private_nh.param("json_file_path", file_path, std::string("default/path/to/json"));

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    std::vector<visualization_msgs::Marker> markers = loadWaypoints(file_path);

    ros::Rate r(1);
    while (ros::ok()) {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers = markers;

        // Set the header time to the current time
        for (auto& marker : marker_array.markers) {
            marker.header.stamp = ros::Time::now(); 
        }

        marker_pub.publish(marker_array);
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
