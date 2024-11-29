#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/MoveToWaypointAction.h>
#include <geographic_msgs/GeoPoint.h>

namespace osdk = dji_osdk_ros;

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_mission_client");

    // Action client
    actionlib::SimpleActionClient<osdk::MoveToWaypointAction> ac("waypoint_control", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Action server started, sending waypoints.");

    // List of waypoints to navigate to, hard-coded for testing
    std::vector<geographic_msgs::GeoPoint> waypoints = {
        [] { geographic_msgs::GeoPoint p; p.latitude = 5.0; p.longitude = 10.0; p.altitude = 115.0; return p; }(),
        [] { geographic_msgs::GeoPoint p; p.latitude = 5.0003; p.longitude = 10.0001; p.altitude = 115.0; return p; }(),
        [] { geographic_msgs::GeoPoint p; p.latitude = 5.0003; p.longitude = 10.0001; p.altitude = 105.0; return p; }()
    };

    for (size_t i = 0; i < waypoints.size(); i++) {
        ROS_INFO_STREAM("Sending waypoint. Latitude: " <<  waypoints[i].latitude << ", Longitude: " << waypoints[i].longitude << ", Altitude: " << waypoints[i].altitude);
        
        osdk::MoveToWaypointGoal goal;
        goal.goal_position = waypoints[i];

        ac.sendGoal(goal);

        // Wait for the result
        bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

        if (finished_before_timeout) {
            auto result = ac.getResult();
            if (result->success) {
                ROS_INFO("Reached waypoint %zu successfully: %s", i + 1, result->message.c_str());
            } else {
                ROS_WARN("Failed to reach waypoint %zu: %s", i + 1, result->message.c_str());
                break;
            }
        } else {
            ROS_ERROR("Timed out waiting for action server to complete waypoint %zu.", i + 1);
            break;
        }
    }

    ROS_INFO("Mission complete.");
    return 0;
}
