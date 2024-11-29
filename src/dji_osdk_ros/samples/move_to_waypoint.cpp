#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dji_osdk_ros/MoveToWaypointAction.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <string>
#include <Eigen/Dense>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>

namespace osdk = dji_osdk_ros;

namespace impulse_control
{

class WaypointControlActionServer
{
public:
    WaypointControlActionServer(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&WaypointControlActionServer::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&WaypointControlActionServer::preemptCB, this));

        set_joystick_mode_client_ = nh_.serviceClient<osdk::SetJoystickMode>("set_joystick_mode");
        joystick_action_client_   = nh_.serviceClient<osdk::JoystickAction>("joystick_action");

        // subscribe to the position in order to run the action
        gps_sub_ = nh_.subscribe("dji_osdk_ros/gps_position", 10, &WaypointControlActionServer::gpsPosCallback, this);

        vel_sub_ = nh_.subscribe("dji_osdk_ros/velocity", 10, &WaypointControlActionServer::measuredVelocityCallback, this);

        as_.start();
    }

    ~WaypointControlActionServer(void) {}

private:

    void goalCB()
    {
        const auto goal_ptr = as_.acceptNewGoal();
        if (goal_ptr != nullptr)
        {
            goal_pos_ = goal_ptr->goal_position;
        }
        else
        {
            throw std::runtime_error("The goal pointer was null!");
        }
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // stop the drone
        osdk::JoystickCommand vel_command { 0, 0, 0, 0 };
        velocityControl(vel_command);
        as_.setPreempted();
    }

    void saturateVelocity(Eigen::Vector3d& velocity)
    {
        const float speed = velocity.norm();
        if (speed > std::abs(speed_saturation_))
        {
            // multiply the velocity unit vector by the saturation speed
            // to get a velocity vector with magnitude of the saturation speed
            velocity = (velocity / speed) * speed_saturation_;
        }
    }

    void measuredVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        measured_speed_ = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
    }

    Eigen::Vector3d getNEDError(const geographic_msgs::GeoPoint& current_position, const geographic_msgs::GeoPoint& target_position)
    {
        // Convert GPS (WGS84) to UTM
        geodesy::UTMPoint ref_utm, target_utm;
        geodesy::fromMsg(current_position, ref_utm);
        geodesy::fromMsg(target_position, target_utm);

        // Calculate NED offsets
        Eigen::Vector3d ned_error { target_utm.northing - ref_utm.northing, target_utm.easting - ref_utm.easting, -(target_position.altitude - current_position.altitude) };
        ROS_DEBUG_STREAM("The NED Error is N: " << ned_error.x() << ", E: " << ned_error.y() << " D: " << ned_error.z());
        return ned_error;
    }

    bool isGoalComplete(const Eigen::Vector3d& ned_error)
    {
        // check that the altitude is within some bounds of the goal
        // and that the current speed is low enough
        const auto position_bound { 1e-1 };
        const auto speed_bound { 1e-2 };
        const auto within_position_bound = ned_error.norm() <= position_bound;
        const auto within_speed_bound = measured_speed_.norm() <= speed_bound;
        return within_position_bound && within_speed_bound;
    }

    geographic_msgs::GeoPoint navSatFixtoGeoPoint(const sensor_msgs::NavSatFix& gps_pos)
    {
        geographic_msgs::GeoPoint ret;
        ret.latitude = gps_pos.latitude;
        ret.longitude = gps_pos.longitude;
        ret.altitude = gps_pos.altitude;
        return ret;
    }

    void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (!as_.isActive())
            return;

        curr_pos_ = navSatFixtoGeoPoint(*msg);

        // run the control loop
        // calculate the error from the current position to the goal
        const auto ned_error = getNEDError(curr_pos_, goal_pos_);

        // calculate the velocity required and cap it at the saturation
        Eigen::Vector3d ned_velocity = Kp.array() * ned_error.array();
        saturateVelocity(ned_velocity);

        // calculate the euclidean distance to the goal
        const auto distance_to_goal = ned_error.norm();
        feedback_.dist_to_goal = distance_to_goal;
        as_.publishFeedback(feedback_);

        // control the drone in the correct coordinate system
        const auto vel_x = static_cast<DJI::OSDK::float32_t>(ned_velocity.x());
        const auto vel_y = static_cast<DJI::OSDK::float32_t>(ned_velocity.y());
        const auto vel_z = static_cast<DJI::OSDK::float32_t>(-ned_velocity.z());
        osdk::JoystickCommand vel_command { vel_x, vel_y, vel_z, 0};
        velocityControl(vel_command);

        // check if the goal has succeeded
        if(isGoalComplete(ned_error))
        {
            // stop the drone
            osdk::JoystickCommand vel_command { 0, 0, 0, 0 };
            velocityControl(vel_command);
            result_.success = true;
            result_.message = "Success, reached target position";

            ROS_INFO_STREAM(action_name_.c_str() << " Succeeded");
            as_.setSucceeded(result_);
        }
    }

    void velocityControl(const osdk::JoystickCommand &command)
    {
        // note that the horizontal coordinate system is hardcoded to interial frame
        osdk::SetJoystickMode joystickMode;
        osdk::JoystickAction joystickAction;
        joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
        joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
        joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
        joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
        joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
        set_joystick_mode_client_.call(joystickMode);

        joystickAction.request.joystickCommand.x = command.x;
        joystickAction.request.joystickCommand.y = command.y;
        joystickAction.request.joystickCommand.z = command.z;
        joystickAction.request.joystickCommand.yaw = command.yaw;
        joystick_action_client_.call(joystickAction);
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<osdk::MoveToWaypointAction> as_;
    std::string action_name_;
    geographic_msgs::GeoPoint goal_pos_;
    osdk::MoveToWaypointResult result_;
    osdk::MoveToWaypointFeedback feedback_;

    geographic_msgs::GeoPoint curr_pos_;

    // gain for each axis... x, y, and z
    const Eigen::Vector3d Kp { 0.9, 0.9, 0.9 };

    // velocity saturation [m/s]
    // max magnitude of the velocity vector
    const float speed_saturation_ { 10.0 };

    Eigen::Vector3d measured_speed_;

    ros::ServiceClient set_joystick_mode_client_;
    ros::ServiceClient joystick_action_client_;
    
    ros::Subscriber gps_sub_;
    ros::Subscriber vel_sub_;
};

} // impulse_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_control_action_server");
    impulse_control::WaypointControlActionServer server("waypoint_control");
    ros::spin();
    return 0;
}
