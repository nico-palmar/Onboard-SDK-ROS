#include <ros/ros.h>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <dji_osdk_ros/AltitudeControlAction.h>
#include <dji_osdk_ros/common_type.h>
#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

namespace osdk = dji_osdk_ros;

class AltitudeControlActionServer
{
public:
  AltitudeControlActionServer(std::string name) :
      as_(nh_, name, false),
      action_name_(name)
  {
      //register the goal and feeback callbacks
      as_.registerGoalCallback(boost::bind(&AltitudeControlActionServer::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&AltitudeControlActionServer::preemptCB, this));

      set_joystick_mode_client_ = nh_.serviceClient<osdk::SetJoystickMode>("set_joystick_mode");
      joystick_action_client_   = nh_.serviceClient<osdk::JoystickAction>("joystick_action");
      
      // subscribe to the position in order to run the action
      gps_sub_ = nh_.subscribe("dji_osdk_ros/gps_position", 10, &AltitudeControlActionServer::gpsPosCallback, this);

      vel_sub_ = nh_.subscribe("dji_osdk_ros/velocity", 10, &AltitudeControlActionServer::measuredVelocityCallback, this);

      // create a timer which is one shot and will not start by default
      altitude_mission_timeout_ = nh_.createTimer(ros::Duration(1.0), &AltitudeControlActionServer::missionOutOfTime, this, true, false);
      as_.start();
  }

  ~AltitudeControlActionServer(void) {}

  void goalCB()
  {
    const auto goal_ptr = as_.acceptNewGoal();
    if (goal_ptr != nullptr)
    {
      altitude_mission_timeout_.setPeriod(ros::Duration(goal_ptr->mission_timeout));
      altitude_mission_timeout_.start();
      // set a flag to update the starting position
      start_pos_.update = true;
      // accept the new goal
      goal_altitude_ = goal_ptr->target_relative_altitude;
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
    altitude_mission_timeout_.stop();
    as_.setPreempted();
  }

  void missionOutOfTime(const ros::TimerEvent&)
  {
    if (!as_.isActive())
      return;

    // timer has elapsed, abort the mission
    result_.success = false;
    result_.message = "The altitude goal has timed out!";
    ROS_WARN("The altitude goal timed out!");
    // stop the drone
    osdk::JoystickCommand vel_command { 0, 0, 0, 0};
    velocityControl(vel_command);

    as_.setAborted(result_);
  }

  float saturateSpeed(const float& speed)
  {
    float ret = speed;
    if (std::abs(speed) > std::abs(speed_saturation_))
    {
      if (speed > 0)
      {
        ret = speed_saturation_;
      }
      else
      {
        ret = -speed_saturation_;
      }
    }
    return ret;
  }

  void measuredVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
  {
    measured_speed_.x = msg->vector.x;
    measured_speed_.y = msg->vector.y;
    measured_speed_.z = msg->vector.z;
  }

  bool isGoalComplete(const float& altitude)
  {
    // check that the altitude is within some bounds of the goal
    // and that the current speed is low enough
    const auto altitude_bound { 1e-2 };
    const auto speed_bound { 1e-2 };
    const auto within_altitude_bound = std::abs(std::abs(altitude) - std::abs(goal_altitude_)) <= altitude_bound;
    const auto within_speed_bound = std::abs(measured_speed_.z) <= speed_bound;
    return within_altitude_bound && within_speed_bound;
  }

  // Goal Callback method
  void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    if (!as_.isActive())
      return;

    if (start_pos_.update)
    {
      // set the start position if not yet set
      start_pos_.pos = *msg;
      start_pos_.update = false;
    }
    curr_pos_ = *msg;

    // run the control loop
    // calculate the current altitude rise
    const float current_altitude = curr_pos_.altitude - start_pos_.pos.altitude;

    // calculate the error
    const float altitude_error = goal_altitude_ - current_altitude;

    // calculate the velocity required and cap it at the saturation
    auto speed = Kp * altitude_error;
    speed = saturateSpeed(speed);

    // calculate the totoal percent risen
    feedback_.altitude_rise_pct = current_altitude / goal_altitude_;
    as_.publishFeedback(feedback_);

    // control the drone z component (up or down)
    osdk::JoystickCommand vel_command { 0, 0, speed, 0};
    velocityControl(vel_command);

    // check if the goal has succeeded
    if(isGoalComplete(current_altitude))
    {
      // stop the timer
      altitude_mission_timeout_.stop();

      // stop the drone
      osdk::JoystickCommand vel_command { 0, 0, 0, 0 };
      velocityControl(vel_command);
      result_.success = true;
      result_.message = "Success, reached target relative altitude of " + std::to_string(goal_altitude_) + " meters";

      ROS_INFO_STREAM(action_name_.c_str() << " Succeeded");
      as_.setSucceeded(result_);
    }
  }

private:

  void velocityControl(const osdk::JoystickCommand &command)
  {
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
  actionlib::SimpleActionServer<osdk::AltitudeControlAction> as_;
  std::string action_name_;
  float goal_altitude_;
  osdk::AltitudeControlResult result_;
  osdk::AltitudeControlFeedback feedback_;

  struct GPSWithUpdate
  {
    bool update { true };
    sensor_msgs::NavSatFix pos;
  };

  // store data needed for feedback control
  sensor_msgs::NavSatFix curr_pos_;
  GPSWithUpdate start_pos_;

  // gain for the proportional controller
  const float Kp { 0.9 };

  // velocity saturation [m/s]
  const int speed_saturation_ { 10 };

  geometry_msgs::Vector3 measured_speed_;

  // timeout for mission
  ros::Timer altitude_mission_timeout_;
  
  // clients to control drone's motion
  ros::ServiceClient set_joystick_mode_client_;
  ros::ServiceClient joystick_action_client_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "altitude_control_action_server");

  AltitudeControlActionServer server("control_altitude");
  ros::spin();
  return 0;
}
