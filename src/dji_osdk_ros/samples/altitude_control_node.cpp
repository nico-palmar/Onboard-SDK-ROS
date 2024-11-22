#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dji_osdk_ros/AltitudeControlAction.h>
// #include <std_msgs/Float32.h>  // For publishing altitude
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <sensor_msgs/NavSatFix.h>


class AltitudeControlActionServer
{
public:
  AltitudeControlActionServer(std::string name) :
      as_(nh_, name, false),
      action_name_(name)
  {
      //register the goal and feeback callbacks
      as_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

      set_joystick_mode_client_ = nh.serviceClient<osdk::SetJoystickMode>("set_joystick_mode");
      joystick_action_client_   = nh.serviceClient<osdk::JoystickAction>("joystick_action");
      obtain_ctrl_authority_client_ = nh.serviceClient<osdk::ObtainControlAuthority>("obtain_release_control_authority");
      
      // subscribe to the position in order to run the action
      gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &AltitudeControlActionServer::gpsPosCallback);
      as_.start();
  }

    ~AltitudeControlActionServer(void) {}

  void goalCB()
  {
    // accept the new goal
    goal_ = as_.acceptNewGoal()->target_relative_altitude;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  // Goal Callback method
  void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    if (!as_.isActive())
      return;

    curr_pos_ = *msg;
    
    // check if the message is too old, abort if this is the case

    // run the control loop in this function

    as_.publishFeedback(feedback_);

    // check if the goal should be aborted or has succeeded
    // sample code
    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<your_package::ControlAltitudeAction> as_;
  std::string action_name_;
  // TODO: consider making goal and feedback float by default
  osdk::AltitudeControlGoal goal_; 
  osdk::AltitudeControlResult result_;
  osdk::AltitudeControlFeedback feedback_;

  // store data needed for feedback control
  // TODO: consider setting an initial value for current pos
  sensor_msgs::NavSatFix curr_pos_;

  // gain for the proportional controller
  const int Kp { 0.1 };

  // velocity saturation [m/s]
  const int vel_saturation { 10 };
  
  // clients to control drone's motion
  ros::ServiceClient set_joystick_mode_client_;
  ros::ServiceClient joystick_action_client_;
  // client to obtain control via OSDK
  ros::ServiceClient obtain_ctrl_authority_client_ ;
  ros::Subscriber gps_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "altitude_control_action_server");

  AltitudeControlActionServer server("control_altitude");
  ros::spin();
  return 0;
}
