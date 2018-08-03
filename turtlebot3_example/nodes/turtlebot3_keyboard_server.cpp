
// Mahdieh Nejati

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "turtlebot3_msgs/SensorState.h"
#include <actionlib/server/simple_action_server.h>
#include <turtlebot3_example/Turtlebot3Action.h>

class Turtlebot3Action
{

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot3_example::Turtlebot3Action> as_;
  std::string action_name_;
  int mode_;
  turtlebot3_example::Turtlebot3Feedback feedback_;
  turtlebot3_example::Turtlebot3Result result_;
  ros::Subscriber stats_sub_, odom_sub_ ;

public:

  Turtlebot3Action(std::string name) :
    as_(nh_, name, boost::bind(&Turtlebot3Action::executeCB, this, _1), false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&Turtlebot3Action::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Turtlebot3Action::preemptCB, this));

    //subscribe to the data topic of interest
    stats_sub_ = nh_.subscribe("/joint_states", 1, &Turtlebot3Action::get_state, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &Turtlebot3Action::get_odom, this);
    as_.start();
  }

  ~Turtlebot3Action(void)
  {
  }

  void goalCB()
  {
    // reset helper variables
    ROS_INFO("IN goalCB");
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void get_state(const sensor_msgs::JointState msg)
  {
  }

  void get_odom(const nav_msgs::Odometry msg)
  {
  }

  void executeCB(const turtlebot3_example::Turtlebot3GoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    ROS_INFO("%s: Executing, %f", action_name_.c_str(), goal->goal.x);

    if(as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted in execute", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      //break;
    }

    if(success)
    {
      result_.result = "Okay";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_action_server");
  Turtlebot3Action turtlebot3_action_server(ros::this_node::getName());
  ros::spin();
  return 0;
}
