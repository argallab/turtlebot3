#ifndef TURTLEBOT3_ACTION_H_
#define TURTLEBOT3_ACTION_H_

// libraries
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot3_dynamic_obstacle/Turtlebot3Action.h>

class Turtlebot3ActionClient
{
public:
  Turtlebot3ActionClient();

  ~Turtlebot3ActionClient(void)
  {
  }

  typedef actionlib::SimpleActionClient<turtlebot3_dynamic_obstacle::Turtlebot3Action> Client;

private:
  ros::NodeHandle nh_;

  // SUBSCRIBERS
  ros::Subscriber input_sub_;

  // ACTION CLIENT
  Client ac_;

  turtlebot3_dynamic_obstacle::Turtlebot3Goal goal_;
  turtlebot3_dynamic_obstacle::Turtlebot3Result result_;
  turtlebot3_dynamic_obstacle::Turtlebot3Feedback feedback_;

  // MEMBER METHODS
  void initializeSubscribers();
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const turtlebot3_dynamic_obstacle::Turtlebot3ResultConstPtr& result);
  void activeCb();
  void feedbackCb(const turtlebot3_dynamic_obstacle::Turtlebot3FeedbackConstPtr& feedback);
  void getKeyCb(const std_msgs::String::ConstPtr& msg);
  void sendTrajGoal(int mode, double area, int count);
};

#endif
