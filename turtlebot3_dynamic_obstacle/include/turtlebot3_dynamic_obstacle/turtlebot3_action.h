#ifndef TURTLEBOT3_ACTION_H_
#define TURTLEBOT3_ACTION_H_

// libraries
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciActionClient
{
public:
  FibonacciActionClient();

  ~FibonacciActionClient(void)
  {
  }

  typedef actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> Client;

private:
  ros::NodeHandle nh_;

  // SUBSCRIBERS
  ros::Subscriber input_sub_;

  // ACTION CLIENT
  Client ac_;

  actionlib_tutorials::FibonacciGoal goal_;
  actionlib_tutorials::FibonacciResult result_;
  actionlib_tutorials::FibonacciFeedback feedback_;

  // MEMBER METHODS
  void initializeSubscribers();
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const actionlib_tutorials::FibonacciResultConstPtr& result);
  void activeCb();
  void feedbackCb(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback);
  void get_key_callback(const std_msgs::String::ConstPtr& msg);
  void doStuff(int order);
};

#endif
