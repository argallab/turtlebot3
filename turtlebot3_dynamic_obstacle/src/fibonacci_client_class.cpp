#include "turtlebot3_action.h"


FibonacciActionClient::FibonacciActionClient(): ac_(nh_, "fibonacci", true)
{
  //constructor
  initializeSubscribers();

  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  doStuff(10);

}

void FibonacciActionClient::doStuff(int order)
{

  goal_.order = order;

  // Need boost::bind to pass in the 'this' pointer
  ac_.sendGoal(goal_,
              boost::bind(&FibonacciActionClient::doneCb, this, _1, _2),
              boost::bind(&FibonacciActionClient::activeCb, this),
              boost::bind(&FibonacciActionClient::feedbackCb, this, _1));
}

void FibonacciActionClient::initializeSubscribers()
{
  input_sub_ = nh_.subscribe<std_msgs::String>("/traj_selection", 1, &FibonacciActionClient::get_key_callback, this);
}


void FibonacciActionClient::get_key_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (msg->data=="s")
  {
    ROS_INFO("Preempting");
    ac_.cancelAllGoals();
  }
}

void FibonacciActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::FibonacciResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

void FibonacciActionClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

void FibonacciActionClient::feedbackCb(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_fibonacci_callback");

  FibonacciActionClient ac_object;

  ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}
