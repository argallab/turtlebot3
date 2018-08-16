// Author: Mahdieh Nejati

#include "turtlebot3_action.h"


Turtlebot3ActionClient::Turtlebot3ActionClient(): ac_(nh_, "turtlebot3", true)
{
  //constructor
  initializeSubscribers();

  // ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  // ROS_INFO("Action server started, sending goal.");

  // doStuff(10);

}

void Turtlebot3ActionClient::sendTrajGoal(int mode, double area, int count)
{

  goal_.goal.x = mode;
  goal_.goal.y = area;
  goal_.goal.z = count;

  // Need boost::bind to pass in the 'this' pointer
  ac_.sendGoal(goal_,
              boost::bind(&Turtlebot3ActionClient::doneCb, this, _1, _2),
              boost::bind(&Turtlebot3ActionClient::activeCb, this),
              boost::bind(&Turtlebot3ActionClient::feedbackCb, this, _1));
}

void Turtlebot3ActionClient::initializeSubscribers()
{
  input_sub_ = nh_.subscribe<std_msgs::String>("/traj_selection", 1, &Turtlebot3ActionClient::getKeyCb, this);
}

void Turtlebot3ActionClient::getKeyCb(const std_msgs::String::ConstPtr& msg)
{
  // MODES:
  // s: Preempted
  // straight: 1, circular: 2, triangle: 3, square: 4,
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (msg->data=="s")
  {
    ROS_INFO("Preempting");
    ac_.cancelAllGoals();
  }
  else if (msg->data=="1")
  {
    sendTrajGoal(1, 0, 5);
  }
  else if (msg->data=="2")
  {
    sendTrajGoal(2, 0, 1);
  }
  else if (msg->data=="3")
  {
    sendTrajGoal(3, 0, 1);
  }
  else
  {
    ROS_INFO("No implementation for selected mode. ");
  }

}

void Turtlebot3ActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const turtlebot3_dynamic_obstacle::Turtlebot3ResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Final Pose X: %f, Y: %f, Z:%f", result->result.x,  result->result.y,  result->result.theta);
  ros::shutdown();
}

void Turtlebot3ActionClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

void Turtlebot3ActionClient::feedbackCb(const turtlebot3_dynamic_obstacle::Turtlebot3FeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of velocities x: %f, y: %f, z: %f", feedback->state.x, feedback->state.y, feedback->state.z);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_client");

  Turtlebot3ActionClient ac_object;

  // ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spin(); //normally, can simply do: ros::spin();
    }

    return 0;
}
