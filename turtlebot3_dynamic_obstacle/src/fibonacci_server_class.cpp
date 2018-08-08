#include "turtlebot3_action_server.h"

// Implementation of the constructor
FibonacciActionServer::FibonacciActionServer():
  as_(nh_, "fibonacci", boost::bind(&FibonacciActionServer::executeCB, this, _1),false)
{
  //constructor
  ROS_INFO("in constructor of FibonacciActionServer...");
    // do any other desired initializations here...specific to your implementation
  as_.start(); //start the server running
  ROS_INFO("Started");

}

void FibonacciActionServer::executeCB(const actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction>::GoalConstPtr& goal)
{
  // helper variables
  ros::Rate r(1);
  bool success = true;

  ROS_INFO("In execution");

  // push_back the seeds for the fibonacci sequence
  feedback_.sequence.clear();
  feedback_.sequence.push_back(0);
  feedback_.sequence.push_back(1);

  // publish info to the console for the user
  ROS_INFO("Executing, creating fibonacci sequence of order %i with seeds %i, %i", goal->order, feedback_.sequence[0], feedback_.sequence[1]);

  // start executing the action
  for(int i=1; i<=goal->order; i++)
  {
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Preempted");
      // if(as_.isNewGoalAvailable())
      // {
      //   actionlib_tutorials::FibonacciGoal new_goal = as_.acceptNewGoal();
      // }
      // // set the action state to preempted
      as_.setPreempted(result_, "premepting due to request");
      as_.setAborted(result_, "aborting");
      success = false;
      break;
    }
    feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
    // publish the feedback
    as_.publishFeedback(feedback_);
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep();
  }

  if(success)
  {
    result_.sequence = feedback_.sequence;
    ROS_INFO("Succeeded");
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
  else
  {
    ROS_INFO("Failed");
    as_.setAborted();
  }

  ROS_INFO("END OF SCRIPT");
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci_action_server_node");

  FibonacciActionServer as_object;
  while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
