// Author: Mahdieh Nejati

#include "turtlebot3_action_server.h"

// Implementation of the constructor
Turtlebot3ActionServer::Turtlebot3ActionServer():
  as_(nh_, "turtlebot3", boost::bind(&Turtlebot3ActionServer::executeCB, this, _1),false)
{
  //constructor
  ROS_INFO("in constructor of Turtlebot3ActionServer...");

  initializePublishers();

  // do any other desired initializations here...specific to your implementation
  as_.start(); //start the server running
  ROS_INFO("Turtlebot3 server sstarted");

}

void Turtlebot3ActionServer::initializePublishers()
{
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
}

void Turtlebot3ActionServer::move_forward(double lin_vel, double duration)
{
  ros::Rate r(10);
  double time_start =ros::Time::now().toSec();
  double time_now =ros::Time::now().toSec();

  while ((time_now-time_start)<=duration)
  {
    ROS_INFO("going forward, %f",time_now-time_start);
    // check that preempt has not been requested by the client
    if (checkPreempt())
    {
      break;
    }
    twist_.linear.x = lin_vel;
    twist_.angular.z = 0.0;
    cmd_pub_.publish(twist_);
    r.sleep();
    time_now =ros::Time::now().toSec();
  }
  clearVelocities();
}

void Turtlebot3ActionServer::turn(double ang_vel, double duration)
{
  ros::Rate r(10);
  double time_start =ros::Time::now().toSec();
  double time_now =ros::Time::now().toSec();

  while ((time_now-time_start)<=duration)
  {
    ROS_INFO("Turning, %f",time_now-time_start);
    // check that preempt has not been requested by the client
    if (checkPreempt())
    {
      break;
    }
    twist_.linear.x = 0.0;
    twist_.angular.z = ang_vel;
    cmd_pub_.publish(twist_);
    r.sleep();
    time_now =ros::Time::now().toSec();
  }
  clearVelocities();
}

void Turtlebot3ActionServer::clearVelocities()
{
  twist_.linear.x = 0.0;
  twist_.angular.z = 0.0;
  cmd_pub_.publish(twist_);
}


bool Turtlebot3ActionServer::checkPreempt()
{
  if (as_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("Preempted");
    as_.setPreempted(result_, "premepting due to request");
    clearVelocities();
    success_ = false;
    return 1;
  }
  else return 0;
}

void Turtlebot3ActionServer::executeCB(const actionlib::SimpleActionServer<turtlebot3_dynamic_obstacle::Turtlebot3Action>::GoalConstPtr& goal)
{
  // Get goal
  int mode = goal->goal.x;
  // double area = goal->goal.y;
  int count = goal->goal.z;

  // helper variables
  ros::Rate r(10);
  success_ = true;

  // start executing the action

  for(int i=1; i<=count; i++){
    double time_start =ros::Time::now().toSec();
    double time_now =ros::Time::now().toSec();
    while ((time_now-time_start)<=60)
    {
      // check that preempt has not been requested by the client
      if (checkPreempt())
      {
        break;
      }
      if (mode==1)
      {
        move_forward(0.1, 10);
        r.sleep();
        turn(0.52, 6);
        r.sleep();
        }
      else if (mode == 2)
      {
        turn(0.1, 5);
        r.sleep();
      }
      else if (mode == 3)
      {
        move_forward(0.1, 5);
        r.sleep();
      }
      time_now =ros::Time::now().toSec();
    }
  }

  if(success_)
  {
    // pose = getOdom();
    result_.result.x = position_.x;
    result_.result.y = position_.y;
    result_.result.theta = rotation_.z;
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
  else
  {
    // set the action state to failed
    as_.setAborted();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_action_server_node");

  Turtlebot3ActionServer as_object;
  while (ros::ok()) {
        ros::spin();
    }
  return 0;
}
