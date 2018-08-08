// Author: Mahdieh Nejati

#include "turtlebot3_action_server.h"

// Implementation of the constructor
Turtlebot3ActionServer::Turtlebot3ActionServer():
  as_(nh_, "turtlebot3", boost::bind(&Turtlebot3ActionServer::executeCB, this, _1),false)
{
  //constructor
  ROS_INFO("in constructor of Turtlebot3ActionServer...");

  initializeSubscribers();
  initializePublishers();

    // do any other desired initializations here...specific to your implementation
  as_.start(); //start the server running
  ROS_INFO("Turtlebot3 server sstarted");

}

void Turtlebot3ActionServer::initializeSubscribers()
{
  state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &Turtlebot3ActionServer::getState, this);
  odom_sub_  = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &Turtlebot3ActionServer::getOdom, this);
}

void Turtlebot3ActionServer::initializePublishers()
{
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void Turtlebot3ActionServer::getOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
  position_ = odom->pose.pose.position;
}

void Turtlebot3ActionServer::getState(const sensor_msgs::JointState::ConstPtr& state)
{
  float TICK2RAD = 0.001533981;
  double last_pos, diff_pos, cur_pos, encoder; // TO DO:: CHECK ENCODER IF NEEDS TO BE INT
  last_pos = 0.0;
  diff_pos = 0.0;
  encoder = 0.0;

  // Origin is where turtlebot is at start of each request started
  cur_pos = state->position[0];
  diff_pos = cur_pos - last_pos;
  encoder = encoder + (diff_pos/TICK2RAD);
  right_encoder_ = encoder;
}

void Turtlebot3ActionServer::turn(float angle)
{
  ros::Rate r(1);

  if (init_state_)
  {
    init_right_encoder_ = right_encoder_;
    init_state_ = false;
  }

  float diff_encoder = ((angle*3.14159/180) * 0.080) / (0.207 / 4096);
  while (fabs(init_right_encoder_ - right_encoder_) < fabs(diff_encoder))
  {

    if (diff_encoder >= 0)
    {
      twist_.angular.z = -0.5;
      ROS_INFO("SERVER: Turning -0.5");
    }
    else
    {
      twist_.angular.z = 0.5;
      ROS_INFO("SERVER: Turning 0.5");
    }
    cmd_pub_.publish(twist_);
    r.sleep();
  }

  init_state_ = true;
  clearVelocities();

}

bool Turtlebot3ActionServer::getChangeInPosition(double length, int mode)
{
  bool desired_position;
  if (mode == 0)
  {
    desired_position = fabs(position_.x - start_position_.x) < fabs(length);
    ROS_INFO("SERVER: FIRST SIDE %f, length %f, bool %d",position_.x - start_position_.x, length, desired_position);
  }
  else if (mode == 1)
  {
    desired_position = fabs(position_.y - start_position_.y) < fabs(length);
    ROS_INFO("SERVER: SECOND SIDE %f, length %f, bool %d",position_.x - start_position_.x, length, desired_position);
  }
  else if (mode == 2)
  {
    desired_position = fabs(position_.x - start_position_.x) > fabs(length);
    ROS_INFO("SERVER: THIRD SIDE %f, length %f, bool %d",position_.x - start_position_.x, length, desired_position);
  }
  else if (mode == 3)
  {
    desired_position = fabs(position_.y - start_position_.y) > fabs(length);
    ROS_INFO("SERVER: FOURTH SIDE %f, length %f, bool %d",position_.x - start_position_.x, length, desired_position);
  }
  return desired_position;
}

void Turtlebot3ActionServer::goForward(double length, int count)
{
  ROS_INFO("SERVER: Trying to send %f", length);
  double x_vel, z_vel;
  ros::Rate r(1);

  if (length < 0)
  {
    x_vel = -0.1;
  }
  else
  {
    x_vel = 0.1;
  }

  while(getChangeInPosition(length, count))
  {
    if (checkPreempt())
    {
      break;
    }
    twist_.linear.x = x_vel;
    ROS_INFO("x_vel %f", x_vel);
    twist_.angular.z = 0.0;
    cmd_pub_.publish(twist_);
    feedback_.state.x = twist_.linear.x;
    feedback_.state.z = twist_.angular.z;
    as_.publishFeedback(feedback_);
    ROS_INFO("SERVER: Executing, current state: %f, %f", feedback_.state.x, feedback_.state.z);
    r.sleep();
  }
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
    // as_.setAborted(result_, "aborting");
    success_ = false;
    return 1;
  }
  else return 0;
}

void Turtlebot3ActionServer::executeCB(const actionlib::SimpleActionServer<turtlebot3_dynamic_obstacle::Turtlebot3Action>::GoalConstPtr& goal)
{
  // Get goal
  int mode = goal->goal.x;
  double area = goal->goal.y;
  int count = goal->goal.z;

  start_position_ = position_;
  init_state_ = true;

  // ROS_INFO("SERVER: In execution: Mode: %i, Area %f, Count %i", mode, area, count);

  // helper variables
  ros::Rate r(1);
  success_ = true;

  bool circle_mode = true;
  bool half_patrol = false;
  bool count_flag = false;
  int circle_count = 0;

  // start executing the action
  for(int i=1; i<=count; i++)
  {
    // check that preempt has not been requested by the client
    if (checkPreempt())
    {
      break;
    }

    if (mode == 1) // straight traj
    {
      goForward(area, 0);
      clearVelocities();
    }

    else if (mode == 2) // circular traj
    {
      ROS_INFO("SERVER: Circle Mode");
      while(circle_mode)
      {
        if (checkPreempt())
        {
          circle_mode = false;
          break;
        }

        if (position_.x < -area/2)
        {
          half_patrol = true;
          count_flag = true;
        }
        else
        {
          twist_.linear.x = area/2;
          twist_.angular.z = 0.5;
        }
        if (half_patrol && position_.x>0)
        {
          if(count_flag)
          {
            circle_count += 1;
            count_flag = false;
          }
          half_patrol = false;
          if (circle_count == count)
          {
            circle_mode = false;
            clearVelocities();
          }
        }
        cmd_pub_.publish(twist_);
      }
    }

    else if (mode == 4) // square traj
    {
      float sides[4] = {0};
      sides[0] = area;
      sides[1] = area;
      for (int j=0; j<=3; j++)
      {
        ROS_INFO("Going forward for side length %f, side %i", sides[j], j);
        goForward(sides[j], j);
        r.sleep();
        turn(-90);
        r.sleep();
      }
    }
  }

  if(success_)
  {
    result_.result.x = position_.x;
    result_.result.y = position_.y;
    result_.result.x = position_.z;
    // ROS_INFO("Succeeded");
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
  else
  {
    // ROS_INFO("Failed");
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
