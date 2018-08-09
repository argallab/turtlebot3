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
  rot_ = odom->pose.pose.orientation;
  double roll, pitch, yaw;

  // roll (x-axis rotation)
	double sinr = +2.0 * (rot_.w * rot_.x + rot_.y * rot_.z);
	double cosr = +1.0 - 2.0 * (rot_.x * rot_.x + rot_.y * rot_.y);
	roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
	double sinp = +2.0 * (rot_.w * rot_.y - rot_.z * rot_.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (rot_.w * rot_.z + rot_.x * rot_.y);
	double cosy = +1.0 - 2.0 * (rot_.y * rot_.y + rot_.z * rot_.z);
	yaw = atan2(siny, cosy);

  rpy_.x = roll;
  rpy_.y = pitch;
  rpy_.z = yaw;
  // try{
  //   listener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform_);
  // }
  // catch (tf::TransformException &ex) {
  //   ROS_ERROR("%s", ex.what());
  //   ros::Duration(1.0).sleep();
  // }
  // trans_ = transform_.translation;
  // rot_ = transform_.rotation;
  // double roll, pitch, yaw;
  // tf::Matrix3x3(rot_).getRPY(roll, pitch, yaw);
  // rpy_.x = roll;
  // rpy_.y = pitch;
  // rpy_.z = yaw;
  // ROS_INFO("x: %f, y: %f, z:%f, theta: %f", trans_.x, trans_.y, trans_.z, rpy_.z);
}

double Turtlebot3ActionServer::getRadian(double angle)
{
  return angle*180/M_PI;
}

void Turtlebot3ActionServer::move(double distance, double angle)
{
  int last_rotation = 0;
  int linear_speed = 1;
  int angular_speed = 1;

  double x_start, y_start, path_angle;

  double radian = getRadian(angle);

  while (distance > 0.05)
  {
    x_start = position_.x;
    y_start = position_.y;
    path_angle =
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
