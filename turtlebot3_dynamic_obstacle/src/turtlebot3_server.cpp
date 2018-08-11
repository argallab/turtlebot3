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


void Turtlebot3ActionServer::getOdom()
{
  try{
    listener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform_);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  trans_ = transform_.getOrigin();
  rot_ = transform_.getRotation();

  position_.x = trans_.x();
  position_.y = trans_.y();
  position_.z = trans_.z();

  tf::Matrix3x3 m(rot_);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  rotation_.x = roll;
  rotation_.y = pitch;
  rotation_.z = yaw;
  // ROS_INFO("x: %f, y: %f, z:%f, theta: %f", position_.x, position_.y, position_.z, rotation_.z);
}

double Turtlebot3ActionServer::getRadian(double angle)
{
  return angle*M_PI/180;
}

double Turtlebot3ActionServer::wrapAngle(double angle)
{
  angle = fmod(angle+M_PI, 2*M_PI);
  if (angle<0)
  {
    angle += 2*M_PI;
  }
  return angle - M_PI;
}

void Turtlebot3ActionServer::move(double goal_x, double goal_y, double angle)
{
  ros::Rate r(10);

  int last_rotation = 0;
  int linear_speed = 1;
  int angular_speed = 1;

  double x_start, y_start, path_angle;

  double goal_z = getRadian(angle);
  getOdom();
  double goal_distance = sqrt(pow(goal_x - position_.x, 2) + pow(goal_y - position_.y, 2));
  double distance = goal_distance;

  while (distance > 0.05)
  {
    // check that preempt has not been requested by the client
    if (checkPreempt())
    {
      break;
    }
    getOdom();
    x_start = position_.x;
    y_start = position_.y;
    path_angle = atan2(goal_y - y_start, goal_x - x_start);
    path_angle = wrapAngle(path_angle);
    ROS_INFO("x %f, y %f, angle %f", x_start, y_start, path_angle);

    if (path_angle < -1*M_PI/4 || path_angle > M_PI/4)
    {
      if (goal_y < 0 && y_start < goal_y)
      {
        path_angle = -2 * M_PI + path_angle;
      }
      else if (goal_y >= 0 && y_start > goal_y)
      {
        path_angle = 2 * M_PI + path_angle;
      }
    }
    if (last_rotation > M_PI - 0.1 && rotation_.z <= 0)
    {
      rotation_.z = 2 * M_PI + rotation_.z;
    }
    else if (last_rotation < -M_PI + 0.1 && rotation_.z >0)
    {
      rotation_.z = -2 * M_PI + rotation_.z;
    }
    distance = sqrt(pow(goal_x - x_start, 2) + pow(goal_y - y_start, 2));
    twist_.linear.x = std::min(linear_speed*distance, 0.1);

    twist_.angular.z = angular_speed * path_angle-rotation_.z;
    if (twist_.angular.z > 0)
    {
      twist_.angular.z = std::min(twist_.angular.z, 1.5);
    }
    else
    {
      twist_.angular.z = std::max(twist_.angular.z, -1.5);
    }
    last_rotation = rotation_.z;
    cmd_pub_.publish(twist_);
    ROS_INFO("linear vel %f, angular vel %f", twist_.linear.x, twist_.angular.z);
    r.sleep();
  }

  getOdom();

  while( fabs(rotation_.z - goal_z) > 0.05)
  {
      // check that preempt has not been requested by the client
      if (checkPreempt())
      {
        break;
      }

      getOdom();
      if (goal_z >=0)
      {
        if (rotation_.z <= goal_z && rotation_.z >= goal_z - M_PI)
        {
          twist_.linear.z = 0.00;
          twist_.angular.z = 0.5;
        }
        else
        {
          twist_.linear.z = 0.00;
          twist_.angular.z = -0.5;
        }
      }
      else
      {
        if (rotation_.z <= goal_z + M_PI && rotation_.z > goal_z )
        {
          twist_.linear.z = 0.00;
          twist_.angular.z = -0.5;
        }
        else
        {
          twist_.linear.z = 0.00;
          twist_.angular.z = 0.5;
        }
      }
      cmd_pub_.publish(twist_);
      // ROS_INFO("linear vel %f, angular vel %f", twist_.linear.x, twist_.angular.z);
      // ROS_INFO("angular distance %f", fabs(rotation_.z - goal_z));
      r.sleep();
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
  double area = goal->goal.y;
  int count = goal->goal.z;

  // start_position_ = position_;
  // init_state_ = true;

  // helper variables
  ros::Rate r(10);
  success_ = true;

  // start executing the action
  for(int i=1; i<=count; i++)
  {
    // check that preempt has not been requested by the client
    if (checkPreempt())
    {
      break;
    }

    // move(0, 0, 180); //doesn't work
    //   move(0, 0,-90); //dosen't work
    //   move(0,0,30); // doens't work
    if (mode==1)
    {
      move(0.5, 0, 0);
      r.sleep();
      move(0, 0.5, 180);
      // 2, 1, 30
    }
    else if (mode ==2)
    {//togeter doesn't work
      move(0.5, 0, 0);
      r.sleep();
      move(0.5, 0.5, 90);
    }
    else if (mode ==3)
    {
      move(0, 1, 90); // works
      r.sleep();
      move(0.7, 0.7,0);
    }
    else if (mode ==4)
    {// together doesn't work
      move(2, 1,0); //works
      r.sleep();
      move(2, 1.2,90); //works
    }
  }

  if(success_)
  {
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
