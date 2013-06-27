
#include <ros/ros.h>
#include "tf.h"
#include <geometry_msgs.h>

#define PI 3.14

geometry_msgs::Pose location;

void callback(geometry_msgs::PoseWithCovarianceStamped pose)
{
  location = pose.pose.pose; // -.-
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "door_assist");
  if(argc < 7) {
    ROS_ERROR("Correct door_assist usage: rosrun door_assist position_x position_y position_z orientation_x orientation_y orientation_z orientation_w");
  }

  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal");
  ros::Subscriber get_location = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, callback);

  location.position.x = -1.0;

  while(ros::ok() && location.position.x == -1.0) {
    ros::spinOnce();
  }

  geometry_msgs::Point pos;
  geometry_msgs::Quaternion orient;
  pos = location.position;
  orient = location.orient;

  /*  pos.x = atof(argv[0]);
  pos.y = atof(argv[1]);
  pos.z = atof(argv[2]);
  orient.x = atof(argv[3]);
  orient.y = atof(argv[4]);
  orient.z = atof(argv[5]);
  orient.w = atof(argv[6]);*/

  double angle = tf::getYaw(orient);
  double xd = 0.5*cos(angle);
  double zd = 0.5*sin(angle);
  
  Quaternion q;
  q.setEuler(PI/2,0,0);
  orient += q;

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = pos.x + xd;
  goal.pose.position.y = pos.y;
  goal.pose.position.z = pos.x + zd;
  goal.pose.orientation = orient;

  move_pub.publish(goal);

  ros::spinOnce();
    
}
