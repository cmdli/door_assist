
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "door_assist");

  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform("/base_footprint", "/map",
			     ros::Time(0), transform);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return 1;
  }

  geometry_msgs::Point pos;
  tf::Quaternion orient;
  pos.x = transform.getOrigin().x();
  pos.y = transform.getOrigin().y();
  pos.z = transform.getOrigin().z();
  orient = transform.getRotation();

  double angle = tf::getYaw(orient);
  double xd = 0.5*cos(angle);
  double zd = 0.5*sin(angle);
  
  tf::Quaternion q;
  q.setEuler(PI/2,0,0);
  orient += q;

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = pos.x + xd;
  goal.pose.position.y = pos.y;
  goal.pose.position.z = pos.x + zd;
  goal.pose.orientation.x = orient.getX();
  goal.pose.orientation.y = orient.getY();
  goal.pose.orientation.z = orient.getZ();
  goal.pose.orientation.w = orient.getW();

  move_pub.publish(goal);

  ROS_INFO("Goal Published");

  ros::spinOnce();
    
}
