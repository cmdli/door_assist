//Written by: Chris de la Iglesia

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

ros::ServiceClient plan_check;

#define PATH_VARIATION_TOLERANCE 1.2
#define ROBOT_LOC_BUFFER 1.2
#define ROBOT_WIDTH_RAW 0.5
#define ROBOT_WIDTH ROBOT_WIDTH_RAW*ROBOT_LOC_BUFFER

//Takes in a door pose and width and returns
//a set of global points next to the door
//that the robot can go to get help
void get_door_positions(geometry_msgs::PoseStamped door,
			double door_width,
			std::vector<geometry_msgs::PoseStamped>& global_points)
{
  //Use the door's position and orientation as
  //a coordinate transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(door.pose.position.x, door.pose.position.y,
				  door.pose.position.z));
  transform.setRotation(tf::Quaternion(door.pose.orientation.x,
				       door.pose.orientation.y,
				       door.pose.orientation.z,
				       door.pose.orientation.w));
  
  //Positions relative to the door
  tf::Vector3 p1(ROBOT_WIDTH/2.0, 
		 0.0 - door_width/2.0 - ROBOT_WIDTH/2.0, 
		 0.0);
  tf::Vector3 p2(ROBOT_WIDTH/2.0, 
		 door_width/2.0 + ROBOT_WIDTH/2.0, 
		 0.0);
  tf::Vector3 p3(door_width + ROBOT_WIDTH/2.0, 
		 0.0 - door_width/2.0 + ROBOT_WIDTH/2.0, 
		 0.0);
  tf::Vector3 p4(door_width + ROBOT_WIDTH/2.0, 
		 door_width/2.0 - ROBOT_WIDTH/2.0,
		 0.0);

  //Transform the relative positions into global positions
  std::vector<tf::Vector3> points;
  points.push_back(transform(p1));
  points.push_back(transform(p2));
  points.push_back(transform(p3));
  points.push_back(transform(p4));

  //Copy the global positions into the return container
  //and add in the orientation facing the door
  global_points.resize(points.size());
  tf::Quaternion turn_around;
  turn_around.setEuler(0.0, 0.0, 0.0); //TODO: find what values turn the robot 180deg
  tf::Quaternion face_door = transform.getRotation() + turn_around;
  /*ROS_INFO("Door Rotation-- X: %f Y: %f Z: %f W: %f",
	   face_door.getX(),
	   face_door.getY(),
	   face_door.getZ(),
	   face_door.getW());*/
  for(int i = 0; i < points.size(); i++) {
    global_points[i].header.frame_id = "/map";
    global_points[i].header.stamp = ros::Time::now();
    global_points[i].pose.position.x = points[i].getX();
    global_points[i].pose.position.y = points[i].getY();
    global_points[i].pose.position.z = points[i].getZ();

    global_points[i].pose.orientation.x = face_door.getX();
    global_points[i].pose.orientation.y = face_door.getY();
    global_points[i].pose.orientation.z = face_door.getZ();
    global_points[i].pose.orientation.w = face_door.getW();
  }
}

//Gets the current position of the robot
geometry_msgs::PoseStamped get_position()
{
  //Looks up the position using TF
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/base_footprint", "/map",
			      now, ros::Duration(3.0));
    listener.lookupTransform("/base_footprint", "/map",  
			     now, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  } 

  //Copies the info into a PoseStamped
  geometry_msgs::PoseStamped current_pos;
  current_pos.header.frame_id = "/map";
  current_pos.header.stamp = ros::Time::now();
  current_pos.pose.position.x = transform.getOrigin().x();
  current_pos.pose.position.y = transform.getOrigin().y();
  current_pos.pose.position.z = transform.getOrigin().z();
  current_pos.pose.orientation.x = transform.getRotation().getX();
  current_pos.pose.orientation.y = transform.getRotation().getY();
  current_pos.pose.orientation.z = transform.getRotation().getZ();
  current_pos.pose.orientation.w = transform.getRotation().getW();

  return current_pos;
}

//Returns the distance between to poses
double dist(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
  double p1_x = p1.pose.position.x;
  double p1_y = p1.pose.position.y;
  double p2_x = p2.pose.position.x;
  double p2_y = p2.pose.position.y;
  return sqrt((p1_x - p2_x)*(p1_x - p2_x) +
	      (p1_y - p2_y)*(p1_y - p2_y));
 }

//Checks if there is a valid path between two points
//(no major obstacles between them)
bool valid_path(geometry_msgs::PoseStamped start,
		geometry_msgs::PoseStamped end)
{
  //Get a path from move_base
  nav_msgs::GetPlan planMsg;
  planMsg.request.start = start;
  planMsg.request.goal = end;
  planMsg.request.tolerance = 0.1;
  plan_check.call(planMsg);
  nav_msgs::Path plan = planMsg.response.plan;

  //No path between the two points
  if(plan.poses.empty())
    return false;

  //Get the length of the path
  double path_length = 0.0;
  for(int i = 0; i < plan.poses.size() - 1; i++) {
    path_length += dist(plan.poses[i], plan.poses[i+1]);
  }

  //If the path length is much greater than a straight path,
  //then there is probably an obstacle and the path is not valid.
  return path_length < dist(start,end)*PATH_VARIATION_TOLERANCE;
}

int main(int argc, char* argv[])
{
  //Init ROS
  ros::init(argc, argv, "door_assist");


  //Advertise message
  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  ROS_INFO("Waiting on connection to move_base...");
  ros::Rate poll_rate(100);
  while(move_pub.getNumSubscribers() == 0)
    poll_rate.sleep();

  //Connect to move_base for planning a path
  plan_check = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

  //Create message
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "/base_footprint";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  goal.pose.orientation.w = 1.0;

  //Publish message
  /*  move_pub.publish(goal);

  ROS_INFO("Goal Published");*/

  /*geometry_msgs::PoseStamped start;
  start.header.frame_id = ""; //Use the robot's current position
  start.header.stamp = ros::Time::now();
  start.pose.position.x = 0.0;
  start.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped goalGlobal;
  tf::TransformListener listener;
  try {
    ros::Time now = ros::Time(0);
    goal.header.stamp = now;
    listener.waitForTransform("/map", "/base_footprint",
			      now, ros::Duration(3.0));
    listener.transformPose("/map", goal, goalGlobal);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  

  if(valid_path(start, goalGlobal))
    ROS_INFO("Good! The path ahead is valid.");
  else
    ROS_INFO("Bad! The path ahead is not valid.");

  ROS_INFO("Done.");*/

  //Test code for door position
  geometry_msgs::PoseStamped door;
  tf::Quaternion door_rot;
  door_rot.setEuler(0.0,0.0,1.0);
  door.pose.orientation.x = door_rot.getX();
  door.pose.orientation.y = door_rot.getY();
  door.pose.orientation.z = door_rot.getZ();
  door.pose.orientation.w = door_rot.getW();
  
  std::vector<geometry_msgs::PoseStamped> points;
  get_door_positions(door, 2.0, points);

  ROS_INFO("Number of door positions: %lu", points.size());
  for(int i = 0; i < points.size(); i++) {
    ROS_INFO("Point %d-- X: %f Y: %f Z: %f", i, 
	     points[i].pose.position.x, 
	     points[i].pose.position.y,
	     points[i].pose.position.z);
  }

  move_pub.publish(points[0]);
  ROS_INFO("Goal Published.");

  //Spin
  ros::spinOnce();

  /*  //Wait a few seconds for the message to publish
  ros::Rate sleep_time(0.25);
  sleep_time.sleep();*/

}
