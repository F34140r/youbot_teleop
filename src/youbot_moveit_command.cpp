/*  Moves arm to position provided by user
 *
 *  Formatting for commandline:
 *  rosrun youbot_teleop youbot_moveit_command [action] [destination] [etc]
 *
 *  [action] can be either:
 *    --p : Pose
 *    --f : Forward Kinematics
 *    --i : Inverse Kinematics
 *  [destination] can be:
 *    if [action] was --p:
 *      - the string of a specific pose name stored in the SRDF
 *        ex)  rosrun youbot_teleop youbot_moveit_command --p arm_object
 *    if [action] was --f:
 *      - a series of five numbers (joint1 joint2 joint3 joint4 joint5) indicating joint angles to move to
 *        ex)  rosrun youbot_teleop youbot_moveit_command --f 3.0 0.5 -0.9 0.1 3.0
 *    if [action] was --i:
 *      - a string indicating the location of the reference frame.  Can be either base or odom
 *      - a series of six numbers representing the x, y, z, roll, pitch, and yaw of the target destination
 *        ex)  rosrun youbot_teleop youbot_moveit_command --i base_link -0.130 -0.013 0.401 -0.015 -1.284 0.060
 *        ex)  rosrun youbot_teleop youbot_moveit_command --i odom -0.130 -0.013 0.401 -0.015 -1.284 0.060
 */

#include <iostream>
#include <assert.h>
#include "ros/ros.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/PlanningScene.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/world.h>
#include "geometry_msgs/Pose.h"
#include "shape_msgs/SolidPrimitive.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "boost/thread.hpp"

#include <moveit/move_group_interface/move_group.h>

volatile bool trajectory_complete = false;
volatile char cancel = '0';
std::string goal_string = "0";
//planning_scene::PlanningScene* current_scene = NULL;


using namespace std;

char getch(void)
{
	char ch;
	struct termios oldt;
	struct termios newt;
	tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
	newt = oldt; /* copy old settings to new settings */
	newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediately */
	ch = getchar(); /* standard getchar call */
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
	return ch; /*return received char */
}

void getch_check()
{
  cancel = '0';
  while(cancel != 'c')
  {
    boost::this_thread::interruption_point();
    cancel = getch();
  }
}

geometry_msgs::Quaternion getQuatFromRPY( double roll, double pitch, double yaw)
{
  tf::Quaternion quad = tf::createQuaternionFromRPY(roll, pitch, yaw);

  geometry_msgs::Quaternion quad_msg;

  tf::quaternionTFToMsg(quad, quad_msg);

  return quad_msg;
}

geometry_msgs::Point getPosition( double xpo, double ypo, double zpo)
{
  geometry_msgs::Point pos;

  pos.x = xpo;
  pos.y = ypo;
  pos.z = zpo;

  return pos;
}

geometry_msgs::Pose getArmPointRelativeToOdom( double xpo, double ypo, double zpo, double ro, double pi, double ya, tf::TransformListener *tf_list, tf::TransformBroadcaster *broad )
{
  geometry_msgs::Pose end_pose;
  tf::StampedTransform base_relative_to_odom;

  try
  {
    (*tf_list).lookupTransform("odom","base_link",ros::Time(0), base_relative_to_odom);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  tf::Transform point_relative_to_odom;

  tf::Transform point_relative_to_base;
  tf::Vector3 point_tf_origin;
  tf::Quaternion point_tf_quaternion;

  point_tf_origin.setX(xpo);
  point_tf_origin.setY(ypo);
  point_tf_origin.setZ(zpo);

  point_tf_quaternion = tf::createQuaternionFromRPY(ro, pi, ya);

  point_relative_to_base.setOrigin(point_tf_origin);
  point_relative_to_base.setRotation(point_tf_quaternion);

  point_relative_to_odom.setIdentity();

  tf::Vector3 point_tf_odom_origin;

  point_tf_odom_origin.setX(point_relative_to_base.getOrigin().getX() + base_relative_to_odom.getOrigin().getX());
  point_tf_odom_origin.setY(point_relative_to_base.getOrigin().getY() + base_relative_to_odom.getOrigin().getY());
  point_tf_odom_origin.setZ(point_relative_to_base.getOrigin().getZ() + base_relative_to_odom.getOrigin().getZ());
  
  point_relative_to_odom.setOrigin(point_tf_odom_origin);
  point_relative_to_odom.setRotation( base_relative_to_odom.getRotation() * point_relative_to_base.getRotation());

  geometry_msgs::Quaternion quad_msg;

  tf::quaternionTFToMsg(point_relative_to_odom.getRotation(), quad_msg);

  end_pose.position.x = point_relative_to_odom.getOrigin().getX();
  end_pose.position.y = point_relative_to_odom.getOrigin().getY();
  end_pose.position.z = point_relative_to_odom.getOrigin().getZ();
  end_pose.orientation = quad_msg;

  return end_pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_moveit_command", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster br;

  spinner.start();


  double xposition = 0;
  double yposition = 0;
  double zposition = 0;

  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  int option = 0;
  string pose_name;
  int reference = 0;

  double filler[] = {0,0,0,0,0};
  vector<double> joints (filler, filler + sizeof(filler) / sizeof(double) );

  geometry_msgs::Pose new_pose;

  if( argc >= 3 )
  {
    if( strncmp(argv[1],"--p",3) == 0)
    {
      option = 1;
      if( argc < 3 )
      {
        cout << "Not enough arguments provided.";
        return 0;
      }
      pose_name = argv[2];
    }
    
    if( strncmp(argv[1],"--f",3) == 0 )
    {
      option = 2;
      if( argc < 7 )
      {
        cout << "Not enough arguments provided.";
        return 0;
      }
      joints.at(0) = atof(argv[2]);
      joints.at(1) = atof(argv[3]);
      joints.at(2) = atof(argv[4]);
      joints.at(3) = atof(argv[5]);
      joints.at(4) = atof(argv[6]);
    }

    if( strncmp(argv[1],"--i",3) == 0 )
    {
      option = 3;
      if( argc < 9 )
      {
        cout << "Not enough arguments provided.";
        return 0;
      }
      xposition = atof(argv[3]);
      yposition = atof(argv[4]);
      zposition = atof(argv[5]);
      roll = atof(argv[6]);
      pitch = atof(argv[7]);
      yaw = atof(argv[8]);
      if( strncmp(argv[2],"base",4) == 0 )
      {
        reference = 1;
      }
      if( strncmp(argv[2],"odom",4) == 0 )
      {
        reference = 2;
      }
      if( ( strncmp(argv[2],"odom",4) != 0 ) && ( strncmp(argv[2],"base",4) != 0 ) )
      {
        cout << "Invalid reference frame provided.  Must be either 'base' or 'odom'" << endl;
        return 0;
      }
    }

    if( ( strncmp(argv[1],"--p",3) != 0 ) && ( strncmp(argv[1],"--f",3) != 0 ) && ( strncmp(argv[1],"--i",3) != 0 ) )
    {
      cout << "Invalid option provided.  Must be either '--p', '--f', or '--i'" << endl;
      return 0;
    }
  }
  else
  {
    cout << "\n\nPlease provide arguments.  See below for a description of arguments.\n\n" << endl;
    cout << "rosrun youbot_teleop youbot_moveit_command [action] [destination] [etc]\n    [action] can be either:\n             --p : Pose\n             --f : Forward Kinematics\n             --i : Inverse Kinematics\n    [destination] can be:\n             if [action] was --p:\n                      - the string of a specific pose name stored in the SRDF\n                               ex)  rosrun youbot_teleop youbot_moveit_command --p arm_object\n             if [action] was --f:\n                      - a series of five numbers (joint1 joint2 joint3 joint4 joint5) indicating joint angles to move to\n                               ex)  rosrun youbot_teleop youbot_moveit_command --f 3.0 0.5 -0.9 0.1 3.0\n             if [action] was --i:\n                      - a string indicating the location of the reference frame.  Can be either base_link or odom\n                      - a series of six numbers representing the x, y, z, roll, pitch, and yaw of the target destination\n                               ex)  rosrun youbot_teleop youbot_moveit_command --i base_link -0.130 -0.013 0.401 -0.015 -1.284 0.060\n                               ex)  rosrun youbot_teleop youbot_moveit_command --i odom -0.130 -0.013 0.401 -0.015 -1.284 0.060\n\n";

    return 0;
  }

  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm");

  double tolerance = .1;

  group.setGoalTolerance(tolerance);

  group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot









  if( option == 1 )
  {
    ROS_INFO("Setting pose target...");
    if(!group.setNamedTarget(pose_name))
    {
      ROS_ERROR("\nInvalid pose name provided.  Please try again.");
      //temp.robot_model::RobotModel::~RobotModel();
      return 0;
    }
  }
  if( option == 2 )
  {
    ROS_INFO("Setting joint targets..");
    group.setJointValueTarget(joints);
  }
  if( option == 3 )
  {
    ROS_INFO("Setting ik target...");
    new_pose.position = getPosition(xposition, yposition, zposition);
    new_pose.orientation = getQuatFromRPY(roll, pitch, yaw);
    if( reference == 1 )
    {
      new_pose = getArmPointRelativeToOdom(xposition,yposition,zposition,roll,pitch,yaw, &tf_listener, &br);
    }
    group.setPoseTarget(new_pose);
  }

  group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
//Plan a motion path to the user-provided position
  ROS_INFO("Planning...");
  move_group_interface::MoveGroup::Plan p;

  //If position is unreachable or a path is not possible, the user will be prompted to put in new information
  if(!group.plan(p))
  {
    ROS_ERROR("Destination is unreachable!");
    return 0;
  }
  
  ROS_INFO("Executing...");
  group.execute(p);
  ROS_INFO("Trajectory complete.");

  return 0;
}
