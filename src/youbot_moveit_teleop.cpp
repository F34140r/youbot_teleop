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

void trajectory_execution(move_group_interface::MoveGroup *g, move_group_interface::MoveGroup::Plan *plan)
{
  trajectory_complete = g->execute(*plan);
}

void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  goal_string = msg->goal_id.id;
}

/*
void planning_scene_callback(const moveit_msgs::PlanningScene::ConstPtr& msg)
{
  current_scene.clone(msg);
}


void clear_scene()
{
  moveit_msgs::PlanningScene cleared_scene(current_scene);

  
  //  I can show you the world 
  //  Shining, shimmering, splendid 
  //  Tell me, princess, now when did 
  //  You last let your heart decide?

  //  I can open your eyes 
  //  Take you wonder by wonder 
  //  Over, sideways and under 
  //  On a magic carpet ride

  cleared_scene = current_scene;

  //A whole new wooooooooooooorld!
  collision_detection::World new_world;
  
  //A new fantastic point of view!
  cleared_scene.world = new_world;
}
*/

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

  cout << "xpo = " << xpo << endl;
  cout << "ypo = " << ypo << endl;
  cout << "zpo = " << zpo << endl;

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
  ros::init(argc, argv, "youbot_moveit_teleop", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  ros::NodeHandle n;

  ros::Publisher marker_publisher = n.advertise<visualization_msgs::Marker>("goal_marker", 1);
  ros::Publisher cancel_publisher = n.advertise<actionlib_msgs::GoalID>("/arm_1/arm_controller/follow_joint_trajectory/cancel", 1);
  ros::Subscriber goal_listener = n.subscribe("/arm_1/arm_controller/follow_joint_trajectory/goal",1,goal_callback);


  tf::TransformListener tf_listener;
  tf::TransformBroadcaster br;

  cout << "\nStarting youbot_moveit_teleop..." << endl;
  spinner.start();

  cout << "Connecting to move_group node for youbot arm..." << endl;
  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm");

  group.setEndEffector("gripper_palm_link");
  //group.setEndEffectorLink("arm_link_5");

  std::string eef_link = group.getEndEffectorLink();
  std::string eef = group.getEndEffector();

  cout << "\nEnd Effector Link for this group is: " << eef_link << endl;
  cout << "\nEnd Effector for this group is: " << eef << endl;

  vector<string> states;

  robot_model::RobotModel temp = *(*group.getCurrentState()).getRobotModel();

  (*temp.getJointModelGroup("arm")).getKnownDefaultStates(states);

  double tolerance = .1;
  double orientationtolerance = 3.141592654;
  double positiontolerance = 0.05;

  //cout << "Setting goal tolerance";
  group.setGoalTolerance(tolerance);
  //group.setGoalOrientationTolerance(orientationtolerance);
  //group.setGoalPositionTolerance(positiontolerance);







  // BEGINNING OF MAIN LOOP ////////////////////////////////////////////////
  while(ros::ok())
  {

    visualization_msgs::Marker goalmarker;
    goalmarker.header.frame_id = "/odom";
    goalmarker.id = 0;
    goalmarker.type = goalmarker.MESH_RESOURCE;
    goalmarker.action = goalmarker.ADD;
    goalmarker.scale.x = 1;
    goalmarker.scale.y = 1;
    goalmarker.scale.z = 1;
    goalmarker.pose.position.x = 0;
    goalmarker.pose.position.y = 0;
    goalmarker.pose.position.z = 0;
    goalmarker.color.r = 0;
    goalmarker.color.g = 1;
    goalmarker.color.b = 0;
    goalmarker.color.a = 1;
    goalmarker.mesh_resource = "package://youbot_description/meshes/youbot_arm/arm5.dae";


    cout << "\nSetting starting position to current position..." << endl;
    group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
    
    cout << "Select mode of control:\n\t1)\tTarget specific point\n\t2)\tControlled Marker Teleop\n\t3)\tJoint Poses\n\t4)\tExit" << endl;
    
    int choose = 0;

    cin >> choose;

    double xposition = 0;
    double yposition = 0;
    double zposition = 0;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    geometry_msgs::Pose new_pose;












    //CHOICE 1:  TARGETING A SPECIFIC END-EFFECTOR POS/ORI//////////////////
    if(choose == 1)
    {
      cout << "\n---------------------------------------------------------------------" << endl;

      cout << "Is your target frame relative to the arm or to the global frame (/odom)?\n\t1)\tarm\n\t2)\todom" << endl;

      int frame = 0;

      cin >> frame;

      cout << "\nPlease provide an x,y, and z position that the arm will move towards." << endl;

      cout << "\nPlease enter an x position for the arm: ";
      cin >> xposition; //The user-provided x-position for the end-effector
      cout << "Please enter a y position for the arm: ";
      cin >> yposition; //The user-provided y-position for the end-effector
      cout << "Please enter a z position for the arm: ";
      cin >> zposition; //The user-provided z-position for the end-effector
      cout << "\n\nYou chose the point (" << xposition << ", " << yposition << ", " << zposition << ") ";

      cout << "\nPlease provide a roll,pitch, and yaw orientation that the arm will move towards." << endl;

      cout << "\nPlease enter a roll for the arm: ";
      cin >> roll; //The user-provided x-position for the end-effector
      cout << "Please enter a pitch for the arm: ";
      cin >> pitch; //The user-provided y-position for the end-effector
      cout << "Please enter a yaw for the arm: ";
      cin >> yaw; //The user-provided z-position for the end-effector
      cout << "\n\nYou chose the point (" << xposition << ", " << yposition << ", " << zposition << ") with orientation (" << roll << ", " << pitch << ", " << yaw << ") ";

      new_pose.position = getPosition(xposition, yposition, zposition);

      new_pose.orientation = getQuatFromRPY(roll, pitch, yaw);

      if(frame == 1)
      {     
        cout << "relative to the arm." << endl;
        ROS_INFO("Transforming arm coordinate frame to global (odom) coordinate frame.");

        new_pose = getArmPointRelativeToOdom(xposition,yposition,zposition,roll,pitch,yaw, &tf_listener, &br);
      }
      if(frame == 2)
      {
        cout << "relative to the global (odom) coordinate frame." << endl;
      }
      if((frame != 1)&&(frame != 2))
      {
        cout << "." << endl;
        ROS_WARN("Invalid frame provided.  Defaulting to global (odom) coordinate frame.");
      }

      goalmarker.pose.position = new_pose.position;

      goalmarker.pose.orientation = new_pose.orientation;

      marker_publisher.publish(goalmarker);

    } //END OF CHOICE 1/////////////////////////////////////////////////













    //CHOICE 2:  USING THE FAKE INTERACTIVE MARKERS/////////////////////
    if(choose == 2)
    {
      cout << "Generating initial marker at end effector." << endl;
      
      tf::StampedTransform tf_odom_link_5;

      try
      {
        tf_listener.lookupTransform("/odom","arm_link_5",ros::Time(0), tf_odom_link_5);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      goalmarker.pose.position.x = tf_odom_link_5.getOrigin().getX();
      goalmarker.pose.position.y = tf_odom_link_5.getOrigin().getY();
      goalmarker.pose.position.z = tf_odom_link_5.getOrigin().getZ();
  
      //goalmarker.pose = goalmarker.pose * transform;

      cout << "To move the target marker along the x-axis, press w/s.\nTo move the target marker along the y-axis, press a/d.\nTo move the target marker along the z-axis, press t/g.\nTo change roll angle, press y/h.\nTo change pitch angle, press g/j.\nTo change yaw angle, press i/k.\nTo confirm your target location, press 'p'." << endl;

      char keyboardInput = '0';

      while((keyboardInput != 'p')&&ros::ok())
      {
        keyboardInput = getch();

        //Position controls        
        if(keyboardInput == 'w')
          goalmarker.pose.position.x += 0.01;
        if(keyboardInput == 's')
          goalmarker.pose.position.x -= 0.01;
        if(keyboardInput == 'a')
          goalmarker.pose.position.y += 0.01;
        if(keyboardInput == 'd')
          goalmarker.pose.position.y -= 0.01;
        if(keyboardInput == 'r')
          goalmarker.pose.position.z += 0.01;
        if(keyboardInput == 'f')
          goalmarker.pose.position.z -= 0.01;
        
        //Roll, Pitch, Yaw controls
        if(keyboardInput == 'y')
          roll += 0.01;
        if(keyboardInput == 'h')
          roll -= 0.01;
        if(keyboardInput == 'g')
          pitch += 0.01;
        if(keyboardInput == 'j')
          pitch -= 0.01;
        if(keyboardInput == 'i')
          yaw += 0.01;
        if(keyboardInput == 'k')
          yaw -= 0.01;

        goalmarker.pose.orientation = getQuatFromRPY(roll, pitch, yaw);
        
        marker_publisher.publish(goalmarker);
      }

      tf::Transform goal_tf;
      tf::Transform tf_arm_goalmarker;

      tf::Vector3 goal_tf_origin;
    
      goal_tf_origin.setX(goalmarker.pose.position.x);
      goal_tf_origin.setY(goalmarker.pose.position.y);
      goal_tf_origin.setZ(goalmarker.pose.position.z);

      goal_tf.setOrigin(goal_tf_origin);

      tf::StampedTransform tf_arm_odom;

      try
      {
        tf_listener.lookupTransform("base_link", "/odom",ros::Time(0), tf_arm_odom);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      //arm->goal         //arm->odom       //odom->goal
      tf_arm_goalmarker = tf_arm_odom  *  goal_tf;

      double goalXToArmX = tf_arm_goalmarker.getOrigin().getX();
      double goalYToArmY = tf_arm_goalmarker.getOrigin().getY();
      double goalZToArmZ = tf_arm_goalmarker.getOrigin().getZ();

      cout << "Target selected: (" << goalXToArmX << ", " << goalYToArmY << ", " << goalZToArmZ << ")" << endl;

      xposition = goalmarker.pose.position.x;
      yposition = goalmarker.pose.position.y;
      zposition = goalmarker.pose.position.z;  

      new_pose.position = goalmarker.pose.position;    
      new_pose.orientation = goalmarker.pose.orientation;
    } //END OF CHOICE 2/////////////////////////////////////////////////////










    //CHOICE 3:  MOVING TO SPECIFIC POSES FOR EACH JOINT////////////////////
    if(choose == 3)
    { 
      cout << "Joint Pose Target Names: " << endl;

      for(vector<string>::iterator i= states.begin(); i!= states.end(); ++i)
      {
        cout << "\t" << (*i) << endl;
      }        

      string target;

      cout << "Specify name of joint pose target to reach: ";
      cin >> target;
      if(!group.setNamedTarget(target))
      {
        ROS_ERROR("\nInvalid pose name provided.  Please try again.");
        //temp.robot_model::RobotModel::~RobotModel();
        continue;
      }
    } //END OF CHOICE 3/////////////////////////////////////////////////////











    //CHOICE 4:  EXITING////////////////////////////////////////////////////
    if(choose == 4)
    {
      cout << "Exiting...";
      break;
    } //END OF CHOICE 4/////////////////////////////////////////////////////



    if(choose == 1337)
    {
      xposition = -0.114;
      yposition = -0.013;
      zposition = 0.408;

      roll = -0.005;
      pitch = -1.284;
      yaw = 0.052;

      geometry_msgs::Pose temp_pose = getArmPointRelativeToOdom(xposition, yposition, zposition, roll, pitch, yaw, &tf_listener, &br);  
      
      goalmarker.pose.position = temp_pose.position;

      goalmarker.pose.orientation = temp_pose.orientation;
      
      marker_publisher.publish(goalmarker);

      new_pose.position = goalmarker.pose.position;
      new_pose.orientation = goalmarker.pose.orientation;
    }












    // All options aside from 3 and 4 use specified positions and set a different targets in different ways
    if(choose != 3)
    {
      cout << "Setting Position target to the provided location: (" << xposition << ", " << yposition << ", " << zposition << ") with orientation: (" << roll << ", " << pitch<< ", " << yaw << ")" << endl;

      //Sets the target position for the move_group to the user-sepcified location
      group.setPoseTarget(new_pose);
     
      cout << "\nPose target is: \n" << group.getPoseTarget(eef).pose << endl;
    }
    cout << "\n\t\tJoint targets are:\tCurrent joint values are:" << endl;
    for(int i = 0; i < group.getJointValueTarget().getJointNames().size(); i++)
    {
      robot_state::JointState j = *group.getJointValueTarget().getJointStateVector().at(i);

      cout << group.getJointValueTarget().getJointNames()[i] << ":  " << j.getVariableValues().back() << "\t\t\t" << group.getCurrentJointValues().at(i) << endl;
     
    } // END OF "NOT CHOICE 3"





    cout << "\n";

    cout << "Setting starting position to current position..." << endl;
    group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
    //Plan a motion path to the user-provided position
    move_group_interface::MoveGroup::Plan p;

    //If position is unreachable or a path is not possible, the user will be prompted to put in new information
    if(!group.plan(p))
    {
      ROS_ERROR("Destination is unreachable!");
      continue;
    }

    goalmarker.pose.position = getPosition(xposition, yposition, zposition);

    goalmarker.pose.orientation = getQuatFromRPY(roll, pitch, yaw);

    marker_publisher.publish(goalmarker);

    cout << "Please press be sure to monitor the arm during its trajectory.  If you wish to cancel the trajectory during execution, please press 'c' during execution." << endl << "To start the trajectory, please press 's' and then enter: ";

    char enter = 0;

    cin >> enter;



    // BEGINNING OF ARM MOVEMENT AND CANCELLING
    if(enter == 's')
    {
      trajectory_complete = false;

      cout << "~~Moving to requested position~~" << endl; 

      goal_string = "0";
      //Execute will perform the trajectory.  It is non-blocking. <-- Not true!
      boost::thread execute_thread(trajectory_execution, &group, &p); //Create new thread to execute the blocking (at the time of writing this they lied and say it is non-blocking) trajectory command

      boost::thread read_cancel(getch_check); //Create new thread to monitor the movement of current trajectory for canceling


      // BEGINNING OF CANCELING
      while(ros::ok())
      {
        ros::spinOnce();

        //Hitting 'c' mid trajectory will cancel the current trajectory
        if(cancel == 'c')
        {
          actionlib_msgs::GoalID g_id;
          g_id.id = goal_string; 

          cancel_publisher.publish(g_id); //Provide the cancel_trajectory service with the goal id to cancel
          cout << "Canceled the current trajectory!" << endl;
          cout << "Setting starting position to current position..." << endl;
          group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
          read_cancel.join(); //End cancel_trajectory thread
          break;
        }
        
        
        // If move_group reports trajectory is complete
        if(trajectory_complete)
        {
          ROS_INFO("Trajectory completed.");
          execute_thread.join(); //End trajectory_complete thread
          read_cancel.interrupt(); //Kill idled cancel_trajectory thread
          break; //Return to main loop
        }

      } //END OF CANCELLING


    } // END OF ARM MOVEMENT AND CANCELLING



    cout << "Hit Control + C to exit." << endl;
    r.sleep();
  } // END OF MAIN LOOP ////////////////////////////////////////////////////

  states.clear();
}
