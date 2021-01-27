// load ros
#include "ros/ros.h"

// message of joint values via jointState library 
#include "sensor_msgs/JointState.h"

// ArmJointState is automatically created in the arduino library by ros_lib
#include "centauri6dof_moveit/ArmJointState.h"
#include "math.h"


// load of joint states by means of the arm_step parameter (?)
centauri6dof_moveit::ArmJointState arm_steps;
centauri6dof_moveit::ArmJointState total;

// microsteps/revolution berechnet mit Ubersetzungsverhaeltnissen und 8x microstepping
int stepsPerRevolution[7] = {16000,8800,34857,1600,7200,1600,0}; 
int joint_status = 0;
double cur_angle[7];
int joint_step[7];
double prev_angle[7] = {0,0,0,0,0,0,0};
double init_angle[7] = {0,0,0,0,0,0,0};
double total_steps[7] = {0,0,0,0,0,0,0};


//counter to indicate whether a position has already been established
int count = 0;



//keep a running sum of all the step counts and use that as the final step to send to arduino accelstepper

// int angle_to_steps(double x)
// {
//   float steps;
//   steps=((x / M_PI)*stepsPerRevolution)+0.5; // (radians)*(1 revolution/PI radians)*(200 steps/revolution)
//   return steps;
// }

//command callback (for position) function 
void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
  if (count==0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];

    init_angle[0] = cmd_arm.position[0];
    init_angle[1] = cmd_arm.position[1];
    init_angle[2] = cmd_arm.position[2];
    init_angle[3] = cmd_arm.position[3];
    init_angle[4] = cmd_arm.position[4];
    init_angle[5] = cmd_arm.position[5];
    init_angle[6] = cmd_arm.position[6];
  }

  // ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
  // ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps",50);
  ROS_INFO_STREAM("Received /move_group/fake_controller_joint_states");

 // arm_steps.position1 = (cmd_arm.position[0]*stepsPerRevolution[0]/M_PI+0.5)-prev_angle[0];
  // arm_steps.position2 = (cmd_arm.position[1]*stepsPerRevolution[1]/M_PI+0.5)-prev_angle[1];
  // arm_steps.position3 = (cmd_arm.position[2]*stepsPerRevolution[2]/M_PI+0.5)-prev_angle[2];
  // arm_steps.position4 = (cmd_arm.position[3]*stepsPerRevolution[3]/M_PI+0.5)-prev_angle[3];
  // arm_steps.position5 = (cmd_arm.position[4]*stepsPerRevolution[4]/M_PI+0.5)-prev_angle[4];
  // arm_steps.position6 = (cmd_arm.position[5]*stepsPerRevolution[5]/M_PI+0.5)-prev_angle[5];

  //compute relative step count to move each joint-- only works if all joint_angles start at 0
  //otherwise, we need to set the current command to the initial joint_angles
  //ROS_INFO_NAMED("test", "cmd_arm.position[4]: %f, prev_angle[4]: %f, init_angle[4]: %f", cmd_arm.position[4], prev_angle[4], init_angle[4]);
  //ROS_INFO_NAMED("test", "arm_steps.position5 #1: %f", (cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/M_PI);


  //convert revolution step parameters to radians 
  arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  arm_steps.position4 = (int)((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));
  arm_steps.position5 = (int)((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*M_PI));
  arm_steps.position6 = (int)((cmd_arm.position[5]-prev_angle[5])*stepsPerRevolution[5]/(2*M_PI));
  arm_steps.position7 = (int)((cmd_arm.position[6]-prev_angle[6])*stepsPerRevolution[6]/(2*M_PI));

  
  //%d to print the position 6 parameters as a test
  ROS_INFO_NAMED("test", "arm_steps.position6 #2: %d", arm_steps.position6);


// condition if the counter is different from zero, updates the positions
  if (count!=0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];
  }


//total steps taken to reach the objective

//total steps taken to get to goal
  // total_steps[0]+=arm_steps.position1;
  // total_steps[1]+=arm_steps.position2;
  // total_steps[2]+=arm_steps.position3;
  // total_steps[3]+=arm_steps.position4;
  // total_steps[4]+=arm_steps.position5;

  total.position1 += arm_steps.position1;
  total.position2 += arm_steps.position2;
  total.position3 += arm_steps.position3;
  total.position4 += arm_steps.position4;
  total.position5 += arm_steps.position5;
  total.position6 += arm_steps.position6;

  ROS_INFO_NAMED("test", "total_steps[5]: %f, total: %d", total_steps[5], total.position6);
  ROS_INFO_NAMED("test", "arm_steps.position6 #3: %d", arm_steps.position6);


// console information that you have already completed the steps 
  ROS_INFO_STREAM("Done conversion to /joint_steps");

    //start running the arduino void loop() commands 
  joint_status = 1;

    // counter in 1 to know previous starting position 
  count=1;
}
//main fuction 
int main(int argc, char **argv)
{

    //initiates robot moveit package 
  ros::init(argc, argv, "centauri6dof_moveit"); 

    // serial communication with ros
  ros::NodeHandle nh;

    //console message of function startup
  ROS_INFO_STREAM("In main function");

    // subcription to the robot joint packages
  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
 
    //publishes the joints 
  ros::Publisher pub = nh.advertise<centauri6dof_moveit::ArmJointState>("joint_steps",50);

// frequency at which the loop runs 20hz
  ros::Rate loop_rate(50);

//SIGINT driver Installed by roscpp to handle ctrl-C suspending operations, ros::ok return false 
  while (ros::ok())
  {
    if(joint_status==1)
    {
      joint_status = 0;
      //pub.publish(arm_steps);

     
      // publish joint statuses
      pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
    }
    ros::spinOnce();
   
    // loop idle mode
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
