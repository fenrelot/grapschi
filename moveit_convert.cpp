// load ros
#include "ros/ros.h"
// mensaje de valores de articulaciones mediante libreria jointState 
// message of joint values via jointState library 
#include "sensor_msgs/JointState.h"
// ArmJointState se crea automaticamente en la libreria de arduino mediante ros_lib
// ArmJointState is automatically created in the arduino library by ros_lib
#include "centauri6dof_moveit/ArmJointState.h"
#include "math.h"

// cargo de estados de articulaciones mediante el parametro 
// load of joint states by means of the arm_step parameter (?)
centauri6dof_moveit::ArmJointState arm_steps;
centauri6dof_moveit::ArmJointState total;

// micro pasos/revolución (usando 16tHz) por observación, para cada motor
// microsteps/revolution (using 16ths) from observation, for each motor
int stepsPerRevolution[7] = {32000,16400,72000,3200,14400,3000,0}; 
int joint_status = 0;
double cur_angle[7];
int joint_step[7];
double prev_angle[7] = {0,0,0,0,0,0,0};
double init_angle[7] = {0,0,0,0,0,0,0};
double total_steps[7] = {0,0,0,0,0,0,0};

// contador para indicar si ya se tiene una posicion establecida
//counter to indicate whether a position has already been established
int count = 0;


//funcion para hacer llamdo de comandos por posicion 
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

// convertir parametros de pasos por revolucion a radianes 
//convert revolution step parameters to radians 
  arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  arm_steps.position4 = (int)((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));
  arm_steps.position5 = (int)((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*M_PI));
  arm_steps.position6 = (int)((cmd_arm.position[5]-prev_angle[5])*stepsPerRevolution[5]/(2*M_PI));
  arm_steps.position7 = (int)((cmd_arm.position[6]-prev_angle[6])*stepsPerRevolution[6]/(2*M_PI));

  //%d para imprimir los parametros de posicion 6 como test
  //to print the position 6 parameters as a test
  ROS_INFO_NAMED("test", "arm_steps.position6 #2: %d", arm_steps.position6);

// condicion si el contador es diferente de cero, actualiza las posiciones
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

//pasos totales tomados para llegar al objetivo
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

// consola informacion de que ya ha convetido los pasos 
// console information that you have already completed the steps 
  ROS_INFO_STREAM("Done conversion to /joint_steps");
  //empiza a correr los comando de void loop() de arduino 
    //start running the arduino void loop() commands 
  joint_status = 1;
  // contador en 1 para saber posicion previa de inicio 
    // counter in 1 to know previous starting position 
  count=1;
}
//main fuction 
int main(int argc, char **argv)
{
  //incia paquete de moveit del robot 
    //initiates robot moveit package 
  ros::init(argc, argv, "centauri6dof_moveit"); 
  // comunicacion serial con ros
    // serial communication with ros
  ros::NodeHandle nh;
  //mensaje de consola de incio de la funcion
    //console message of function startup
  ROS_INFO_STREAM("In main function");
  // subcripcion a los paquetes de las articulaciones del robot
    // subcription to the robot joint packages
  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
  //publica las articulaciones 
    //publishes the joints 
  ros::Publisher pub = nh.advertise<centauri6dof_moveit::ArmJointState>("joint_steps",50);
// frecuencia a la que corre el loop 20hz
// frequency at which the loop runs 20hz
  ros::Rate loop_rate(50);
//Controlador SIGINT Instalado por roscpp para manejo de ctrl-C que suspende operaciones, ros::ok return false
//SIGINT driver Installed by roscpp to handle ctrl-C suspending operations, ros::ok return false 
  while (ros::ok())
  {
    if(joint_status==1)
    {
      joint_status = 0;
      //pub.publish(arm_steps);

      // publicar estados de articulaciones
      // publish joint statuses
      pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
    }
    ros::spinOnce();
    // modo de reposo del loop 
    // loop idle mode
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
