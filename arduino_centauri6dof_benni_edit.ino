#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ros.h>
#include <Servo.h> 
#include "centauri6dof_moveit/ArmJointState.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>  
#include <std_msgs/Float64.h>


// Joint 1 Rotation gesamter arm (2.0A)
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2 NEMA23 stepper 2x (auf cnc shield ueber aux2) (2x 1.5A)
#define AUX_STEP_PIN         63
#define AUX_DIR_PIN          40
#define AUX_ENABLE_PIN       42

#define JOINT2_STEP_PIN_M2     18 //CLK+
#define JOINT2_DIR_PIN_M2      19 //CW+

// Joint 3 NEMA17 mit untersetzung 1:5 (2.1A)
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
//#define Y_MIN_PIN          14
//#define Y_MAX_PIN          15

// Joint 4 NENA 17 (2.0A)
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 NEMA 14 (0.8A)
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

//// Joint 6 !!! NEMA 14 (0.8A)
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
//#define Z_MIN_PIN          18
//#define Z_MAX_PIN          19


AccelStepper joint1   (1, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint2_m1(1, AUX_STEP_PIN, AUX_DIR_PIN);
AccelStepper joint2_m2(1, JOINT2_STEP_PIN_M2, JOINT2_DIR_PIN_M2);
AccelStepper joint3   (1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4   (1, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5   (1, E0_STEP_PIN, E0_DIR_PIN);
AccelStepper joint6   (1, Z_STEP_PIN, Z_DIR_PIN);

Servo gripper; 
int pos_gripper = 0;
int algo = 0;
MultiStepper steppers; 

int joint_step[7];//[joint1,joint2,joint3,joint4,joint5,joint6,servo]
int joint_status = 0;
int pos = 0;
int eff0 = 0;//Efector final cerrado
int eff1 = 0;//Efector final abierto

float Sensibility = 0.185;

ros::NodeHandle nh; // NodeHandle declaration with instance nh
std_msgs::Int16 msg;
std_msgs::Float64 test;


void arm_cb(const centauri6dof_moveit::ArmJointState& arm_steps){
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6;
  joint_step[6] = arm_steps.position7; //Gripper position <0-89>.
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  //gripper.write(msg_angulo.data);
   
  if(cmd_msg.data > 0)
  {
    for(pos = 0; pos < 90; pos += 1)   // Ranges from 0 to 89° In 1-degree steps
    {                                   
      gripper.write(pos);              // Tell the servo to acquire the variable pos. 
      delay(5);                        // Wait 5ms for the servo to reach the position. 
    }
  }
  
  if(cmd_msg.data == 0)
  {
    for(pos = 90; pos>=1; pos-=1)      // Ranges from 89 to 0°. 
    {                                
      gripper.write(pos);              // Tell the servo to acquire the variable pos. 
      delay(5);                        // Wait 5ms for the servo to reach the position. 
    }
  }
}
/* //Current
float get_current(int n){
  float VoltSens;
  float current = 0;
  for(int i = 0; i < n; i++){
    VoltSens = analogRead(A0) * (5.0 / 1023.0);
    current = current+(VoltSens-2.5)/Sensibility;
  }
  current = current/n;
  return(current);
}
*/
/*------------------definition of subscriber objects------------------*/
//the function arm_cb is executed when there is a message in the topic joint_steps
ros::Subscriber<centauri6dof_moveit::ArmJointState> arm_sub("joint_steps",arm_cb);

//the function arm_cb is executed when there is a message in the topic gripper_angle
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); 

/* //Current
ros::Publisher p("current", &test);
*/

void setup() {
  //i dont know if it is neccesary to set all to output (it is not for the dir and step pins ??)
  pinMode(E1_ENABLE_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT);
  pinMode(AUX_ENABLE_PIN, OUTPUT);

  digitalWrite(E1_ENABLE_PIN, LOW);
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(E0_ENABLE_PIN, LOW);
  digitalWrite(AUX_ENABLE_PIN, LOW); 

  
  //Serial.begin(57600);
  joint_status = 1;

  // node initialization for the use of serial port communication
  nh.initNode(); 
  
  //Initialize subscribers
  nh.subscribe(arm_sub); 
  nh.subscribe(gripper_sub);
  /* //Current
  nh.advertise(p);
  */

  //Assignment of maximum speed value for each motor
  //EDIT: setAcceleration also! so the movement begins and ends more smoothly
  joint1.setMaxSpeed(1500);
  joint1.setAcceleration(100);
  
  joint2_m1.setMaxSpeed(400);
  joint2_m1.setAcceleration(100);
  
  joint2_m2.setMaxSpeed(400);
  joint2_m2.setAcceleration(100);
  
  joint3.setMaxSpeed(2000);
  joint3.setAcceleration(100);
  
  joint4.setMaxSpeed(200);
  joint4.setAcceleration(100);
  
  joint5.setMaxSpeed(1000);
  joint5.setAcceleration(100);
  
  joint6.setMaxSpeed(500);
  joint6.setAcceleration(100);

  //AAdd motors to the MultiStepper library
  steppers.addStepper(joint1);
  steppers.addStepper(joint2_m1);
  steppers.addStepper(joint2_m2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);

  //Assign PWM port 4 to AL Gripper
  gripper.attach(11); //original 4 -> i changed it to 11 because 11 is very accessible on the RAMPS 1.4

}

void loop() {
  /* //Current
  float I = get_current(200); 
  test.data = I;
  p.publish( &test );
  */
  if (joint_status == 1){ // If arm_cb is being called it sets the joint_state to 1.
    
    long positions[7];

    positions[0] = joint_step[0];     //8000  = 90°
    positions[1] = -joint_step[1];    //-4100  = 90°
    positions[2] = joint_step[1];     //4100 = 90°
    positions[3] = joint_step[2];     //18000 = 90°
    positions[4] = joint_step[3];     //800   = 90°
    positions[5] = -joint_step[4];    //3600  = 90°
    positions[6] = joint_step[5];     //750   = 90°

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition();
    
    if(joint_step[6] > 0){
      if(eff1 == 0){
        for(pos = 0; pos < 90; pos += 1){  // Ranges from 0 to 89° In steps of 1 degree                                   
          gripper.write(pos);              // Instruct the servo to acquire the variable pos. 
          delay(5);                        // Wait 5ms for the servo to reach the position. 
        }        
      }
      eff1++;
      eff0 = 0;
    }

    if(joint_step[6] == 0){
      if(eff0 == 0){
        for(pos = 90; pos>=1; pos-=1){     // Ranges from 89 to 0°.                               
          gripper.write(pos);              // Instruct the servo to acquire the variable pos.
          delay(5);                        // Wait 5ms for the servo to reach the position.
        }
      }
      eff0++;
      eff1 = 0;
    }    
  }
  joint_status = 0;

  nh.spinOnce();
  delay(1);
}
