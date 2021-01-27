# grapschi
ros moveit code für Moveo 6dof(mod) robot arm


# This is a clone of jesseweisberg/moveo_ros but will be 6DOF version (maybe if i get it up and running) 

Found some improvements for the Moveo (endstops, some extra bearings and a 6th DOF ("rotating hand")) on thingiverse ! https://www.thingiverse.com/thing:2146252 by labala

should run on 1x arduino mega 2560 with ramps 1.4 for all motors. 
Changes to the original code are:

Stepper Z is now for Joint 6 !!!
There is an Arduino CNC shield attached to RAMPS AUX2 port, for the two Steppers which control the Joint 2 (nema23)

using ros Kinetic ! (because code was made for kinetic...if everything is running i will try to port to Neotic (moveit visual tools not yet running on neotic ?))

# Installation of stuff
1. install Ubuntu 16.04 amd64 on ssd
2. install ROS like in http://wiki.ros.org/kinetic/Installation/Ubuntu
	install fill desktop version (sudo apt-get install ros-kinetic-desktop-full)
	install catkin !!! and make a workspace which works
3. install moveit https://moveit.ros.org/install/ with  (sudo apt-get install ros-kinetic-moveit) and install also moveit visual tools https://github.com/ros-planning/moveit_visual_tools via ( sudo apt-get install ros-kinetic-moveit-visual-tools )

4. follow the instructions from jesse weisberg (original description further down)

moveit rviz should work! then we need rosserial !!!

# Update
everything seems to run at the moment, and my robot is working (no bearings yet...but everything moves)
rostopic pub joint_steps moveo_moveit/ArmJointState <Joint1 Joint2 Joint3 Joint4 Joint5 0> (needs to be put like rostopic pub joint_steps moveo_moveit/ArmJointState 100 100 100 100 100 0 )
	- Change "Joint1, Joint2, etc." to the number of steps you want each joint to move.
	
this worked! but value 0 seems to be the starting value for all joints, and thats not cool because putting negative values seems to not work!... soo what can we do instead? 

fortunately the code of jesse all in all seems to have been written actually for 6 axis robot...which is nice.

. Limits in the moveit config seem to be very much off (foind that from the moveit config tutorial). also im using different microstepping settings...and im gona use a planetary gearbox 1:5 for the joint 4 (direct drive is really not working well)

(also i found that controlling the endeffector with xbox360 controller is very laggy but cpu is idling when starting joystick node so there seems to be other issues with that) 

next up will be updating the urdf files with the 6th axis

# UPdate 2
so there is a urdf file generator for solidworks !!! ...how convenient...i guess everybody used that.
...but that program is way to expensive for this project.... so fortunately i found https://github.com/andresaraque/centauri6dof <- some guys who also used the 6 axis mod of the moveo and exported the kinematic model. they also have some gui which is fun to play with (already tested and seems to work)

...next up planetary gearbox für joint 4 and playing around with the code.... will also try if this centauri6dof code runs on newer ROS versions // 16.04 is tooo old

# update 3
