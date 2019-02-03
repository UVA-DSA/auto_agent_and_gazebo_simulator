This is a combine of ROS packages about Raven II surgical robot.
Todo: more description
# auto_agent
A lab projcet of the surgical robots -- Raven II. Autonomous agent for picking up an object and transferring it to a container. A gazebo simulator. This pipeline works without the real robot device.
Moveit! is used.

## auto_agent/image_analysis
for image processing
## auto_agent/pc_analysis
for analysis of point cloud data from a 3D camera

# raven_2arm
gazebo simulator extended from the raven_ros_gazebo.
Two arms are included.
Jennifer grasp fix is used to increase the grasping success rate.
A virtual camera is added.

# raven_ros_gazebo
gazebo simulator from UW with only one arm
PID controller is used to control the robot.
Some code of inverse kinematics of the UW r2_control code is included to realize Cartesian space control.

# raven_visulizaton
The gazebo simulator Robort worked on. The code of this version does not consider the PID control of each joint.
The code absolutely set the status of the robot.

# raven2
The UW r2_control code

# pose_est
for pose estimation of object, instrment graspers and their relations.

# zed-ros-wrapper
functions for zed camera


