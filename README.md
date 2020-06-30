# CMU_PID
A PID-based controller to drive a Husky through a vineyard (or through other objects)

To make this program accessible on your computer, clone this repository to your catkin workspace and compile.

  To use in simulations, this program requries certain Husky simulators to run. 
  To install these simulators and other Husky tools, follow the "Using Husky" section on
  http://www.clearpathrobotics.com/assets/guides/melodic/husky/

  To use in simulations, this program also requires launch files to receive the 3d Lidar data. 
  To use these launch files, clone the repository below to your catkin workspace and compile:
  https://github.com/fyandun/Husky_velodyne


To run this program, use the following commands:
  
  (To launch the simulators) roslaunch husky_velodyne husky_vlp16.launch //This will launch Gazebo and Rviz and spawn the Husky
  
rosrun pid_controller wall_controller [P_gain] [I_gain] [D_gain]
  
  
If you run into any problems when compiling or executing, please contact me: aaronzberger@gmail.com
