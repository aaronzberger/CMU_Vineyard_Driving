# CMU_Vineyard_Driving
Make a robot autonomously drive down simulated wall rows, using RANSAC line detection, an Extended Kalman Filter for tracking, and a PID controller for driving.

This is the original testing code for an autonomous vineyard navigation pipeline. The first node in that pipeline is [here](https://github.com/aaronzberger/CMU_UNet_Node).

Eventually, I may update this repository with testing in simulated vine rows.

#### Usage:
    
  rosrun pid_controller wall_controller \[P_gain] \[I_gain] \[D_gain]
  
If you run into any problems when compiling or executing, or you have any questions, please contact me: aaronzberger@gmail.com
