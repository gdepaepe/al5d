# ROS AL5D robot arm controller

Manage the Lynxmotion AL5D robot arm (with medium duty wrist rotate) with ROS, with joint and cartesian coordinates (using an inverse kinematics service).


### Prerequisites

* ROS driver for the Lynxmotion SSC-32: git clone https://github.com/smd-ros-devel/lynxmotion_ssc32.git
* AL5D robot description: git clone https://github.com/gdepaepe/al5d_description.git
* AL5D gazebo: git clone https://github.com/gdepaepe/al5d_gazebo.git
* AL5D scripts: git clone https://github.com/gdepaepe/al5d.git


### Installing

Change "baud" rate in "lynxmotion/src/ssc32_driver.cpp" to 9600 (afterwards do catkin_make). Add your username to the "dialout" group in /etc/group.

Home position robot arm:

           wrist-
          =.--.--------.   elbow
              tilt     |
           rotate      |
                       |
                       .   shoulder
                     -----
                     | . | base
                     -----

Servos are connected to channel 0 to 5 (see al5d/config/al5d.config).




