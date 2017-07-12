# CPFA-ROS

Please make sure that you have read all of the [documentation](https://github.com/BCLab-UNM/CPFA-ROS) and have set up all of the software regarding the Swarmathon as well as having read the [paper](https://www.cs.unm.edu/~melaniem/Publications_files/Hecker_Beyond_Pheromones_Swarm_Intelligence_2015.pdf)  that contains the algorithm.

This repository is a ROS (Robot Operating System) controller framework for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition. This particular framework is a ROS implementation of the CPFA (central-place foraging algorithm) developed for [iAnt robot swarms](http://swarms.cs.unm.edu) at the [University of New Mexico](http://www.unm.edu/).


### Running the CPFA code

Assuming you have installed everything as the Swarmathon-ROS repository has instructed and you have read the paper, then we can begin. The CPFA relies a set of parameters that can be modified depending on the resource distribution that will be used for a run. 

In the ` ~/rover_workspace/CPFA_parameters/` directory you will see a set of files with a `.yaml` extension. Each corresponding to the appropriate resource distribution and have been hand tuned to have paramters that will work for each of those distributions, albeit not optimally. 

You will see all the parameters that the paper describes as the paramters evolved by the GA. 

To start the simulation you will run the `~/rover_workspace/run.sh` script like normal to start up the GUI.











This is the first screen of the GUI:

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/guiFirstScreen.png "Opening Screen")

Click the "Simulation Control" tab:

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/simControlTab.png "Simulation Parameters")

Choose the ground texture, whether this is a preliminary or final round (3 or 6 robots), and the distribution of targets.

Click the "Build Simulation" button when ready.

The gazebo physics simulator will open.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/buildSim.png "Gazebo Simulator")

Click back to the Swarmathon GUI and select the "Sensor Display" tab.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/sensorDisplayTab.png "Sesnor display")

Any active rovers, simulated or real will be displayed in the rover list on the left side.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/activeRovers.png "Active rovers")

Select a rover to view its sensor outputs. 

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/roverSensorOutputs.png "Rover sensor outputs")

There are four sensor display frames:

The camera output. This is a rover eye's view of the world.

The ultrasound output is shown as three white rays, one for each ultrasound. The length of the rays indicates the distance to any objects in front of the ultrasound. The distance in meters is displayed in text below these rays. The maximum distance reported by the ultrasounds is 3 meters.  

The IMU sensor display consists of a cube where the red face is the bottom of the rover, the blue face is the top of the rover, and the red and blue bars are the front and back of the rover. The cube is viewed from the top down. The cube is positioned according to the IMU orientation data. For example, if the rover flips over, the red side will be closest to the observer. Accelerometer data is shown as a 3D vector projected into 2D space pointing towards the sum of the accelerations in 3D space.
 
The map view shows the path taken by the currently selected rover. Green is the encoder position data. In simulation, the encoder position data comes from the odometry topic being published by Gazebo's skid steer controller plugin. In the real robots, it is the encoder output. GPS points are shown as red dots. The EKF is the output of an extended Kalman filter which fuses data from the IMU, GPS, and encoder sensors.

Click on the "Task Status" tab.

![Alt text](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/development/readmeImages/taskStatusTab.png "Task Status tab")

This tab displays the number of targets detected, the number of targets collected, and the number of obstacle avoidance calls.

To close the simulation and the GUI, click the red exit button in the top left-hand corner.

### Software Documentation

Source code for Swarmathon-ROS can be found in the ```~/rover_workspace/src``` directory. This diretory contains severals subdirectories, each of which contain a single ROS package. Here we present a high-level description of each package.

- ```abridge```: A serial interface between Swarmathon-ROS and the A-Star 32U4 microcontroller onboard the physical robot. In the Swarmathon-ROS simulation, ```abridge``` functionality is supplanted by [gazebo_ros_skid_steer_drive](http://docs.ros.org/indigo/api/gazebo_plugins/html/classgazebo_1_1GazeboRosSkidSteerDrive.html) (motor and encoders) and [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) (sonar and IMU; see [step 3](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#3-install-additional-gazebo-plugins) of the Quick Start guide).
- ```mobility```: The top-level controller class for the physical and simulated robots. This package receives messages on the status of targets and/or obstacles in front of the robot and autonomously makes decisions based on these factors. This packages also receives pose updates from [robot_localization](http://wiki.ros.org/robot_localization) (see [step 2](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#2-install-additional-ros-plugins) of the Quick Start guide) and commands from [joystick_drivers](http://wiki.ros.org/joystick_drivers) (see [step 3](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#3-install-additional-gazebo-plugins) of the Quick Start guide).
- ```obstacle_detection```: A logic processor that converts sonar signals into a ternary collision value. This package receives sonar messages from ```abridge``` (for physical robots) or [hector_gazebo_ros_sonar](http://wiki.ros.org/hector_gazebo_plugins) (for simulated robots), and returns a status message to differentiate between three possible cases:
  1. No collision
  2. A collision on the right side of the robot
  3. A collision in front or on the left side of the robot
- ```rqt_rover_gui```: A Qt-based graphical interface for the physical and simulated robots. See [How to use Qt Creator](https://github.com/BCLab-UNM/Swarmathon-ROS/blob/master/README.md#how-to-use-qt-creator-to-edit-the-simulation-gui) for details on this package.
- ```target_detection```: An image processor that detects [AprilTag](https://april.eecs.umich.edu/wiki/index.php/AprilTags) fiducial markers in the onboard camera's video stream. This package receives images from the ```usbCamera``` class (for physical robots) or [gazebo_ros_camera](http://docs.ros.org/indigo/api/gazebo_plugins/html/classgazebo_1_1GazeboRosCamera.html) (for simulated robots), and, if an AprilTag is detected in the image, returns the integer value encoded in the tag.
- ```ublox```: A serial interface to the ublox GPS receiver onboard the physical robot. This package is installed as a git submodule in the Swarmathon-ROS repo. See the [ublox ROS wiki page](http://wiki.ros.org/ublox) for more information.

### How to use Qt Creator to edit the simulation GUI

1. Install Qt Creator:

  ```
  sudo apt-get install qtcreator
  ```
  
2. Build the workspace:

  ```
  catkin build
  ```

3. Run Qt Creator:
  ```
  qtcreator &
  ```

4. Choose "Open File or Project" from the File menu

5. Navigate to ```~/rover_workspace/src/rqt_rover_gui/```

6. Select CMakeLists.txt

7. Click "Open" to continue.

8. Enter ```path to your home directory /rover_workspace/build``` in the text box, this is the default build path. You cannot use the ~ as a shorthand to your home directory here.

9. Click Configure Project

10. Click on the Projects icon on the left toolbar

11. Enter ```-DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel``` in the CMake arguments text box

12. Click the "Edit" toolbox icon on the left

13. Double-click CMakeLists.txt

14. Click the "Build Now" button to build the project

Qt Creator can now be used to build the rover_workspace

Note: start qtcreator in your terminal with rover_workspace as the current directory. Source the ~/.bashrc if the catkin environment variables are not set so that QT Creator can properly build the project.

### Debugging with GDB and Qt Creator

Debuggers are particularly useful for tracking down segfaults and for tracing through the logic of your programs. In order to use the GNU debugger (GDB) with the swarmathon competition add the following line to the CMakelists.txt file for the project you want to debug.

```set(CMAKE_BUILD_TYPE Debug)```

This will compile your code with debugging symbols enabled.

Since ROS is multithreaded you may need to attach the debugger to threads that have been spawned by your program. To enable this enter the following in a terminal:

```sudo apt-get install libcap2-bin```

```sudo setcap cap_sys_ptrace=eip /usr/bin/gdb```

To use QT Creator to debug your already running program click the "Debug" menu. Choose "Start Debugging" and then "Attach to Running Application...". You will be able to use the graphical interface to GDB from here. 
