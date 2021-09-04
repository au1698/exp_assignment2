# Behavioral Architecture
The scenario describes a pet robot and a green ball in the arena.  
The human can interact with the robot launching the ball while the robot can have three behaviors: sleep, play, normal.
The ball is defined as a robot with no collision element and zero gravity. This because can be present on the arena or not, depends on human's choice. 
The dog is a wheeled robot with a fixed joint that represents the neck and an actuated joint that represents the head. On top of it is placed a camera. 
At the beginning, the robot is in a sleep state. Everytime the robot is in the sleep state, it reaches the home position and after some time switches to normal state.   
Everytime the robot is in the normal state, it moves randomly and reaches randomly positions. It searches the green ball in the arena and when it sees the the ball switches to "play" state. 
When the state is 'play' the robot reaches follows the ball. After some time, the robot switches randomly to the 'normal' or "sleep" state.  


## ROS Architecture of the System
The system is made by four ros nodes: "human_command.py", "state_machine.py", "go_to_point_robot.py" and "go_to_point_ball.py". 
In the folder “config” there is the descriptionof the parameters of the PID – controller. 

# Ros Messages 

- Twist: expresses robot's or ball's velocity in free space broken into its linear and angular parts.
-  PoseStamped: Robot's pose with reference coordinate frame and timestamp. It expresses the result in the      Action. 
- CompressedImage: image acquired from the camera. 
- Pose: robot's or ball's pose. It's refereed to the goal in the Action. 

# Rqt_graph 
<p align="center"> 
<img src=https://github.com/au1698/exp_assignment2/blob/main/exp_assignment2/Images/arena.png raw=true">
</p>

## human_command
This node until simulates user's launch of the green ball in the arena. 
The green ball action is planned by the Action: Planning.action. 
Ball's position is chosen randomly and can be negative along the z - axis. In this way, the ball can disapper.

## state_machine
This node is a finite state machine composed of three states: PLAY, SLEEP, NORMAL.

SLEEP: the state implements the function "Go_home()" in which is implemented and Action Client that let the robot move to a definite destination "home" which is at point (2,0,0). 
After that, the state machine whitches into "normal" state.   

NORMAL: when the robot is in the normal state implements the function "Move_Normal()". 
It implements an Action Client that let the robot move to random destinations. 
When the robot is slowly reaching the goal position moves it's head in order to detect the ball that can be over or below the floor. Infact, it subscribes to the topic "/robot/camera1/image_raw/compressed" in order to recheive a "CompressedImage" from the camera. 
The subscriber has as callback function: "callback_camera" that converts the RosImage acquired into a CV2 format in order to compute image processing with OpenCv functions. 
The OpenCv library is used to blur the acquired image in order to reduce the noise (at high frequency), to convert the image color's namespace from RGB into HSV, to apply a mask and to detect properties of the object such as radius, center and colors. 
This function when the robot is close to the green ball draws a circle and a centroid on the frame, then it updates the list of tracked points. The robot modifies its angular and linear velocity and in order to allign itself to the ball mantaining the ball always at the center of the image frame.
When it sees the green ball, it switches to the "play" state. 

PLAY: when the robot is in the "play" state, it keeps the track of the ball and stops when the ball stops.    

## go_to_point_robot
This node is robot's controller that implements an Action Server. It subscibes to the topic /odom and publishes on the topic 'cmd_vel'robot's twist and on the '/gazebo/set_link_state' in order to move robot's links. Planar move Ros plug in is used to move the robot or the ball  on the environment.  

## go_to_point_ball 
This node is ball's controller that implements an Action Server.

## Installation
- 'vision_opencv' package 
- 'cv_bridge' package
- 'open_cv' library for Python 
  ```
  pip3 install opencv-python 
  ```

- 'find_object_2d' package

## How to run the code
The first thing to do, after having cloned the repository in the Ros workspace, is to build the package in your workspace with
    ```
    catkin_make
    ```
and activate it with install.sh
    ```
    chmod +x install.sh 
    ./install.sh
    ```

To run the system:
    
    ```
    roslaunch exp_assignment2 gazebo_world.launch
    
    ```


## Working hypoteses
The home position is fixed (2,0,0).  
When the robot is in the 'SLEEP' state, it only reaches the home position and after some time goes to the 'NORMAL' state. 
The ball has no gravity and collision elements. 

## Possible improvements
- The robot moves very slowly.
- Improve robot's URDF.   
- Robot moves its head when the ball stops. 

## Author: 

* Aurora Bertino: bertino.aurora16@gmail.com
