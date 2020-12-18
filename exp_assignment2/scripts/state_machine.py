#! /usr/bin/python
# -*- coding: utf-8 -*-

# Python libs
import sys
import time
import math
import random

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import smach
import smach_ros
import actionlib

import exp_assignment2.msg

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
from gazebo_msgs.msg import LinkState
from tf import transformations
from std_msgs.msg import String, Float64 

from exp_assignment2.msg import PlanningAction, PlanningGoal

# topic where we publish (PUBLISHERS)
image_pub = rospy.Publisher("/output/image_raw/compressed",CompressedImage, queue_size=1)
vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

see_ball = bool
move_head = bool 
rotate_camera = Float64() 

VERBOSE = False     

# Function that describes that the robot goes home (Action-Client)
def Go_home():
    # ActionClient that moves the robot
     client = actionlib.SimpleActionClient('/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

     client.wait_for_server()

     # Creates a goal to send to the action server.
     goal = exp_assignment2.msg.PlanningGoal()
     # Go to home position 
     goal.target_pose.pose.position.x = 2 
     goal.target_pose.pose.position.y = 0
     goal.target_pose.pose.position.z = 0

     # Sends the goal to the action server.
     client.send_goal(goal)

     # Waits for the server to finish performing the action.
     wait = client.wait_for_result() 
     
     # If the result doesn't arrive, the server is not avaible 
     if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
     else:
        result = client.get_result()   # when arrive the result = robot arrived to goal 
        return result 

def Move_normal():
     
    # ActionClient that moves the robot
     client = actionlib.SimpleActionClient('/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

     client.wait_for_server()

     # Creates a goal to send to the action server.
     goal = exp_assignment2.msg.PlanningGoal()
     # Go to home position 
     goal.target_pose.pose.position.x = np.random.randint(1, 8)
     goal.target_pose.pose.position.y = np.random.randint(1, 8)
     goal.target_pose.pose.position.z = 0

     # Sends the goal to the action server.
     client.send_goal(goal)

     subscribe_camera = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                                CompressedImage, callback_camera,  queue_size=1)

     # Waits for the server to finish performing the action.
     wait = client.wait_for_result() 
     
     # If the result doesn't arrive, the server is not avaible 
     if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
     else:
        result = client.get_result()   # when arrive the result = robot arrived to the goal 
        return result 

# Robot follows the ball 
class image_feature:
    global move_head 

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        
       # topics where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        orangeLower = (50, 50, 20)   # range to detect green ball 
        orangeUpper = (70, 255, 255) 

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, orangeLower, orangeUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()

                ball_param = rospy.get_param('/ball_param')

                while (ball_param == True):  # enter in the loop when the ball stops 
                
                     vel.linear.x = 0
                     move_head = True        # set move head True
                     time.sleep(10)         
                 
                else: 
                     
                     vel.angular.z = 0.002*(center[0]-400) 
                     vel.linear.x = -0.01*(radius-100)
                     self.vel_pub.publish(vel)
            else:
                vel = Twist()
                vel.linear.x = 0.5
                self.vel_pub.publish(vel)

        else:
            vel = Twist()
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)

        move_head = False 
        cv2.imshow('window', image_np)
        cv2.waitKey(2)   

def callback_camera(ros_data):
     
     global see_ball 
    
     '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
     if VERBOSE:
         print ('received image of type: "%s"' % ros_data.format)

     #### direct conversion to CV2 ####
     np_arr = np.fromstring(ros_data.data, np.uint8)
     image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

     orangeLower = (50, 50, 20)   # to detect green ball ->  greenLower = (50, 50, 20)
     orangeUpper = (70, 255, 255) # " - > greenUpper = (70, 255, 255) 

     blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
     hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
     mask = cv2.inRange(hsv, orangeLower, orangeUpper)
     mask = cv2.erode(mask, None, iterations=2)
     mask = cv2.dilate(mask, None, iterations=2)
     #cv2.imshow('mask', mask)
     cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
     cnts = imutils.grab_contours(cnts)
     center = None
     # only proceed if at least one contour was found
     if len(cnts) > 0:
         see_ball = True  # robot can see the ball
         # find the largest contour in the mask, then use
         # it to compute the minimum enclosing circle and
         # centroid
         c = max(cnts, key=cv2.contourArea)
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         # only proceed if the radius meets a minimum size
         if radius > 10:
             #see_ball = True  # robot can see the ball   

             # draw the circle and centroid on the frame,
             # then update the list of tracked points
             cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
             cv2.circle(image_np, center, 5, (0, 0, 255), -1)
             vel = Twist()
             vel.angular.z = 0.002*(center[0]-400)  # change sign 
             vel.linear.x = -0.01*(radius-100)
             vel_pub.publish(vel)
            
             #see_ball = True  # robot can see the ball   

         else:
             vel = Twist()
             vel.linear.x = 0.5
             vel_pub.publish(vel)

     else:
             vel = Twist()
             vel.angular.z = 0.5
             vel_pub.publish(vel)
             see_ball = False 

     cv2.imshow('window', image_np)
     cv2.waitKey(2)
    
## Define state Sleep
class Sleep(smach.State):
    ## Constructor of the class Sleep
    def __init__(self):       
    ## Initialization function 
        smach.State.__init__(self, 
                        outcomes=['wake_up'],
                        input_keys=['sleep_counter_in'],
                        output_keys=['sleep_counter_out'])


    def execute(self,userdata):
        
        rospy.loginfo('Executing state SLEEP')
        userdata.sleep_counter_out = userdata.sleep_counter_in + 1  
        # ActionClient that moves the robot 
        
        rospy.set_param('/robot/human_command', True)

        time.sleep(5)
        
        rospy.set_param('/robot/human_command', False) # the ball is not launched 

        if (Go_home()):
             rospy.loginfo('robot arrived at destination') 
             time.sleep(10) # stay at position 10 sec 
             rospy.set_param('human_command', True)       # the ball is launched   

    ## Change state: from 'SLEEP' to 'NORMAL'  
             return 'wake_up'  

## Define state Normal
class Normal(smach.State):
    ## Constructor of the class Normal
    def __init__(self):     
    ## Initialization function 
        smach.State.__init__(self, 
                        outcomes=['go_to_play'],
                        input_keys=['normal_counter_in'],
                        output_keys=['normal_counter_out'])

    def execute(self,userdata):
        time.sleep(5)
        rospy.loginfo('Executing state NORMAL')
        userdata.normal_counter_out = userdata.normal_counter_in + 1  
        

        global see_ball

        while True:   

             rospy.set_param('/robot/human_command', True)       # the ball is launched 
             Move_normal()         # robot moves randomly 
                 
             # Subscribe to camera topic 
             subscribe_camera = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                                CompressedImage, callback_camera,  queue_size=1)
             time.sleep(5)
                

             if (see_ball == True):
                 print('the robot sees the ball') 
                 break  

        return 'go_to_play'  
                                
## Define state Play 
class Play(smach.State):
    time.sleep(5)
    global move_head
    ## Constructor of the class Play
    def __init__(self):       
    ## Initialization function   
        smach.State.__init__(self, 
                         outcomes=['go_to_normal','go_to_sleep'],
                         input_keys=['play_counter_in'],
                         output_keys=['play_counter_out'])

        self.rotate_pub = rospy.Publisher("/robot/joint_position_controller/command",
                                    Float64, queue_size=1) 


    def execute(self,userdata):
        
        rospy.loginfo('Executing state PLAY')
        userdata.play_counter_out = userdata.play_counter_in + 1
        # To track the data  
        time.sleep(5)
        for i in range(1,3):     # play at max 3 times 
             ic = image_feature()
             print('robot is in play state and is following the ball')
             rotate_camera = Float64() 
             rotate_camera.data = 0.0 
             if (move_head == True):
                 rospy.loginfo('The robot moves head')
                # Robot rotates the  camera 
                 while rotate_camera.data < 0.5:                   # robot head turns left
                    rotate_camera.data = rotate_camera.data + 0.1 
                    self.rotate_pub.publish(rotate_camera)
                    time.sleep(3) 
                 time.sleep(1)

                 while rotate_camera.data > 0.01:                 # robot head turns right 
                    rotate_camera.data = rotate_camera.data - 0.1 
                    self.rotate_pub.publish(rotate_camera) 
                    time.sleep(3)

                 time.sleep(1)
             
    ## Change state randomly: from 'PLAY' to 'NORMAL' 
        return random.choice([ 'go_to_normal','go_to_sleep'])
            
def main():    
    ## Inizialize ros node ''pet_state_machine'
    rospy.init_node('state_machine')   
    
    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])   
    sm.userdata.sm_counter = 0
    
    ## Open state machine container
    with sm:
         ## Add states to the container
         smach.StateMachine.add('SLEEP', Sleep(),
                                  
                               transitions={'wake_up':'NORMAL'},
                                                                
                                            
                               remapping={'sleep_counter_in':'sm_counter', 
                                         'sleep_counter_out':'sm_counter'})
         smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_play' :'PLAY'},
                                            
                                            
                               remapping={'normal_counter_in':'sm_counter',
                                          'normal_counter_out':'sm_counter'})

         smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_to_normal':'NORMAL',
                                             'go_to_sleep' : 'SLEEP'}, 
                                            
                                            
                               remapping={'play_counter_in':'sm_counter',
                                          'play_counter_out':'sm_counter'})
                                



    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
     main()