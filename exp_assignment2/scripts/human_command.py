#! /usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import roslib
import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import exp_assignment2.msg
import time 
import random

from exp_assignment2.msg import PlanningAction, PlanningGoal

# Human_command node 

global command   

def  Simulator (): 

     
      command = rospy.get_param('/robot/human_command') 

      print ('the param command is:', command)
      
     

      if (command == True ):  
             # ActionClient
             client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)
             # Waits until the action server has started up and started
             client.wait_for_server()
                  
 
                  
             goal = exp_assignment2.msg.PlanningGoal()
             goal.target_pose.pose.position.x = np.random.randint(0, 8)
             goal.target_pose.pose.position.y = np.random.randint(0, 8)
             goal.target_pose.pose.position.z = random.randrange(-1, 6, 2) 

             print("ball position: ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y ,goal.target_pose.pose.position.z) 
     
             # Sends the goal to the action server.
             client.send_goal(goal)
             

             # Waits for the server to finish performing the action.
             client.wait_for_result()

             # Prints out the result of executing the action
             return client.get_result()

def main(): 
      # Inizialize the node   
      rospy.init_node('human_command', anonymous=True)

      while True:
           
           Simulator ()
           time.sleep(20) # wait some time 
           

if __name__ == '__main__':
      main()