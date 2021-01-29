#!/usr/bin/env python
## IHSAN SOYDEMIR - 150180702
## ALPER MERIC - 150160714
## Robot behaves as a wall follower, this can be considered as bonus :)
## Also, robot tries to maintain a certain distance (h in code) with wall 
## Note: Could you please give it one more chance if robot gets stuck? Gazebo is buggy...

## Common ROS headers.
import rospy
## Required for some printing options
import sys

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid
## Required for mathematical functions
import numpy as np
## Required for storing mid_laser
import queue
import time, math
## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##


# All possible actions for robot
class Actions:
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    FORWARD_LEFT = 5
    FORWARD_RIGHT = 6
    SLIGHTLY_LEFT = 7
    SLIGHTLY_RIGHT = 8


def move_robot(action):
    global motor_command_publisher
    motor_command = Twist()
    if action == Actions.FORWARD:
        motor_command.linear.x = 0.5
        motor_command.angular.z = 0.0
    elif action == Actions.BACKWARD:
        motor_command.linear.x = -1
        motor_command.angular.z = 3
    elif action == Actions.TURN_LEFT:
        motor_command.linear.x = 0.0
        motor_command.angular.z = 1.0
    elif action == Actions.TURN_RIGHT:
        motor_command.linear.x = 0.0
        motor_command.angular.z = -1.0
    elif action == Actions.FORWARD_RIGHT:
        motor_command.linear.x = 0.5
        motor_command.angular.z = -0.5
    elif action == Actions.FORWARD_LEFT:
        motor_command.linear.x = 0.33
        motor_command.angular.z = 0.33
    elif action == Actions.SLIGHTLY_LEFT:
        motor_command.linear.x = 0.25
        motor_command.angular.z = 0.1
    elif action == Actions.SLIGHTLY_RIGHT:
        motor_command.linear.x = 0.25
        motor_command.angular.z = -0.1
    else: # Stop if action not defined
        motor_command.linear.x = 0.0
        motor_command.angular.z = 0.0
    motor_command_publisher.publish(motor_command)
    time.sleep(0.005)
    
wall_follower = False
door_search = False
tmp = 0
def laser_callback(data):
    global motor_command_publisher
    global door_search
    global wall_follower
    global tmp
    ## Lets fill a twist message for motor command 
    motor_command = Twist()
    # Calculate total number of laser points
    len_laser = len(data.ranges) 
    midpoint = len_laser/2
    laser_numpy = np.asarray(data.ranges)
    # Create an array with nan values of laser points
    nan_numpy = laser_numpy[np.isnan(laser_numpy)]
    nan_ctr = len(nan_numpy)
    range_min = data.range_max
    range_max = data.range_min
    # h is the value that maintains the closest distance to the wall
    # h is calculated using basic trigonometry, 1.047 is 60 degrees 
    h = np.cos(1.047) * laser_numpy[0]
    left_sum = 0
    right_sum = 0
    for i in range(len(laser_numpy)):
        range_min = min(range_min, laser_numpy[i])
        # Calculate max for the right most quarter of laser array which corresponds to 15 degrees
        if i < 160:
            range_max = max(range_max, laser_numpy[i])
        # Split laser array and calculate the sum of left and right halves
        if i > midpoint:
            left_sum += laser_numpy[i] 
        else:
            right_sum += laser_numpy[i]
    # Check if 630 or more of the values in the laser array are empty to detect an accident, 
    # because nan values are also returned when the robot is too close to the wall. 
    # It also averages the 0.9375 (0.09375*10) degree part in the middle of the laser array and checks if it is greater than 0.25.
    if np.mean(laser_numpy[midpoint-5:midpoint+6]) < 0.25 or nan_ctr > 630:
        move_robot(Actions.BACKWARD)
    else:
        diff = h - tmp
        if range_min <= 0.5 and not door_search:
            wall_follower = True
            move_robot(0) # Needed for stability
            # Required for turning
            if left_sum >= right_sum:
                tmp = h
                move_robot(Actions.TURN_RIGHT)
            else:
                move_robot(Actions.TURN_LEFT)
        else:
            move_robot(0)
            if wall_follower:
                # Here we find some door candidates
                if range_max >= 1.85:
                    door_search = True
                    wall_follower = False
            if door_search:
                # Check rightest 3 elements mean
                right_most_mean = np.mean(laser_numpy[:3])
                if right_most_mean <= 0.5:
                    door_search = False
                else: # We make sure that it is actually a door
                    move_robot(Actions.FORWARD_RIGHT)
            else:
                # Follow wall while keeping a certain distance(h) with door
                if diff > 0.10 and diff < 0.20:
                    move_robot(Actions.SLIGHTLY_LEFT)
                elif diff > 0.25 and diff < 0.75:
                    move_robot(Actions.SLIGHTLY_RIGHT)
                else:
                    move_robot(Actions.FORWARD)
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()