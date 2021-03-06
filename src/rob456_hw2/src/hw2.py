#!/usr/bin/env python

import numpy as np
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# the velocity command message
from geometry_msgs.msg import Twist

# instantiate global variables "globalOdom"
globalOdom = Odometry()

# global pi - this may come in handy
pi = math.pi

# global collision avoidance toggle
colAvd = False

# method to control the robot
def callback(scan,odom):
    
    # state variable for collision avoidance
    global colAvd
    
    # the odometry parameter should be global
    global globalOdom
    globalOdom = odom

    # make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # get goal x and y locations from the launch file
    goalX = rospy.get_param('hw2/goalX',0.0)
    goalY = rospy.get_param('hw2/goalY',0.0)

    # find current (x,y) position of robot based on odometry
    currentX = globalOdom.pose.pose.position.x
    currentY = globalOdom.pose.pose.position.y

    # find current orientation of robot based on odometry (quaternion coordinates)
    xOr = globalOdom.pose.pose.orientation.x
    yOr = globalOdom.pose.pose.orientation.y
    zOr = globalOdom.pose.pose.orientation.z
    wOr = globalOdom.pose.pose.orientation.w
    # find orientation of robot (Euler coordinates)
    (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

    # find currentAngle of robot (equivalent to yaw)
    # now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
    currentAngle = yaw

    # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
    maxAngle = scan.angle_max
    minAngle = scan.angle_min
    angleIncrement = scan.angle_increment

    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    currentLaserTheta = minAngle        # init loop variable
    maxScanLength = scan.range_max      # setting 
    distanceArray = scan.ranges
    numScans = len(distanceArray)

    # the code below (currently commented) shows how 
    # you can print variables to the terminal (may 
    # be useful for debugging)

    print 'x: {0}'.format(currentX)
    print 'y: {0}'.format(currentY)
    print 'theta: {0}'.format(currentAngle) # e [-pi, pi]
    for i in range(0, 50):
        print ''

    # for each laser scan
    for curScan in range(0, numScans):
        # curScan (current scan) loops from 0 to 
        # numScans (length of vector containing laser range data)
        # for each laser scan, the angle is currentLaserTheta,
        # and the range is distanceArray[curScan]
        
        # ............................................
        # ..... insert code here which uses...........
        # ..... distanceArray[curScan] and ...........
        # ......... currentLaserTheta.................
        # ............................................
        
        distanceArray[curScan]
        
        
        
        # after you are done using one laser scan, update 
        # the current laser scan angle before the for loop
        # is incremented
        currentLaserTheta = currentLaserTheta + angleIncrement	
    
    # based on the motion you want (found using goal location,
    # current location, and obstacle info), set the robot
    # motion, e.g.:
    
    print 'lengthOfScan: {0}'.format(numScans)
    
    obstacleIndex = np.argmin(distanceArray)

    angleOfObject = ((float(obstacleIndex)/numScans)*(maxAngle - minAngle) + minAngle)
    
    print 'indexOfMinArray: {0}'.format(obstacleIndex)
    print 'scanAtMin: {0}' .format(distanceArray[obstacleIndex])
    print 'angleToObstacle: {0}' .format(angleOfObject)


    dispX = goalX - currentX
    dispY = goalY - currentY
    dispAngle = math.atan2(dispY,dispX)
    print 'dispAngle: {0}'.format(dispAngle)
    
    toleranceAngle = pi/10
    destinationTolerance = 0.5
    angleFromObstacle = pi/2
    bumperToggle = 1 
    bumperTolerance = 1.5

    if( (dispX**2 + dispY**2)**(0.5) > destinationTolerance):
        if( (math.fabs(angleOfObject) < angleFromObstacle) 
                and (distanceArray[obstacleIndex] < bumperTolerance)
                and not colAvd):
        
            colAvd = True
        
            if (angleOfObject > 0):
                command.angular.z = -1.0
            else:
                command.angular.z = 1.0
    
        elif (( math.fabs(dispAngle - currentAngle) > toleranceAngle)
                and not colAvd):
            command.linear.x = 0.0
            if (dispAngle - currentAngle > 0):
                command.angular.z = 1.0
            else:
                command.angular.z = -1.0
        else:
            colAvd = False
            command.angular.z = 0.0
            command.linear.x = 5.0
    
    # command.linear.x = 1.0
    # command.angular.z = 0.0
    pub.publish(command)

# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG)

    # subscribe to laser scan message
    sub = message_filters.Subscriber('base_scan', LaserScan)

    # subscribe to odometry message    
    sub2 = message_filters.Subscriber('odom', Odometry)

    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([sub, sub2], 10)
    ts.registerCallback(callback)

    # publish twist message
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()

