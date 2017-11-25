#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
# x,y,z and oreintations
from nav_msgs.msg import Odometry

# The velocity command message
from geometry_msgs.msg import Twist

class GuidanceControl: 

    # global pi
    pi = math.pi

    # class costructor; it defines what variables will be stored by the class
    def __init__(self):
        print "Creating GuidanceControl object..."
        rospy.init_node('guidance controls') #ros node housekeeping

        # Subscribers and Publishers
        self.scan_sub_ = rospy.Subscriber('base_scan', LaserScan, self.scan_callback) 
            # subsrcribe to base camp topicc
            # trailing underscore says 'Don't mess with this - intenal only
        
        self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_callback)
            # subscribe to odom topic

        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            # publish to cmd_vel topic

        # Member Varibles
        self.scan_received = False
        self.odom_received = False
        # Robot.control variables
        self.distThreshold = 2.0    # obstable avoidance threshold
        self.scaleVel = 1.0         # magnitude of obstacle avoidance and goal seeking velocities

        # Get goal x and y locations from the launch file
        self.goalX = rospy.get_param('robot_control/goalX', 0.0)
        self.goalY = rospy.get_param('robot_control/goalY', 0.0)

    # END constructor function


    # Callback function triggered whenever a base_scan message is received
    def scan_callback(self, msg):
        
        # find laser scanner properties (min/max scan angles, scan angle increment)
        self.maxAngle = msg.angle_max
        self.minAngle = msg.angle_min
        self.angleIncrement = msg.angle_increment
        
        # find current laser angle, max scan length,
        # distance array for all scans, and number of laser scans
        self.currentLaserTheta = self.minAngle
        self.maxScanLength = msg.range_max
        self.distanceArray = msg.ranges
        self.numScans = len(self.distanceArray)

        # acknowledge that a scan has been received
        # and attempt to compute a new control command
        self.scan_received = True
        self.compute_cmd_vel() # equivalent to HW2 implementation

    # END scan_callback function


    # Callback function triggered whenever an odom message is reveived
    def odom_callback(self, msg):

        # find current location (x,y) based on odometry
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y

        # find current orientation of robot based on odometry
        xOr = msg.pose.pose.orientation.x
        yOr = msg.pose.pose.orientation.y
        zOr = msg.pose.pose.orientation.z
        wOr = msg.pose.pose.orientation.w

        # find orientation of robot (Euler coordinates)
        (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

        # find current angle of robot (equivalent to yaw)
        # now that you have yaw, the robot's pose is completely
        # defined by (currenX, currentY, currentAngle)
        self.currentAngle = yaw

        # acknowledge that an obometry message has been 
        # received and attempt to compute a new control command
        self.odom_received = True
        self.compute_cmd_vel()

    # END odom_callback function


    # member function to compute a new control command
    # given the latest sensor scan and odometry
    def compute_cmd_vel(self):
        
        # how close to the goal until start slowing down
        goalThreshold = 5.0

        # only compute a new control if both base_scan and odom have been received
        if (self.scan_received and self.odom_received):
            # new Twist message
            command = Twist()
            
            # fill in the fields. Field values are unspecified until
            # they are actually assigned. The Twist message old
            # linear and angular velocities
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            
            # for each laser scan
            turnLeft = False        # boolean check for left turn around maneuver
            turnRight = False       # boolean check for right turn around maneuver
            obsAvoidBearing = 0.0   # heading change to avoid obstacles
            obsAvoidVel = 0.0       # velocity change to avoid obstacles

            for curScan in range(0, self.numScans):
                
                if self.distanceArray[curScan] < self.distThreshold:
                    
                    if self.currentLaserTheta <= -self.pi/2.0 and self.currentLaserTheta <= 0.0:
                        
                        # obstacle detected on the right quadrant
                        if not turnLeft:     # has not applied turn left yet
                            obsAvoidBearing = 1.0       # turn left
                            print 'Left turn maneuver applied'
                            turnLeft = True
                        # END if turnLeft
                    
                    elif self.currentLaserTheta >= 0.0 and self.currentLaserTheta <= self.pi/2:

                        # obstacle detected on left quadrant
                        if not turnRight:   # has not yet applied right turn
                            obsAvoidBearing += -1.0     # turn right
                            print 'Right turn maneuver applied'
                            turnRight = True
                        # END if turnRight
                    # END if currentLaserTheta

                    if self.currentLaserTheta >= -self.pi/6.0 and self.currentLaserTheta <= self.pi/2.0:

                        # obstacle detected in from of 60 degree cone
                        if self.distanceArray[curScan]/self.distThreshold < 1.0 - obsAvoidVel:

                            # slow down aggressively if close, but not as much if far away
                            obsAvoidVel = self.scaleVel*(1.0 - (self.distanceArray[curScan])/self.distThreshold)
                            print 'Slowing Down'
                        
                        # END if distanceArray
                    
                    # END if currentLaserTheta

                # END if distanceArray
                
                # Not sure what this does
                self.currentLaserTheta += self.angleIncrement
            
            # END for curScan in numScans
            
            # based on the motion you want
            # (found using goal location, current location and obstacle info)
            # set the robot motion
            headingToGoal = math.atan2(self.goalY - self.currentY, self.goalX - self.currentX)
            bearing = headingToGoal - self.currentAngle

            # compute distance to goal position
            distToGoal = math.sqrt( math.pow(self.goalY - self.currentY,2) + math.pow(self.goalX - self.currentX,2))
            vel = self.scaleVel


            # goalThreshold = 5.0 at top of function
            if distToGoal < goalThreshold:

                # slow down if you are approaching the goal
                vel = vel*distToGoal/goalThreshold

                if distToGoal < 1.0:
                    #stop if within 1m
                    vel = 0.0
                    print 'Arrived at goal'

                # END if distToGoal

            # END if goalThreshold

            # command velocities
            command.linear.x = 2.5 * (vel - obsAvoidVel)
            command.angular.z = 1.0 * (bearing + obsAvoidBearing)
            self.cmd_vel_pub_.publish(command)

            # reset flags so that we only compute

        # END if scan_received and odom_received 
        else:
            if (not self.scan_received):
                print 'No scan received yet'
            if (not self.odom_received):
                print 'No odom received yet'
            # END if
        # END else

    # END compute_cmv_vel

# END GuidanceControl class


if __name__ == '__main__':

    robot_control = GuidanceControl()
    rospy.spin()

# END main

