#!/usr/bin/env python

#import relevant libraries
import roslib
import rospy
import math
import message_filters
import time
import random
pi = math.pi

# The geometry_msgs Twist message
from geometry_msgs.msg import Twist

from actionlib_msgs.msg import GoalID

# The move_base result message
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseActionGoal

# The laser scan!
from sensor_msgs.msg import LaserScan

# SLAM occupancy grid
from nav_msgs.msg import OccupancyGrid

# Estimated odometry from SLAM
from nav_msgs.msg import Odometry

globalOdom = Odometry()


# Thresholds for clear and blocked grid cells
thClr = 20
thBlk = 80
wallWidth = 15


# Properties of generic occupancy grid
width = 4000
resolution= 0.05
def twoDto1((row, col)):
    return (width*row)+col
# end twoDto1

def oneDto2(index):
    col = index % width
    return (index-col)/width, col
# end oneDto2



# Returns a downsampled matrix snipped from the occupancy grid
def localMatrix():
    global globalMap
    OG = globalMap.data
    (row, col) = mapOdomToOG()

    size = 20       # Width to sample before downsampling
    stepSize = 1    # Downsample resolution
    height = size*stepSize
    localGrid = []

    # Copy values from occupancy grid
    for r in range(row-height, row+height):
        tempRow = []
        for c in range(col-height, col+height):
            tempRow.append(OG[ twoDto1((r,c))  ])
        # end for column
        localGrid.append(tempRow[::stepSize])   # Downsampling cells
    # end for
    return localGrid[::stepSize]    # Downsample rows
# end localMatrix



# Returns OG entries for four-connected grid
def checkNeighbors(index):
    global globalMap
    OG = globalMap.data
    row, col = oneDto2(index)
    
    north = twoDto1((row-1, col))
    east  = twoDto1((row, col+1))
    south = twoDto1((row+1, col))
    west  = twoDto1((row, col-1))
    
    neighborIndicies = (north, east, south, west)

    occupancies = []
    for i in range(len(neighborIndicies)):
        occupancies.append( OG[ neighborIndicies[i] ] )
    # end for

    return occupancies
# end checkNeighbors



# Reads the global odometry variable and maps to OG coordinates
# Returns (row, col) as duple
def mapOdomToOG():
    global globalOdom
    #colPix = int( globalOdom.pose.pose.position.x/0.05)+2000
    #rowPix = int( globalOdom.pose.pose.position.y/0.05)+2000
    return rcFromOdom( globalOdom.pose.pose.position.y, globalOdom.pose.pose.position.x )
# end mapOdomToOG



# Searches for a frontier in a spiral pattern from parameter location
# and exits based on the condition given
def spiralSearch( (currR, currC), exitCase ):
    
    global globalMap
    currLen = 1
    currDisp = 0
    onFirstEdge = True
    direction = 'N'
    count = 0

    while( count < 1200*1200 ):
        count += 1
        (currR, currC) = moveInDirection(currR, currC, direction)
        
        if currR < 0 or currC < 0 or currR >= 4000 or currC >= 4000:
            break
        globalMap.data[twoDto1((currR, currC))] = 5

        #printLocalMap()
        #print 'Spiral at',currR,currC
        #time.sleep(0.1)

        currDisp += 1

        if currDisp == currLen:
            direction = turnCW(direction)
            currDisp = 0

            if not onFirstEdge:
                currLen += 1
                onFirstEdge = True
            else:
                onFirstEdge = False
            # end if
        # end if

        cellContents = exitCase(currR, currC)

        if cellContents != (-1, -1):
            return cellContents
        # end if
    # end while

    return (random.randint(0,3999), random.randint(0,3999))

# end spiralSearch



def findFrontier(currR, currC):
    global globalMap
    global thClr
    OG = globalMap.data
    index = twoDto1( (currR, currC) )
   
    #print 'FINDING FRONTIER?'
    if OG[index] > thClr:
        #print 'No frontier'
        return (-1, -1)
    # end if - escape case
    
    #print 'maybe frontier...'

    foundFron = False
    for neighbor in checkNeighbors(twoDto1((currR, currC))):
        if neighbor == -1:
            foundFron = True
        # end if
    # end for

    if foundFron:
        print 'Found frontier at', currR, currC
        return (currR, currC)
        return calculateCentroid(currR, currC)
    else:
        return (-1, -1)
    # end if

# end findFrontier



def calculateCentroid(row, col):

    # for now, don't caluclate anything
    #(cenR, cenC) = spiralSearch( (row, col), isWall):

    return (row, col)
# end calculateCentroid



def turnCW(direction):

    
    if direction == 'N':
        direction = 'E'
    elif direction == 'E':
        direction = 'S'
    elif direction == 'S':
        direction = 'W'
    elif direction == 'W':
        direction = 'N'
    # end if

    return direction
# end turnCW



def moveInDirection(row, col, direction):
    
    if direction == 'N':
        row += 1
    elif direction == 'E':
        col += 1
    elif direction == 'S':
        row -= 1
    elif direction == 'W':
        col -= 1
    # end if

    return (row, col)
# end moveInDirection



# Function only makes sense in this scope
def putWallAtWithProb(mapCopy, index, probability):
    (row, col) = oneDto2(index)

    global wallWidth
    for r in range(row-wallWidth, row+wallWidth):
        for c in range(col-wallWidth, col+wallWidth):
            if r >= 0 and c >=0 and r < 4000 and c < 4000:
                index = twoDto1((r,c))
                #print index, probability, mapCopy[index]
                mapCopy[index] = probability
            # end if
        # end for
    # end for
# end putWallAtWithProb



def widenWallsOfMap():
    global globalMap
    global thBlk
    mapCopy = list(globalMap.data)

    for row in range(1400, 2600):
        for col in range(1400, 2600):
            index = twoDto1((row, col))
            if globalMap.data[index] > thBlk:
                putWallAtWithProb(mapCopy, index, globalMap.data[index])
        # end if
    # end for

    globalMap.data = list(mapCopy)
# end widenWallsOfMap



# Clears OG in surrounding cells
def clearLocals():
    global globalMap
    global thBlk
    (currR, currC) = mapOdomToOG()
    global wallWidth
    width = wallWidth - 2
    if width <= 0:
        width = 1;
    for r in range(currR-width, currR+width):
        for c in range(currC-width, currC+width):
            if globalMap.data[twoDto1((r,c))] == -1:
                globalMap.data[twoDto1((r,c))] = thBlk-1
            # end if
        # end for
    # end for

# end clearLocals



def printLocalMap():
    localCells = localMatrix()
    for r in reversed(localCells):
        print ' '.join('%03s' % i for i in r)
# end printLocalMap



def  odomFromRC(row, col):

    locX = (float(col-2000))*0.05
    locY = (float(row-2000))*0.05
    
    return (locX, locY)
# end odomFromRC

def rcFromOdom(locX, locY):
    
    row = int(locY/0.05)+2000
    col = int(locX/0.05)+2000

    return (row, col)
# end rcFromOdom



# Clears all commands
def clearGoals():
    pubClr.publish(GoalID())
# end clearGoals



# Calback to subscribe to move base
def mb_callback(msg):
  
    print ''
    print 'Move Base Update'
    
    # Check if robot has reached its goal
    st = msg.status.status
    if st==2 or st==4 or st==5 or st==6:
        print 'Robot has failed to reach the goal!'
        clearGoals()
    elif st==3:
        print 'Robot has reached the goal!'
    # if st==1 -> Robot on way to goal
    # end if

    newGoal = Twist()
    newGoal.linear.x = random.uniform(-20,20)
    newGoal.linear.y = random.uniform(-20,20)
    pubGoal.publish(newGoal)

# end mb_callback



# Occupancy grid update callback
def og_callback(msg):
    global globalOdom
    currX = globalOdom.pose.pose.position.x
    currY = globalOdom.pose.pose.position.y
    global globalMap
    globalMap = msg
    widenWallsOfMap()
    clearLocals()
    print ''
    print 'Occupancy Grid Updated'
    #print 'Index of current location: ', mapOdomToOG()
    #printLocalMap()

    # Current location from odom to OG
    (currR, currC) = rcFromOdom(currX, currY)


    # Where to go in occupancy grid reference point
    (newGoalR, newGoalC) = spiralSearch( (currR, currC), findFrontier )

    # Convert it to odom coordinates
    (newGoalX, newGoalY) = odomFromRC(newGoalR, newGoalC)

    # Calculate displacement to goal
    destX = newGoalX - currX
    destY = newGoalY - currY
    print 'Going to', destX, destY
    # Publish the goal
    command = Twist()
    command.linear.x = destX
    command.linear.y = destY
    #command.target_pose.pose.position.x = destX
    #command.target_pose.pose.position.y = destY
    #command.target_pose.header.stamp = rospy.Time.now()
    clearGoals()
    pubGoal.publish(command)
# end og_callback



# Odometry update callback
def odom_callback(odom):
    global globalOdom
    globalOdom = odom
# end dm_callback



if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('move_in_square', log_level=rospy.DEBUG)

    # Publish waypoint data to robot
    pubGoal = rospy.Publisher('/base_link_goal', Twist, queue_size=10)

    pubVel = rospy.Publisher('/cmd_vel', Twist)

    pubClr = rospy.Publisher('/cancel', GoalID, queue_size=1)

    #pub4 = rospy.Publisher('/goal', PoseStamped)

    # Subscribe to move_base result
    subMbRes = rospy.Subscriber('/move_base/result', MoveBaseActionResult, mb_callback)

    # subscribe to laser scan message
    subOG = rospy.Subscriber('map', OccupancyGrid, og_callback)


    subOdom = rospy.Subscriber('odom', Odometry, odom_callback)
    
    command = Twist()
   
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    pubVel.publish(command)
    # Turn control back to ROS
    rospy.spin()
# end main
