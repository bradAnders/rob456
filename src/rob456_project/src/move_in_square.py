#!/usr/bin/env python

#import relevant libraries
import roslib
import rospy
import math
import message_filters
pi = math.pi

# The geometry_msgs Twist message
from geometry_msgs.msg import Twist

# The move_base result message
from move_base_msgs.msg import MoveBaseActionResult

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

    size = 100      # Width to sample before downsampling
    stepSize = 5    # Downsample resolution
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
    colPix = int( globalOdom.pose.pose.position.x/0.05)+2000
    rowPix = int( globalOdom.pose.pose.position.y/0.05)+2000
    return (rowPix, colPix)
# end mapOdomToOG



# Searches for a frontier in a spiral pattern from parameter location
# and exits based on the condition given
def frontierCentroidFromSpiralSearch( (currR, currC), exitCase ):
    
    currLen = 1
    currDisp = 0
    onFirstEdge = True
    direction = 'N'
    count = 0

    while( count < 4000*4000 ):
        count += 1
        (currR, currC) = moveInDirection(currR, currC, direction)
        currDisp += 1

        if currDisp == currLen:
            direction = turnCW(direction)
            currDisp = 0

            if !onFirstEdge:
                currLen += 1
                onFirstEdge = True
            else:
                onFirstEdge = False
            # end if
        # end if

        frontierCentroid = findFrontier(currR, currC)

        if frontierCentroid != (-1, -1):
            return frontierCentroid
        # end if
    # end while

# end frontierCentroidFromSpiralSearch



def findFrontier(currR, currC):
    global globalMap
    OG = globalMap.data
    index = twoDto1( (currR, curC) )
    
    if OG[index] > thrClr:
        return (-1, -1)
    # end if - escape case

    foundFron = False
    for neighbor in checkNeighbors(twoDto1((currR, currC))):
        if neighbor == -1
            foundFron = True
        # end if
    # end for

    if foundFron:
        return calculateCentroid(currR, currC)
    else:
        return (-1, -1)
    # end if

# end findFrontier



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



# Calback to subscribe to
def mb_callback(msg):
  
    # Check if robot has reached its goal
    st = msg.status.status
    if st==2 or st==4 or st==5 or st==6:
        print 'Robot has failed to reach the goal!'
    elif st==3:
        print 'Robot has reached the goal!'
    # if st==1 -> Robot on way to goal
    # end if

    print "mb callback"

# end mb_callback



def og_callback(msg):
    global globalMap
    globalMap = msg
    print ''
    print 'Occupancy Grid Updated'
    print 'Index of current location: ', mapOdomToOG()
    #global globalOdom
    #x = globalOdom.pose.pose.position.x
    #y = globalOdom.pose.pose.position.y
    #row = -y
    #col = x
    print 'Neighbors: ', checkNeighbors(twoDto1(mapOdomToOG()))
    localCells = localMatrix()
    for r in reversed(localCells):
        print ' '.join('%03s' % i for i in r)
# end og_callback



def odom_callback(odom):
    global globalOdom
    globalOdom = odom
    #x = globalOdom.pose.pose.position.x
    #y = globalOdom.pose.pose.position.y
    #print ''
    #print 'Odomety Updated: ', x, y
    #print 'Covariance: ', globalOdom.pose.covariance
# end dm_callback



if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('move_in_square', log_level=rospy.DEBUG)

    # Publish waypoint data to robot
    pub = rospy.Publisher('/base_link_goal', Twist, queue_size=10)

    # Subscribe to move_base result
    sub1 = rospy.Subscriber('/move_base/result', MoveBaseActionResult, mb_callback)

    # subscribe to laser scan message
    sub2 = rospy.Subscriber('map', OccupancyGrid, og_callback)


    sub3 = rospy.Subscriber('odom', Odometry, odom_callback)
    
    # synchronize laser scan and odometry data
    #ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
    #ts.registerCallback(callback) 

    # Turn control back to ROS
    rospy.spin()
# end main
