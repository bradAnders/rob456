#!/usr/bin/env python

#import relevant libraries
import roslib
import rospy
import math
import message_filters
import time
import random
import numpy as np
pi = math.pi

# The geometry_msgs Twist messvage
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped

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

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

globalOdom = Odometry()


# Thresholds for clear and blocked grid cells
thClr = 20
thBlk = 80
wallWidth = int(1.2/0.05)


# Properties of generic occupancy grid
width = 4000
resolution= 0.05
# converts coordinates to location in 1D array
def twoDto1((row, col)):
    return (width*row)+col
# end twoDto1

# converts index to equivalent locaiton in 2D representation
def oneDto2(index):
    col = index % width
    return (index-col)/width, col
# end oneDto2



# Returns a downsampled matrix snipped from the occupancy grid
def localMatrix():
    global globalMap
    global ogRef
    OG = globalMap.data
    #(row, col) = mapOdomToOG()
    (row, col) = ogRef

    size = 20       # Width to sample before downsampling
    stepSize = 4    # Downsample resolution
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
    
    # predefine indicies of the four-connected cells
    north = twoDto1((row-1, col))
    east  = twoDto1((row, col+1))
    south = twoDto1((row+1, col))
    west  = twoDto1((row, col-1))
    
    # contents of the cells at locations adjacent to current cell
    neighborIndicies = (north, east, south, west)

    occupancies = []
    for i in range(len(neighborIndicies)):

        # out of bounds check; treat it like a wall
        if neighborIndicies[i] <= 0 or neighborIndicies[i] >= 3999*3999:
            occupancies.append( 100 )
        else:
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
    return rcFromOdom( globalOdom.pose.pose.position.x, globalOdom.pose.pose.position.y )
# end mapOdomToOG



# Searches for a frontier in a spiral pattern from parameter location
# and exits based on the condition given
def spiralSearch( (currR, currC), exitCase ):
    
    global globalMap
    global wallWidth
    currLen = 1
    #currLen = wallWidth-1
    currDisp = 0
    onFirstEdge = True
    direction = 'N'
    count = 0

    #currR -= wallWidth/2
    #currC -= wallWidth/2

    while( count < 1200*1200 ):
        count += 1
        (currR, currC) = moveInDirection((currR, currC), direction)
        
        if currR < 0 or currC < 0 or currR >= 4000 or currC >= 4000:
            break
        #globalMap.data[twoDto1((currR, currC))] = 5

        #if count % 10 == 0:
            #printLocalMap()
            #print 'Spiral at',currR,currC
            #time.sleep(0.15)

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

    # end case when spiral starts searching outside of the world
    print 'Spiral search failed. Trying random goal'
    return (random.randint(1400,2600), random.randint(1400,2600))

# end spiralSearch



# returns the locaiton of a frontier. If no frontier, returns (-1,-1)
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
        #print 'Found frontier at', currR, currC
        #return (currR, currC)

        # Iterates through boundary of frontier and returns a center
        return calculateCentroid(currR, currC)
    else:
        return (-1, -1)
    # end if

# end findFrontier


frontierVector = []
def calculateCentroid(row, col):

    global frontierVector
    global globalMap
    OG = globalMap.data
    frontierVector = []
    currentLocation = (row, col)
    
    #print OG[twoDto1(currentLocation)], thClr
    #printLocalMap()
    #if OG[twoDto1(currentLocation)] > thClr:
    #    return (-1, -1)
    #reachable = False
    #neighbors = checkNeighbors(twoDto1(currentLocation))
    #for i in range(len(neighbors)):
    #    #print neighbors[i],thClr
    #    if neighbors[i] < thClr:
    #        reachable = True
        # end if
    # end for

    #if not reachable:
    #    return (-1, -1)
    #print 'Reachable'


    #print 'RT protocol'

    # Hugs wall in counte-clockwise direction to trace outline of frontier
    # Returns when falls into a loop or hits a wall
    endLocation = turnRightUntilWall(currentLocation)
    
    if endLocation == (-1,-1):
        return endLocation
    # end if

    #if endLocation != currentLocation:
        #currentLocation = endLocation
        #print 'LT protocol'
    #    endLocation = turnLeftUntilWall(currentLocation)
    #else:
        # found bubble
    
    # Only care about frontiers above a certain size
    if len(frontierVector) < 25:
        return (-1, -1)
    else:
        #centroid = moveAwayFromWall(centroid)

        # average the coordinates to calculate centroid
        centroid = averageValues(frontierVector)
        print 'Found frontier of length',len(frontierVector)

        # Show the frontier in Rviz
        publishVector(frontierVector)
        return centroid
# end calculateCentroid



# Takes an array of coordinates and converts it to array of markers
# Markers are printed to RViz with unique IDs. When ID is overridded by next frontier,
# that marker disappears from RViz
def publishVector(vector):
    global ogRef 
    (row, col) = ogRef
    (currX, currY) = odomFromRC(row, col)

    markerArray = MarkerArray()
    for i in range(len(vector)):
        cell = vector[i]
        (row,col) = cell
        (locX, locY) = odomFromRC(row,col)
        marker = Marker()
        marker.id = i
        marker.header.frame_id = "/map"
        #marker.header.frame_id = "/base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        
        # slightly transparent
        marker.color.a = 0.75
        
        # purple
        marker.color.r = 0.75
        marker.color.b = 0.75
        
        # pose of marker
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = locX #- currX
        marker.pose.position.y = locY #- currY
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        #print 'Added marker',locX,locY
    # end for
    #print markerArray
    pubMrk.publish(markerArray)

# end publishVector



# Returns an average location of coordinates
def averageValues(vector):
    rowSum = 0
    colSum = 0
    
    for i in range(len(vector)):
        (row, col) = vector[i]
        rowSum += row
        colSum += col
    # end if

    return (rowSum/len(vector), colSum/len(vector))
# end averageValues



# Hugs the wall to trace a frontier
def turnRightUntilWall(currentLocation):
    global frontierVector
    start = currentLocation
    neighbors = checkNeighbors(twoDto1(currentLocation))
    direction = -1
    foundWallOrReturned = False
    
    # Point towards a wall
    for i in range(len(neighbors)):
        if neighbors[i] == -1:
            direction = i
            break
        elif neighbors[i] > thBlk:
            # found wall
            currentLocation = moveInDirection(currentLocation,i)
            #print 'Found wall'
            return currentLocation
        # end if
    # end for
    
    # if no walls, error; return exit case
    if direction == -1:
        return (-1, -1)
    # end if
    
    #print 'Searching along wall...'
    # Find a wall or loop
    whileCount = 0
    while not foundWallOrReturned:
        wallCount = 0
        # Turn until not facing a wall
        while neighbors[direction] == -1:
            wallCount += 1
            if wallCount > 4:
                #print 'Turned in circle'
                #print neighbors
                return (-1,-1)
            # end if
            direction = charToIndex(turnCCW(indexToChar(direction)))
        # end while
        if neighbors[direction] > thBlk:
            currentLocation = moveInDirection(currentLocation,i)
            #print 'Found wall'
            return currentLocation
        # end if
        currentLocation = moveInDirection(currentLocation, indexToChar(direction))
        neighbors = checkNeighbors(twoDto1(currentLocation))
        direction = charToIndex(turnCW(indexToChar(direction)))
        for cell in frontierVector:
            if cell == currentLocation:
                #print 'Went in loop'
                #print 'Found loop going right'
                return start
        frontierVector.append(currentLocation)
        #print 'Count', whileCount, 'at',currentLocation, 'start',start
        whileCount += 1
    # end while
# end turnRightUntilWall



# hugs the wall to trace a frontier
def turnLeftUntilWall(currentLocation):
    global frontierVector
    start = currentLocation
    neighbors = checkNeighbors(twoDto1(currentLocation))
    direction = -1
    foundWallOrReturned = False
    #print 'Turning Left' 
    # Point towards a wall
    for i in range(len(neighbors)):
        if neighbors[i] == -1:
            direction = i
            break
        elif neighbors[i] > thBlk:
            # found wall
            currentLocation = moveInDirection(currentLocation,i)
            return currentLocation
        # end if
    # end for
    
    # if no walls, error; return exit case
    if direction == -1:
        return (-1, -1)
    # end if

    # Find a wall or loop
    while not foundWallOrReturned:
        wallCount = 0
        # Turn until not facing a wall
        while neighbors[direction] == -1:
            wallCount += 1
            if wallCount > 4:
                return (-1,-1)
            # end if
            direction = charToIndex(turnCW(indexToChar(direction)))
        # end while
        if neighbors[direction] > thBlk:
            currentLocation = moveInDirection(currentLocation,indexToChar(direction))
            return currentLocation
        # end if
        currentLocation = moveInDirection(currentLocation, indexToChar(direction))
        neighbors = checkNeighbors(twoDto1(currentLocation))
        direction = charToIndex(turnCCW(indexToChar(direction)))
        for cell in frontierVector:
            if cell == currentLocation:
                #print 'Found loop going left'
                return start
        frontierVector.append(currentLocation)
        #print 'Added left element'
    # end while
# end turnRightUntilWall



# Converts magnetic direction to array index
def charToIndex(char):
    if char == 'N':
        return 0
    elif char == 'E':
        return 1
    elif char == 'S':
        return 2
    elif char == 'W':
        return 3
    # end if
# end charToIndex
    


# converts array index to magnetic direction
def indexToChar(index):
    chars = ['N', 'E', 'S', 'W']
    return chars[index]
# end indexToChar



# Returns the proper direction with given turn
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



# Returns the proper direction with given turn
def turnCCW(direction):
    if direction == 'N':
        direction = 'W'
    elif direction == 'E':
        direction = 'N'
    elif direction == 'S':
        direction = 'E'
    elif direction == 'W':
        direction = 'S'
    # end if
    return direction
# end turnCW



# moves a 2D coordinate in a mangetic direction
def moveInDirection((row, col), direction):
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



# Prints a buffer around the walls so that waypoints are not set too close to wall
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
        width = 1
    # end if
    for r in range(currR-width, currR+width):
        for c in range(currC-width, currC+width):
            if globalMap.data[twoDto1((r,c))] == -1:
                globalMap.data[twoDto1((r,c))] = 1
            # end if
        # end for
    # end for

# end clearLocals



# Prints a snapshot of the local occupancy grid
def printLocalMap():
    localCells = localMatrix()
    for r in reversed(localCells):
        print ' '.join('%03s' % i for i in r)
    # end for
# end printLocalMap



# Converts matrix indicies to coordinate
def odomFromRC(row, col):
    locX = (float(col-2000))*0.05
    locY = (float(row-2000))*0.05
    return (locX, locY)
# end odomFromRC

# converts coordinate to matrix indicies
def rcFromOdom(locX, locY):
    row = int(locY/0.05)+2000
    col = int(locX/0.05)+2000
    return (row, col)
# end rcFromOdom



# Clears all commands
def clearGoals():
    pubClr.publish(GoalID())
# end clearGoals



# Creates a bnew goal close to the currect location to attempt
# to get away from a wall
def newRandomGoal():
    newGoal = Twist()
    global globalOdom
    position = globalOdom.pose.pose.position
    dispX = random.uniform(-2,2)
    dispY = random.uniform(-2,2)
    newGoal.linear.x = position.x + dispX
    newGoal.linear.y = position.y + dispY
    command.angular.z = math.degrees(math.atan2(dispY,dispX))
    pubGoal.publish(newGoal)
    print 'New Pseudo-Random Goal:',newGoal.linear.x,newGoal.linear.y
# end newRandomGoal



# Callback to subscribe to current goal
def goal_callback(msg):
    print ''
    print 'Recieved new goal'
    print msg
# end goal_callback



# Callback to subscribe to move base
def mb_callback(msg):
  
    print ''
    print 'Move Base Update'
    
    # Check if robot has reached its goal
    st = msg.status.status
    if st==2 or st==4 or st==5 or st==6:
        print 'Robot has failed to reach the goal!'
        #clearGoals()
        #newRandomGoal()
        print msg
    elif st==3:
        #newRandomGoal()
        print 'Robot has reached the goal!'
    else:
        print 'Other status message'
    # if st==1 -> Robot on way to goal
    # end if

# end mb_callback


# Occupancy grid update callback
#def og_mb_callback(mBase,oGrid):
def og_callback(oGrid):
    #global mBaseRecord
    #mBaseRecord = mBase
    
    # Check if robot has reached its goal
    #st = mBase.status.status
    #if not st == 3:
    #    print 'Robot has failed to reach the goal!'
    #else:
    #    print 'Robot has reached the goal!'
    #    clearGoals()


    global globalOdom
    currX = globalOdom.pose.pose.position.x
    currY = globalOdom.pose.pose.position.y
    
    global ogRef
    global globalMap
    globalMap = oGrid
    widenWallsOfMap()
    clearLocals()
    print ''
    print 'Occupancy Grid Updated'

    # Current location from odom to OG
    (currR, currC) = rcFromOdom(currX, currY)

    ogRef = (currR, currC)
    
    # Where to go in occupancy grid reference point
    (newGoalR, newGoalC) = spiralSearch( (currR, currC), findFrontier )

    # Convert it to odom coordinates
    (newGoalX, newGoalY) = odomFromRC(newGoalR, newGoalC)

    # Calculate displacement to goal
    destX = newGoalX + random.uniform(-1,1)
    destY = newGoalY + random.uniform(-1,1)
    dispX = destX #- currX
    dispY = destY #- currY
    print 'At x',currX,', y',currY
    print 'Going to x', dispX,', y', dispY
    
    # Publish the goal
    command = Twist()
    command.linear.x = dispX #+ random.uniform(-2, 2)
    command.linear.y = dispY #+ random.uniform(-2, 2)
    command.angular.z = math.degrees(math.atan2(dispY,dispX))
    pubGoal.publish(command)

# end og_callback


thTime = 30
stuck = False
reallyStuck = False
# Odometry update callback
def odom_callback(odom):
    global globalOdom
    global thTime
    global stuckTime
    global reallyStuckTime
    global stuck
    global reallyStuck
    now = time.clock()
    if globalOdom.pose.pose == odom.pose.pose:
        if reallyStuck and now - reallyStuckTime > 2:
            print 'Trying harder to unstick'
            reallyStuckTime = now
            clearGoals()
            newRandomGoal()
        else:
        
            if not stuck:
                stuck = True
                stuckTime = now
            elif now-stuckTime > thTime and stuck:
                print 'Stuck for',thTime,'s'
                stuckTime = now
                global globalMap
                global mBaseRecod
                #og_callback(globalMap)
                #og_mb_callback(mBaseRecord,globalMap)
                clearGoals()
                newRandomGoal()
                reallyStuck = True
                reallyStuckTime = now
            # end if
    else:
        stuckTime = now
        reallyStuckTime = now
        stuck = False
        reallyStuck = False
    globalOdom = odom
# end dm_callback



if __name__ == "__main__":
    
    # Initialize the node
    rospy.init_node('move_in_square', log_level=rospy.DEBUG)
    
    #       -- Publishers --
    # Publish waypoint data to robot
    pubGoal = rospy.Publisher('map_goal', Twist, queue_size=1, latch=True)

    pubClr = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    pubMrk = rospy.Publisher('/visualization_marker', MarkerArray, queue_size=10000)

    #       -- Subscribers --
    # Subscribe to move_base result
    #subMb = message_filters.Subscriber('/move_base/result', MoveBaseActionResult)
    subMb = rospy.Subscriber('/move_base/result', MoveBaseActionResult, mb_callback)

    # subscribe to laser scan message
    #subOG = message_filters.Subscriber('map', OccupancyGrid)
    subOG = rospy.Subscriber('map', OccupancyGrid, og_callback)

    # get latest estimate of current location
    subOdom = rospy.Subscriber('odom', Odometry, odom_callback)

    #ts = message_filters.ApproximateTimeSynchronizer([subMb, subOG], queue_size=10, slop=10)
    #ts.registerCallback(og_mb_callback)

    # Publish initial zero velocity to init
    command = Twist()
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0
    pubVel.publish(command)
    rospy.sleep(2)

    newRandomGoal()

    # Turn control back to ROS
    rospy.spin()
# end main
