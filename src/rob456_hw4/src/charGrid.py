# File:           ROB 456, Assignment 4, Helper Functions
# Author:         Bradley Anderson
# Collaborators:  Kenzie Brian and Kyle O'Brien
# Date:           November 7, 2017

import numpy as np
import csv
import time



# Class to define a graph node
class graphNode(object):
    # Pointers to connected edges
    #              North East  South West
    connections = [None, None, None, None]

    heur=-1
    parent=None
    parentDir = -1;
    location = np.array([0,0])

    def __init__(self, parent, loc):
        self.parent = parent
        self.location = loc
    # end init

    def setEdges(self, edgeList):
        #self.connections = copy.deepcopy(edgeList)
        self.connections = edgeList
    # end setEdges

    def setEdge(self, edge, pointer):
        self.connections[edge] = pointer
    # end setEdge

    def setParent(self, parent, direction):
        self.parent = parent
        if parent != None:
            self.parentDir = direction
        #end if
    # end setParent

    def setParentDir(self, direction):
        self.parentDir = direction
    # end setParentDir

    def setHeur(self, heur):
        self.heur = heur
    # end setHeur

    def getLoc(self):
        return self.location
    # end getLoc

    def getParentDir(self):
        return self.parentDir
    # end getParentDir

    def getParent(self):
        return self.parent
    # end getParent

    def getHeur(self):
        return self.heur
    # end getHeur

    def getEdge(self, edge):
        return self.connections[edge]
    # end getEdge

    def getEdges(self):
        return self.connections
    # end getEdges

    def getIndex(self):
        return self.location[0]*20 + self.location[1]
    # end getIndex

# end graphNode



# Prints a number of blank lines to terminal
def printBlank(number):
    for i in range(0, number):
        print
    # end if
# end printBlank



# Prints a header at the top of the script output
def printWelcome():
    printBlank(50)
    print 'Running File: ROB 456, Assignment 4'
    printBlank(2)
# end printWelcome



# Reset traversal flags for recursion
def clearTravFlags(cellMap):
    for row in range(len(cellMap)):
        for col in range(len(cellMap[row])):
            if cellMap[row][col][0] == 2:
                cellMap[row][col][0] = 0;
            # end if
        # end for
    # end for
# end clearTravFlags



# Function to print a graphNode info for debugging
def printNodeInfo(node):
    edges = [None, None, None, None]
    count=0;
    if node != None:
        for edge in node.getEdges():
            if edge != None:
                edges[count] = edge.getIndex()
            # end if
            count = count+1
        # end for
        print 'Node at ', node.getLoc()
        print 'Connections: ', edges
        print 'Heuristic: ', node.getHeur()
        if node.getParent() != None:
            #print node.getParent()
            print 'Parent: ', node.getParent().getIndex()
            print 'Parent direction: ', node.getParentDir()
        else:
            print 'No Parent'
        # end if
    else:
        print 'Node empty'
    # end if
    printBlank(1)
# end printNodeInfo



# Searches the connections at a point and creates node objects for cells that
# have not yet been discovered
def getNodes(location, cellMap):
    
    #       North  East  South West
    nodes = [None, None, None, None]
    
    # Check each diection as encoded above 
    for i in range(0, 4):

        row = location[0]
        col = location[1]

        if i==0:   # North
            row = row-1
        elif i==1: # East
            col = col+1
        elif i==2: # South
            row = row+1
        elif i==3: # West
            col = col-1
        # end if


        # Check index for out-of-bounds
        if (row>=0 and col>=0 and row<cellMap.shape[0] and col<cellMap.shape[1]):

            # Check if cell is unobstructed and undiscovered
            if (cellMap[row, col, 0] == 0):

                # Discovery flag
                cellMap[row, col, 0] = 2; # Add discovered flag

                # Create new graphNode object and store it
                nodes[i] = graphNode(None, [row, col])
                cellMap[row, col, 1] = nodes[i]

            # Cell has been discovered
            # Don't make a new cell, return ptr to existing node
            elif (cellMap[row, col, 0] == 2):
                nodes[i] = cellMap[row, col, 1]
            # end if
        # end if
    # end for

    return nodes
# end getNodes



# Reads a csv file and creates a matrix of tuples matching the size of the csv
# Tuples are structured as follows: (mapDigit, pointer to graphNode object)
def readCSVFromPath(path, permissions, delim):
    
    # Read the CSV into the program memory
    with open(path, permissions) as csvfile:
        mtx = list(csv.reader(csvfile, delimiter=delim))
    
    # Convert list of lists of characters to
    # array of arrays of integers
    for row in range(len(mtx)):
        for cell in range(len(mtx[row])):
            # Convert characters to (integer, node-pointer) tuple
            mtx[row][cell] = [int(mtx[row][cell]), None]
        # end for - cols in row

        # Convert list to array   
        mtx[row] = np.array(mtx[row])
    # end for - rows in mtx

    # Convert list of arrays to array of arrays
    mtx = np.array(mtx)

    return mtx
# end readCSV from Path



# Convert cellMap to complete graph
# Assumes head is at [0,0]
def generateGraphFromMap(cellMap):

    # Cycle by row
    for row in range(0, len(cellMap)):
    #for row in range(0, 3):

        # Cycle by column in row
        for cell in range(0, len(cellMap[row])):

            # map[cell] == 1 -> cell unreachable
            if cellMap[row, cell, 0] != 1:
                
                # map[cell] == 0 -> cell undiscovered; mak
                if cellMap[row, cell, 0] == 0:
                    cellMap[row, cell, 0] = 2
                    #print 'Creating isolated node at ', row, ',',cell
                    cellMap[row, cell, 1] = graphNode(None, [row, cell])
                # end if - undiscovered

                cellMap[row, cell, 1].setEdges(getNodes([row, cell], cellMap))
            # end if - reachable
        # end for - cols in row
    # end for - rows in mtx
# end generateGraphFromMap



# Estimate a heuristic based off manhattan distance to goal
def estHeurForCell(location, goal):
    return abs(location[0]-goal[0]) + abs(location[1]-goal[1])
# end estHeur



# Calculates all heristics
def estimateHeuristicForGraph(cellMap, goal):
    
    #clearTravFlags(cellMap)
    
    for row in range(len(cellMap)):
        for cell in range(len(cellMap[row])):
            node = cellMap[row][cell][1]
            if node != None: node.setHeur(estHeurForCell(node.getLoc(),goal))
        # end for
    # end for

# end estimateHeuristicForGraph
