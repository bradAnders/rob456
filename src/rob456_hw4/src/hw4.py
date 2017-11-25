# File:           ROB 456, Assignment 4
# Author:         Bradley Anderson
# Collaborators:  Kenzie Brian and Kyle O'Brien
# Date:           November 7, 2017

import numpy as np
from charGrid import *
import Queue as Q
#import math
#pi = math.pi




# Inherit from Queue.PriorityQueue in order to add tuple[1].contains([value]) functionality
class ExQ(Q.PriorityQueue):

    # This data structure assumes the following format:
    # ( priority, value, value ), where the values are implemented as
    # ( distanceToGoal, [row, col], [parentRow parentCol] )
    
    # Return bool if the queue contains a value, regardless of parent or priority
    def contains(self, value):
        if not self.empty():
            return value in (x[1] for x in self.queue)
        else:
            return 0
    # end exists

    # Returns lowest priority item without removing it
    def peek(self):
        if not self.empty():
            temp = self.get()
            self.put(temp)
            return temp
        else:
            return None
    # end peek

    # Returns the priority of an item in list if it exists
    # Assumes no duplicate values
    def priority(self,value):
        for x in self.queue:
            if x[1] == value:
                return x[0]
            # end if
        # end for
        return None
    # end priority

    # Resets the priority of the first matching value
    # Assumes no duplicates
    def replace(self, (priority, value)):
        for x in self.queue:
            if x[1] == value:
                x[0] = priority
                return 1
            # end if
        return 0
    # end replace

    
    def parentOf(self, value):
        for x in self.queue:
            if x[1] == value:
                return x[2]
            
        return None
    #end parentOf
# end overwrite of Queue 



# Function to iterate through the graph and find shortest path
# Assumes the following global variables exist
# _edgeCosts := matrix of edge costs; cost from a to b is edgeCost[a_row][a_col][direction to b] 
# _cellMap := map of nodes structured in 20x20; see graphNode object in charGrid.py
# _openSet := priority queue of open nodes
# _closedSet := priorirty queue of closed nodes
def A_Star():
    global _edgeCosts
    global _cellMap
    global _openSet
    global _closedSet

    # Recursive end case
    if _openSet.empty():
        return
    
    # Variables are used frequently; make local copies
    (estDist, localIndex, parentIndex) = _openSet.get()

    # Get the cell object that contains all node info
    cell = getCellInMap(localIndex)
    parent = getCellInMap(parentIndex)
    
    # Get [row][col] index of the node in the matrix
    (row, col) = cell.getLoc()
    
    # Variable is used frequently; make local copy
    localCost = estDist - cell.getHeur() 
    
    # Take from front of openQ and add to closedQ
    _closedSet.put( (localCost, localIndex, parentIndex) )

    # Counter for accessing edge costs
    count = 0;
    localEdgeCosts = _edgeCosts[row][col][:]

    # Edges are node objects at connections
    for edge in cell.getEdges():

        # Only check directions with a connection
        if edge != None:
            
            # Var is used frequently; make a local copy
            index = edge.getIndex()
            
            # Don't check cells that have already been put in closedQueue
            if not _closedSet.contains(index):
                
                # Look up edge cost to node at connection_i
                localEdgeCosts[count] = localEdgeCosts[count] + edge.getHeur();
                # Look up heurstic to get from node at connection_i to goal
                
                # If cell already found, compare distances. Else add it to the list
                if _openSet.contains(index):

                    # Compare distance to goal taking into account heuristic
                    if (localCost+localEdgeCosts[count]) < _openSet.priority(index):

                        # Replace the value if this one is better!
                        _openSet.replace((localCost+localEdgeCosts[count], index, localIndex))
                    # end if
                else:
                    # Make a new entry in the queue if haven't seen it before
                    _openSet.put((localCost + localEdgeCosts[count], index, localIndex))
                # end if
            else:
                # unwanted case detection
                localEdgeCosts[count] = -1;
            # end if
        else:
            # unwanted case detection
            localEdgeCosts[count] = -1;
        # end if

        # Keeps track of direction: N/E/S/W
        count = count+1
    
    # end for

    # Not technically recursion, because this could be done with a while-loop
    A_Star()
    
# end A_Star()



# Returns the graphNode object at index in the cellMap
def getCellInMap(index):
    global _cellMap
    col = index % 20;
    row = (index-col)/20
    return _cellMap[row][col][1]
# end getCellIn2



# Prints the final path to the screen
def printPath(path):

    # Clear the map back to original state for printing
    clearTravFlags(_cellMap)

    # Init a blank character matrix for printing
    charMap = [[' ' for c in range(20)] for r in range(20)]
    
    # Label the path on the global grid
    for index in path:
        node = getCellInMap(index).getLoc()
        #print node
        (row, col) = node
        #(row, col) = [1, 1]
        _cellMap[row][col][0] = 2
    # end for
    
    # Read grid into character matrix
    for row in range(len(_cellMap)):
        for col in range(len(_cellMap[row])):
            cell = _cellMap[row][col][0]
            if cell == 1:
                charMap[row][col] = '#'
            elif cell == 2:
                charMap[row][col] = '.'
            else:
                charMap[row][col] = ' '
            # end if
        # end for
    # end for
    
    # Print the character matrix
    for row in charMap:
        print ''.join(row)
    # end for
# end printPath



def generatePathFromClosedQueue(endLoc):
    
    global _cellMap

    cell = _cellMap[endLoc[0]][endLoc[1]][1]
    path = []
    path.append(cell.getIndex())
    parent = getCellInMap(_closedSet.parentOf(cell.getIndex()))
    distance = _closedSet.priority(cell.getIndex())
    while parent is not cell:
        cell = parent
        parentIndex = _closedSet.parentOf(cell.getIndex())
        parent = getCellInMap(parentIndex)
        path.append(cell.getIndex())
    # end while
    return (path, distance)
# end generatePathFromClosedQueue



# Global variables. Internal; don't want to broadcast at top of file
_edgeCosts = [ [[1, 1, 1, 1] for cell in range(20) ] for row in range(20)] 
_cellMap = readCSVFromPath('world.csv', 'rU', ',')
_openSet = ExQ()
_closedSet = ExQ()
world = []
if __name__ == "__main__":

    # Not completely sure initiating globals works in Python, but this seemed to work  
    _edgeCosts = [ [[1, 1, 1, 1] for cell in range(20) ] for row in range(20)] 
    _cellMap = readCSVFromPath('world.csv', 'rU', ',')
    _openSet = ExQ()
    _closedSet = ExQ()
    
    printWelcome()

    # Define start and end locations
    startLoc = np.array([0,0])
    endLoc = np.array([19,19])

    # Convert matrix of '1' & '0' to undirected graph
    generateGraphFromMap(_cellMap)
    
    # Define a starting node for the graph
    graphHead = _cellMap[startLoc[0],startLoc[0],1]
   
    # Iterate through entire matrix of cells and set heuristic
    estimateHeuristicForGraph(_cellMap, endLoc)

    # Starting node
    _openSet.put( (graphHead.getHeur(), graphHead.getIndex(), graphHead.getIndex()) )

    # Main workhorse function
    A_Star()

    # Get the path from the closed queue created by A*
    (path, distance) = generatePathFromClosedQueue(endLoc)

    # Print the path!!
    printPath(path)
    print 'Path distance:', distance, 'cells (weight 1 per edge)'

# end main  
