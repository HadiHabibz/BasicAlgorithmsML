#!/usr/bin/python
from collections import deque
from copy import copy, deepcopy
from parse import *
from heapq import *

# startState = 11
startState = 9
goalState = 6

# This class holds the matrix represented using either
# an adjacency matrix to vector list. It also includes
# basic graph operations.
class GraphBasics:
    'General class that provides basic operations for graph'
    
    # Adjacency matrix of the graph
    # This remains empty if the graph
    # is represented using vector list
    graphMatrix = []
    
    # Save here the heuristic cost of each
    # Vertex. This is a lazy approach. A better
    # approach would be to embedded h cost information
    # in the graph matrix or the vertex list
    heuristicCosts = [];
        
    # Define constructor
    # Determine if the graph is weighted, directed 
    # and is adjacency matrix or vector list
    def __init__( self ):
        
        self.initializeGraphMatrixAndVertexList()
        
    # end function constructor
        
    # This function returns the next node we need to go to
    # It assumes we are currently in the 'currentNode'.
    # costs are computed by adding the weight of each edge
    # to the heuristic cost of the states, thereby making
    # it compatible with A* search. The function can be
    # used for heuristic search as well, but the cost of
    # each edge must be set to one first. This can be done
    # by calling the convertToUnweighted() function
    # NodeExceptionList includes the states that are already
    # expanded. There is no need to expand them twice.
    def getNextLeastCostlyState( self,\
    currentNode,\
    nodeExceptionList = [] ):
        
        # This variable represent the cost of the node
        # We are seeking the node with the lowest cost
        # Cost by default is greater than 0
        minimumValue = 0;
        
        # Extract the row associated with the current node
        # in the adjacency matrix. This allows us to see
        # the connected states and their costs
        extractStateRow = \
        self.getExtractedRowFromGraph( currentNode )
        
        # Keep a copy of the matrix as we are going to 
        # modify it to avoid reselecting states 
        extractStateRowCopy = list( extractStateRow )
        
        # Repeat the process until there is no more 
        # state left to discover.
        while True:
            
            # This is a dead-end
            # There is nowhere to go from here
            if len( extractStateRow ) == 0:
                return -1
            
            # Find the least cost among connected states
            # If the graph is unweighted this returns
            # the first node according alphabetically
            minimumValue = min( extractStateRow )
            
            # Find the location of the least costly node
            # Because we delete an element at a time from
            # the vector, this location is relative
            minimumValueIndexRelative = \
            extractStateRow.index( minimumValue )
            
            # Find the actual location of the node. The
            # actual location of the node is equivalent 
            # to the node itself (e.g., node 3 is in location
            # 3).
            minimumValueIndexActual = \
            extractStateRowCopy.index( minimumValue )
            
            # Make sure we do not over-count a node twice
            extractStateRowCopy[ minimumValueIndexActual ] = -1;
            
            # If the selected node is actually connected 
            # to the 'currentState' and if the selected node
            # is not already selected in previous iterations
            # stop searching
            if minimumValue != 0 and\
            minimumValueIndexActual not in nodeExceptionList:
                break;
            
            # If this node is not connected to our 'currentState'
            # or if we have already checked it in previous iterations
            # just delete from the list to avoid encountering it again
            del extractStateRow[minimumValueIndexRelative]
            
        # end while minimumValue    
        
        return minimumValueIndexActual
        
    # end function getNextLeastCostlyState
    
    # Return the adjacency row of the 'currentNode' to see
    # which nodes it connects to.
    def getExtractedRowFromGraph(\
    self,\
    currentNode ):
        
        # Basically, return the row in the adjacency matrix 
        # that is associated with the current node 
        extractedStateRow = list( self.graphMatrix[currentNode] )   
        
        # Recompute the cost of each add by adding 
        # the heuristic cost of the connected sate
        # to the weight of the edge
        for i in range( len( extractedStateRow ) ):
            if extractedStateRow[i] != 0:
                extractedStateRow[i] = \
                extractedStateRow[i] + self.getHeuristicCost( i )  
            
        return extractedStateRow
        
    # end function getExtractedRowFromGraph
    
    # Check to see if two states are neighbors or not
    def areTheseNodesConnected( self, node1, node2 ):
        
        connectionFlag = False;
        
        # Extract the entry of the first node from the adjacency matrix
        node1Row = self.getExtractedRowFromGraph( node1, "matrix" )
        
        # Check if there is a link between two nodes
        if node1Row[node2] != 0:
            connectionFlag = True;
            
        return connectionFlag;
        
    # end function areTheseNodesConnected

    # Return the weight of the link between the source
    # and destination, if there is any link at all
    # This is compatible with both adjacency and vector list
    # Does not include heuristic costs
    def getWeight( self, source, destination ):
    
        # Check to see if nodes are indeed connected
        if self.areTheseNodesConnected(\
        source, destination ) == False:
            return 0
            
        extractedRow = \
        self.getExtractedRowFromGraph( source )
        
        return extractedRow[destination]
        
    # end function getWeight
    
    # Return a specific element in the graph matrix
    def setGraphMatrix( self, row, col, weight ):
        self.graphMatrix[row][col] = weight
    # end function setGraphMatrix
    
    # Return the entire graph matrix
    def getGraphMatrix( self ):
        return self.graphMatrix
    # end function getGraphMatrix
    
    # Get the heuristic cost of the state
    def getHeuristicCost( self, state ):
        return self.heuristicCosts[ state ]
    # end function getHeuristicCost
    
    # Set a specific element in the graph matrix
    def getGraphMatrixElement( self, row, col ):
        return self.graphMatrix[row][col]
    # end function getGraphMatrixElement
    
    # Pre-load a weighted undirected graph and
    # the heuristic costs assigned to each state
    def initializeGraphMatrixAndVertexList( self ):
            
        # Set the heuristic costs for the graph
        # Nodes are in this order:
        # [a, b, c, d, e, f, g, h, p, q, r, s]
        self.heuristicCosts = \
        [5, 7, 4, 7, 5, 2, 0, 11, 14, 12, 3, 0];
    
        # Undirected Weighted graph
        # Path to the goal exists
        self.graphMatrix = [\
        [0, 2, 2, 0, 0, 0, 0, 0, 0,  0, 0, 0],\
        [2, 0, 0, 1, 0, 0, 0, 0, 0,  0, 0, 0],\
        [2, 0, 0, 8, 0, 3, 0, 0, 0,  0, 0, 0],\
        [0, 1, 8, 0, 2, 0, 0, 0, 0,  0, 0, 3],\
        [0, 0, 0, 2, 0, 0, 0, 8, 0,  0, 2, 9],\
        [0, 0, 3, 0, 0, 0, 2, 0, 0,  0, 2, 0],\
        [0, 0, 0, 0, 0, 2, 0, 0, 0,  0, 0, 0],\
        [0, 0, 0, 0, 8, 0, 0, 0, 4,  4, 0, 0],\
        [0, 0, 0, 0, 0, 0, 0, 4, 0, 15, 0, 1],\
        [0, 0, 0, 0, 0, 0, 0, 4, 15, 0, 0, 0],\
        [0, 0, 0, 0, 2, 2, 0, 0, 0,  0, 0, 0],\
        [0, 0, 0, 3, 9, 0, 0, 0, 1,  0, 0, 0] ]
        
#         # Undirected Weighted graph
#         self.graphMatrix = [\
#         [0, 2, 2, 0, 0, 0, 0, 0, 0,  0, 0, 0],\
#         [2, 0, 0, 1, 0, 0, 0, 0, 0,  0, 0, 0],\
#         [2, 0, 0, 8, 0, 3, 0, 0, 0,  0, 0, 0],\
#         [0, 1, 8, 0, 2, 0, 0, 0, 0,  0, 0, 3],\
#         [0, 0, 0, 2, 0, 0, 0, 8, 0,  0, 2, 9],\
#         [0, 0, 3, 0, 0, 0, 0, 0, 0,  0, 2, 0],\
#         [0, 0, 0, 0, 0, 2, 0, 0, 0,  0, 0, 0],\
#         [0, 0, 0, 0, 8, 0, 0, 0, 4,  4, 0, 0],\
#         [0, 0, 0, 0, 0, 0, 0, 4, 0, 15, 0, 1],\
#         [0, 0, 0, 0, 0, 0, 0, 4, 15, 0, 0, 0],\
#         [0, 0, 0, 0, 2, 2, 0, 0, 0,  0, 0, 0],\
#         [0, 0, 0, 3, 9, 0, 0, 0, 1,  0, 0, 0] ]
            
        return
 
    # end function initializeGraphMatrixAndVertexList
    
    # This function receives a weighted matrix 
    # and converts it to an unweighted one.
    # It is used by the heuristic search
    def convertToUnweighted( self ):
        
        # A weighted graph has all weights equal to 1
        weight = 1;
        
        # Iterate through every element and set the
        # weight of every edge to 1
        for i in range( len( self.graphMatrix ) ):
            for j in range( len( self.graphMatrix[i] ) ):
                if self.getGraphMatrixElement( i, j ) != 0:
                    self.setGraphMatrix( i, j, weight )
                    
    # end function convertToUnweighted
    
    def printGraphMatrix( self ):
        for i in self.graphMatrix:
            print( i )
    # end function printGraphMatrix
    
# end class GraphBasics

# The parent class of all search algorithms
# It includes common operation for search
class Search:

    # The graph we need to search
    # Constructed by default values
    graph = GraphBasics()
    
    # Hold a copy of the created graph.
    # A copy will be saved in this variable 
    # by the constructor
    graphRestore = GraphBasics()
    
    # The state we are currently expanding
    currentState = 0
    
    # The initial state, where the search 
    # begins from
    startState = 0
    
    # The goal state. Find a path from start state to goal state
    goalState = 0
    
    # Set this when the search is unsuccessful
    failure = False
    
    # Keep track of the expanded node
    # Do not expanded a node twice.
    # This is an improvement over the search
    # algorithm discussed in the class
    nodesVisited = []
    
    # Constructor
    def __init__( self, graph, startState, goalState ):
        
        self.graph = deepcopy( graph )
        self.graphRestore = deepcopy( graph )
        self.currentState = startState
        self.startState = startState
        self.goalState = goalState
        self.nodesVisited = []
        self.failure = False
        
    # end function constructor
    
    # This function receives the number of a node and
    # returns its alphabetical value
    def mapStateNumbersToLetters( self, stateNumber ):    
        
        # These letters are in accordance with the graph
        # provided in the assignment.
        stateLabels = "ABCDEFGHPQRS"
        
        return stateLabels[stateNumber]
    
    # end function mapStateNumbersToLetters
    
    # This function restores the original values
    # It should be called when the search is terminated
    # to make the searcher ready for another search
    def reset( self ):
    
        # Reconnect all the nodes that we temporarily
        # disconnected during the last search
        self.graph = deepcopy( self.graphRestore )
        
        # Go back to the first node
        self.currentState = self.startState
        del self.nodesVisited[:]
        
    # end function reset
    
    # Check if the goal state is reached
    def goalStateIsReached( self, strategy ):
        
        # If the current state is the goal state
        # the search is successful and the function
        # should be terminated
        if self.currentState == self.goalState:
            print ( 'Path found (', strategy, '):' )
            self.printStack()
            self.reset()
            return True
            
    # end class goalStateIsReached
    
    # Get all the possible paths we can take from the
    # current state. Save the paths in a queue from the
    # most desirable to the least desirable
    def extractAndSaveEveryFringe( self ):
        
        # Hold all the existing paths in this temporary stack
        # The first element that does in is the most desirable one
        temporaryShortFringeStack = [];
        
        # Keep iterating until there is no more
        # path left from this current node to explore
        while True:
            
            # Get the best next node to explore
            nextStateToGo = self.graph.getNextLeastCostlyState(\
            self.currentState,\
            self.nodesVisited ) 
            
            # All paths are explored
            if nextStateToGo < 0:
                return temporaryShortFringeStack
            
            # Avoid encountering this node again
            self.graph.setGraphMatrix( self.currentState, nextStateToGo, 0 )
            
            # Add the node to the stack
            temporaryShortFringeStack.append( nextStateToGo )
            
        # end while
        
    # end function extractAndSaveEveryFringe
    
    # Extract all nodes that are connected to the current
    # state. From them pick the least expensive one and make
    # a transition into that one. If no such node exists, 
    # take a step backward and attempt to restore a previous
    # node. If no previous node exists, terminate the search
    # unsuccessfully. 
    def goToTheNextState( self ):

        # Get the best nodes to explore
        extractedNodes = self.extractAndSaveEveryFringe()
        
        # The current node is now fully expanded
        # Add it to the expanded nodes list
        self.nodesVisited.append( self.currentState )
                
        # If there is no node left to extend from the 
        # current node, we are in a dead end. Take a step
        # back to the previous state. If not, move to the
        # next state and extend it. Make sure no loop exists
        # Handling the dead-ends and making transitions 
        # based on costs are algorithm specific. 
        if len( extractedNodes ) == 0:
            self.resolveDeadend()
            return False
        
        else:                    
            # Complete the transition
            self.makeTransition( extractedNodes )  
            return True
                
    # end function goToTheNextState
    
    # The major steps for all search algorithms are
    # the same. Keep searching until either no state
    # is left to expand or the goal is reached
    def performSearch( self, strategyName ):
        
        # Before performing the search, ensure
        # that everything is initialized correctly.
        self.initializations()  
        
        # Keep searching until either a path
        # is found or the search is utterly
        # unsuccessful
        while self.failure == False:
            
            # Check is search is successful, terminate the
            # function
            if self.goalStateIsReached( strategyName ):
                return;
            
            # Expand all the nodes connected to the current state
            # Start from the ones with the least heuristic cost
            # until either a dead is reached or the the goal state
            # is found
            self.goToTheNextState()
            
    # end function performSearch
    
    # Print the states that are visited during
    # the search
    def printNodesVisited( self ):
    
        print( 'States Expanded:', end="" )
        
        # Add the start state to the list, if it's not already there
        if self.nodesVisited[0] != self.startState:
            self.nodesVisited.insert( 0, self.startState )
            
        if self.goalState not in self.nodesVisited:
            self.nodesVisited.append( self.goalState )
        
        # Print every single state visited
        for i in self.nodesVisited:
        
            # Convert numbers to letters
            stateLetter = \
            self.mapStateNumbersToLetters( int( i ) )
            
            # Do some formatting for the very last letter
            if i != self.nodesVisited[-1]:
                print( stateLetter, ',', end="" )
            else:
                print( stateLetter );
                
        # end for i
    
    # end function printNodesVisited
    
# end class Search

# Search a graph using A* algorithm
class AStarSearch( Search ):
    'Search a graph using A* algorithm'
    
    # Hold fringe and its associated cost
    # This will be a priority queue of tuples
    priorityQueue = []  
    
    # Constructor
    def __init__( self, graph, startState, goalState ):
        
        Search.__init__( self, graph, startState, goalState )
        self.priorityQueue = []
        
    # end function constructor

    # Do some basic initializations 
    # This is required if we run two HS's on 
    # the same graph consecutively
    def initializations( self ):
        
        # Only delete it if it is not empty
        if len( self.priorityQueue ) != 0:
            del self.priorityQueue
        
        # Create the queue anew
        self.priorityQueue = []
        
        # Put the start state in the fringe by default
        # (as a list)
        # This is the structure of tuples in the priority 
        # queue: (total A cost, heuristic cost of the last
        # state in the fringe, fringe)
        # It is important to save the h cost of the last node
        # as it allows us to break ties properly
        firstTuple = ( 0, 0, [self.startState] )
        self.priorityQueue.append( firstTuple )
    
        # Initially, we assume the search is 
        # successful unless we are proved the 
        # opposite    
        self.failure = False
        
    # end function initializations
    
    # A wrapper function for performing search
    def aStarSearch( self ):
        
        self.performSearch( "A* Search" )
        
    # end function aStarSearch
    
    # Compute and return A cost from the current node to 
    # the destination node. The A cost includes to A cost
    # to get to the current node plus the h cost of the 
    # destination node plus the weight of the edge that 
    # connects the destination node to the current node
    def getAccumulativeCost( self, destinationNode, currentCost ):
        
        # Extract the weight of the connecting edge
        weight = self.graphRestore.getGraphMatrixElement(\
        self.currentState, destinationNode )

        # Extract the h cost of the destination
        # A cost only include the cost of the very 
        # last state in the fringe. So we must remove
        # the h cost of any node prior to that one
        heuristicCost = self.graph.getHeuristicCost(\
        destinationNode ) - self.graph.getHeuristicCost(\
        self.currentState )
        
        totalCost = currentCost + weight + heuristicCost
        
        return totalCost
    # end function getAccumulativeCost
    
    # Probably the heart of A* algorithm
    # Pop the fringe that is fully expanded 
    # from the priority queue.
    # Also, add all new fringes to the queue
    # Priority queue automatically sorts them 
    # and inherently breaks ties effectively
    # Finally, set the current state to the 
    # next best one
    def makeTransition( self, extractedNodesToGo ):
            
        # The node at the top of the stack is fully
        # expanded at this point. Pop it.
        # Each entry in the priority queue is a tuple,
        # where the first entry is the cost and the 
        # second one is the fringe
        fringeTailFullInfo = heappop( self.priorityQueue )
        fringeTailCost = fringeTailFullInfo[0]
        fringeTail = fringeTailFullInfo[2]
        
        # Insert a new fringe for every possible path explored
        # Priority queue takes care of ordering things 
        while len( extractedNodesToGo ) > 0:
            
            # The last state that comes out of extractedNodesToGo is the 
            # node with the least cost that we want to transit into
            nextStateToGo = extractedNodesToGo.pop()
             
            # Add each connected state to fringe to create a new longer fringe 
            extendedFringe = deepcopy( fringeTail )
            extendedFringe.append( nextStateToGo )
            extendedFringeCost = self.getAccumulativeCost(\
            nextStateToGo, fringeTailCost )
            hCostOfNextStateToGo =\
            self.graphRestore.getHeuristicCost( nextStateToGo )
             
            # Put the new fringe in the stack
            heappush( self.priorityQueue, (\
            extendedFringeCost, hCostOfNextStateToGo,\
            extendedFringe ) )
                        
        # end while
        
        # Now that everything is sorted, fetch the state 
        # with the least total cost from the priority queue
        leastExpensiveFringeTuple = self.priorityQueue[0]
        leastExpensiveFringe = deepcopy( leastExpensiveFringeTuple[2] )
        nextStateToGo = leastExpensiveFringe.pop()
        
        # Avoid loops
        # Destroy the path from destination to source
        self.graph.setGraphMatrix( self.currentState, nextStateToGo, 0 )
        
        # Go to the next state
        self.currentState = nextStateToGo
        
    # end function makeTransition
    
    # If we end up in a dead end, this function
    # enables us to take a step back to the 
    # previous state by extracting the next best
    # fringe (if it exists)
    def resolveDeadend( self ):
    
        # If this is a dead-end and there is no more
        # alternatives left to explore that the search
        # is unsuccessful
        if len( self.priorityQueue ) == 0 :
            self.failure = True
            return
            
        # Remove the bad fringe (dead-end) from the
        # priority queue and discard it
        heappop( self.priorityQueue )
        
        # No alternative is left
        # The search is not successful
        if len( self.priorityQueue ) == 0 :
            self.failure = True
            return
                
        # Read the next fringe double from the priority queue
        alternativeFringeTuple = self.priorityQueue[0]
        alternativeFringe = alternativeFringeTuple[2]
       
        # The state at the end of the fringe is the next
        # one to expand
        alternativeState = alternativeFringe.pop();
        
        # Keep fetching until a either the queue is empty
        # or an unexplored state is found in the queue
        if alternativeState in self.nodesVisited:
            self.resolveDeadend()
            return
            
        # Go back to the parent node
        self.currentState = alternativeState 
        return alternativeState
    # end function resolveDeadend
    
    # Print the path that is currently in the fringe
    def printStack( self ):
        
        # Prints nodes that are expanded 
        self.printNodesVisited()
        
        print( 'Path Returned: ', end="" )
        
        # Get the length of the stack
        fringeTuple = self.priorityQueue[0]
        fringe = fringeTuple[2]
        
        # Get the number of states on the path
        statesCount = len( fringe )
        
        for i in range( statesCount ):

            # Convert numbers to letters
            stateLetter = \
            self.mapStateNumbersToLetters( int( fringe[i] ) )
            
            # The very last node does not need a transition
            if i < statesCount-1:
                print( stateLetter, "->", end="" )
                
            else:
                print( stateLetter );
        # end for i        
    # end function printStack
    
# end class AStarSearch

# An implementation of heuristic search graph 
# Example use:
# myGraph = GraphBasics()
# HSsearch = HeuristicSearch( myGraph, startState, goalState )
# HSsearch.heuristicSearch()
class HeuristicSearch( Search ):
    'Search a graph for the goal state using Heuristic search'

    # Hold fringe and total cost here
    # Keep the lest expensive path on top of the stack
    fringeStack = []

    # Constructor
    def __init__( self, graph, startState, goalState ):
        
        Search.__init__( self, graph, startState, goalState )
        self.fringeStack = []
        self.failure = False
        
    # end function constructor
    
    # Do some basic initializations 
    # This is required if we run two HS's on 
    # the same graph consecutively
    def initializations( self ):
        
        # Only delete it if it is not empty
        if len( self.fringeStack ) != 0:
            del self.fringeStack
        
        # Create the queue anew
        self.fringeStack = []
        
        # Put the start state in the fringe by default
        # (as a list)
        self.fringeStack.append( [self.startState] )
        
        # Heuristic search does not consider the 
        # weight of the edges. Therefore, we set 
        # all of them to 1 to avoid any discrimination
        self.graph.convertToUnweighted();
    
        # Initially, we assume the search is 
        # successful unless we are proved the 
        # opposite    
        self.failure = False
        
    # end function initializations
    
    # A wrapper function for search
    # Call the parent function
    def heuristicSearch( self ):
        
        self.performSearch( "Heuristic Search" )
        
    # end function heuristicSearch
    
    # The heart of the heuristic algorithm
    # Add every fringe to the stack based on
    # their cost and set the current state to
    # the next least expensive state
    def makeTransition( self, extractedNodesToGo ):
        
        # The node at the top of the stack is fully
        # expanded at this point. Pop it.
        fringeTail = self.fringeStack.pop()
        
        # Insert a new fringe for every possible path explored
        # The first node that comes out of the extractedNodesToGo
        # is the worst option. Therefore, when we are done copying
        # the most desirable option is on the top of the stack 
        while len( extractedNodesToGo ) > 0:
            
            # The last state that comes out of extractedNodesToGo is the 
            # node with the least cost that we want to transit into
            nextStateToGo = extractedNodesToGo.pop()
            
            # Add each connected state to fringe to create a new longer fringe 
            extendedFringe = deepcopy( fringeTail )
            extendedFringe.append( nextStateToGo )
             
            # Put the new fringe in the stack
            self.fringeStack.append( extendedFringe )
            
        # end while
        
        # Avoid loops
        # Destroy the path from destination to source
        self.graph.setGraphMatrix( self.currentState, nextStateToGo, 0 )
        
        # Go to the next state
        self.currentState = nextStateToGo
        
    # end function makeTransition
    
    # If we end up in a dead end, this function
    # enables us to take a step back to the 
    # previous state by extracting the next best
    # fringe (if it exists)
    def resolveDeadend( self ):
    
        # If this is a dead-end and there is no more
        # alternatives left to explore that the search
        # is unsuccessful
        if len( self.fringeStack ) == 0 :
            self.failure = True
            return
            
        # Remove the bad fringe (dead-end) from the
        # fringe stack
        self.fringeStack.pop()
        
        # No alternative is left
        # The search is not successful
        if len( self.fringeStack ) == 0 :
            self.failure = True
            return
                
        # Remove the next fringe in the stack and
        # quickly return back, we just want to read it
        alternativeFringe = self.fringeStack.pop()
        self.fringeStack.append( alternativeFringe );
        
        # The state at the end of the fringe is the next
        # one to expand
        alternativeState = alternativeFringe.pop();
        
        if alternativeState in self.nodesVisited:
            self.resolveDeadend()
            return
            
        # Go back to the parent node
        self.currentState = alternativeState 
        return alternativeState
    # end function resolveDeadend
    
    # Print the path that is currently in the fringe
    def printStack( self ):
        
        # Prints nodes that are expanded 
        self.printNodesVisited()
        
        print( 'Path Returned: ', end="" )
        
        # Get the path to the goal
        fringe = self.fringeStack.pop()
        
        # Get the number of states on the path
        statesCount = len( fringe )
        
        for i in range( statesCount ):

            # Convert numbers to letters
            stateLetter = \
            self.mapStateNumbersToLetters( int( fringe[i] ) )
            
            # The very last node does not need a transition
            if i < statesCount-1:
                print( stateLetter, "->", end="" )
                
            else:
                print( stateLetter );
        # end for i        
    # end function printStack    
    
# end class HeuristicSearch


myGraph = GraphBasics()
HSsearch = HeuristicSearch( myGraph, startState, goalState )
HSsearch.heuristicSearch()
print( "\n---------------------------------------------------" )
ASearch = AStarSearch( myGraph, startState, goalState )
ASearch.aStarSearch()