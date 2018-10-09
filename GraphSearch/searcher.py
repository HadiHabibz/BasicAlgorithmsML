#!/usr/bin/python
from collections import deque
from copy import copy, deepcopy
from parse import *
from heapq import *

startState = 11
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
	
	# Vector list of the graph
	# This remains empty if the graph
	# is represented by adjacency matrix
	vertexList = []
		
	# Define constructor
	# Determine if the graph is weighted, directed 
	# and is adjacency matrix or vector list
	def __init__( self,\
	representation = "matrix",\
	directed = "directed",\
	weighted = "unweighted"  ):
		self.initializeGraphMatrixAndVertexList(\
		representation, directed, weighted )
	# end function constructor

	# This function returns the next node we need to go to
	# It assumes we are currently in the 'currentNode'
	# It returns the least costly node first if the graph
	# is weighted. If the graph is unweighted it returns
	# the nodes alphabetically. It excludes any node that
	# is currently expanded. It is compatible with both 
	# adjacency matrix and vector list
	def getNextLeastCostlyState( self, currentNode,\
	nodeExceptionList = [], graphRepresentation = "matrix" ):
		
		# This variable represent the cost of the node
		# We are seeking the node with the lowest cost
		minimumValue = 0;
		
		extractStateRow = \
		self.getExtractedRowFromGraph(\
		currentNode, graphRepresentation )
		
		extractStateRowCopy = list( extractStateRow )
		
		while True:
			
			# This is a deadend
			if len( extractStateRow ) == 0:
				return -1
			
			# Find the cost of the node with the least cost
			# If the graph is unweighted this returns
			# the first node according to the alphabet 
			minimumValue = min( extractStateRow )
			
			# Find the location of the least costly node
			# Because we delete an element at a time from
			# the vector, this location is relative
			minimumValueIndexRelative = \
			extractStateRow.index( minimumValue )
			
			# Find the actual location of the node. The
			# actual location of the node is equivalent 
			# to the node itsel (e.g., node 3 is in location
			# 3).
			minimumValueIndexActual = \
			extractStateRowCopy.index( minimumValue )
			
			# Make sure we do not overcount a node twice
			extractStateRowCopy[ minimumValueIndexActual ] = -1;
			
			# If the selected node is actually connected 
			# to the 'currentState' and if the selected node
			# is not already selected in previous iterations
			# stop searching
			if minimumValue != 0 and\
			minimumValueIndexActual not in nodeExceptionList:
				break;
			
			# If this node is not connected to our 'currentState'
			# or if we have already checked it in previolus iterations
			# just delete from the list to avoid encountering it again
			del extractStateRow[minimumValueIndexRelative]
			
		# end while minimumValue	
		
		return minimumValueIndexActual
		
	# end function getNextLeastCostlyState
	
	# For the adjacency row of the 'currentNode' to see
	# which nodes it is connects to. We select and delete 
	# the least costly nodes one by one. We also need an
	# untouched copy of this row to locate the actual location
	# of each node.
	def getExtractedRowFromGraph(\
	self, currentNode, graphRepresentation = "matrix" ):
		
		# Reading from the adjacency matrix is straightforward
		if graphRepresentation == "matrix":
			extractStateRow = list( self.graphMatrix[currentNode] )	
			return extractStateRow
		
		# The rest is only needed for vector lists
		totalNumberOfStates = len( self.vertexList )
		extractStateRow = [0] * totalNumberOfStates
		
		# Extract the vertex list of the current node
		statesAndWeights = str( self.vertexList[currentNode] )
		a = statesAndWeights	
		# Each entry in the vertex list has three fields:
		# current state, connected states, weights
		statesAndWeights = parse( "['{}:[{}],[{}]']", statesAndWeights )
		
		if statesAndWeights == None:
			return extractStateRow
			
		currentNodeNumber = statesAndWeights[0]
		connectedNodes = statesAndWeights[1].split( "," )
		weights = statesAndWeights[2].split( "," )
		
		# Encode connected nodes and their weights together
		j = 0
		
		for i in connectedNodes:
			extractStateRow[int( i )] = int( weights[j] )
			j += 1
		
		# Return the result
		return extractStateRow
	# end function getExtractedRowFromGraph
	
	# Check to see if two states are neighbors or not
	# Compatible with both adjacency matrix and vector list
	def areTheseNodesConnected( self, node1, node2 ):
		connectionFlag = False;
		
		# Check if the graph is represented by adjacency matrix or not
		if len( self.graphMatrix ) != 0:
			node1Row = self.getExtractedRowFromGraph( node1, "matrix" )
		else:
			node1Row = self.getExtractedRowFromGraph( node1, "vectorList" )
		
		# Check if there is a link between two nodes
		if node1Row[node2] != 0:
			connectionFlag = True;
			
		return connectionFlag;
		
	# end function areTheseNodesConnected

	# Return the weight of the link between the source
	# and destination, if there is any link at all
	# This is compatible with both adjacency and vector list
	def getWeight( self, source, destination ):
	
		# Check to see if nodes are indeed connected
		if self.areTheseNodesConnected(\
		source, destination ) == False:
			return 0
			
		# Check if the grpah is represented by the adjacency
		# matrix or vector list
		if len( self.graphMatrix ) == 0:
			representation = "VertexList"
		else:
			representation = "matrix"
			
		extractedRow = \
		self.getExtractedRowFromGraph( source, representation )
		
		return extractedRow[destination]
		
	# end function getWeight
	
	# This function deletes a link between a source and 
	# destination to avoid loops.
	def destroyPath( self, source, destination ):
		
		# Check if nodes are indeed connected
		# If not, no action is required
		if self.areTheseNodesConnected( source, destination ) == False:
			return
		
		# Check if the graph is represented by adjacency matrix
		# If it is, the procedure is straightforward
		if len( self.graphMatrix ) != 0:
			self.setGraphMatrx( source, destination, 0 )
			return
		 
		# Extract the connected nodes from the vector list
		extractedRow =\
		self.getExtractedRowFromGraph( source, "vertexList" )
		
		# Include all connected nodes except the destination
		# to the entry of the source in the vector list
		entry = str( source ) + ":" + "["
		weights = "["
		
		for i in range( len( extractedRow ) ):
			
			# Skip the destination
			if extractedRow[i] == 0 or\
			i == destination:
				continue
			
			# Reconstruct the entry in the vector list
			entry += str( i )
			entry += ","
			weights += str( extractedRow[i] )
			weights += ","
			
		entry = entry[:-1]
		entry += "]"
		weights = weights[:-1]
		weights += "]"
		
		entry += ","
		entry += weights
		
		self.vertexList[source][0] = entry 			
	
	# end function destroyPath
	
	def setGraphMatrx( self, row, col, weight ):
		self.graphMatrix[row][col] = weight
	# end function setGraphMatrx
	
	def getGraphMatrix( self ):
		return self.graphMatrix
	# end function getGraphMatrix
	
	def getGraphMatrixElement( self, row, col ):
		return self.graphMatrix[row][col]
	# end function getGraphMatrixElement
	
	# Pre-load the graph given in the document file
	# Based on the input setting, load directed, weighted,
	# adjacency matrix or vector list
	# adjacency matrix format is:
	#   a b c d e f g h p q r s
	# a w w w w w w w w w w w w
	# b w w w w w w w w w w w w
	# c w w w w w w w w w w w w
	# d w w w w w w w w w w w w
	# e w w w w w w w w w w w w
	# f w w w w w w w w w w w w
	# g w w w w w w w w w w w w
	# h w w w w w w w w w w w w
	# p w w w w w w w w w w w w
	# q w w w w w w w w w w w w
	# r w w w w w w w w w w w w
	# s w w w w w w w w w w w w
	# Where 'w' is the weight and zero weight means no
	# connection. The vector list has one entry per node
	# and each entry is a string formatted like this:
	# "A:[B,C,F],[4,9,1]"
	# In this example, node A is connected to nodes B, C,
	# and F with weights 4, 9, and 1, respectively
	def initializeGraphMatrixAndVertexList(\
	self, representation = "matrix",\
	directed = "directed",\
	weighted = "unweighted" ):
	
		isDirected = False
		isMatrix = False
		isWeighted = False
		
		if directed == "directed":
			isDirected = True
			
		if weighted == "weighted":
			isWeighted = True
			
		if representation == "matrix":
			isMatrix = True
			
		# self.graphMatrix = [ \
		# [0, 2, 3, 0, 5, 0],\
		# [2, 0, 0, 4, 0, 0],\
		# [3, 0, 0, 0, 4, 0],\
		# [0, 4, 0, 0, 1, 2],\
		# [5, 0, 4, 1, 0, 5],\
		# [0, 0, 0, 2, 5, 0] ]

		# This graph is directed and weighted
		if isDirected == True and\
		isWeighted == True and\
		isMatrix == True:
			
			self.graphMatrix = [\
			[0, 0, 0, 0, 0, 0, 0, 0,  0,  0, 0, 0],\
			[2, 0, 0, 0, 0, 0, 0, 0,  0,  0, 0, 0],\
			[2, 0, 0, 0, 0, 0, 0, 0,  0,  0, 0, 0],\
			[0, 1, 8, 0, 2, 0, 0, 0,  0,  0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 8,  0,  0, 2, 0],\
			[0, 0, 3, 0, 0, 0, 2, 0,  0,  0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0,  0,  0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0,  4,  4, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0,  0, 15, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0, 15,  0, 0, 0],\
			[0, 0, 0, 0, 0, 2, 0, 0,  0,  0, 0, 0],\
			[0, 0, 0, 3, 9, 0, 0, 0,  1,  0, 0, 0] ] 
			
			return
			
		# This graph is undirected and weighted
		if isDirected == False and\
		isWeighted == True and\
		isMatrix == True:
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
			
			return
			
		# This graph is directed and unweighted
		if isDirected == True and\
		isWeighted == False and\
		isMatrix == True:
			
			self.graphMatrix = [\
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0],\
			[0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0] ]
			
			return
			 
		# This graph is undirected and unweighted
		if isDirected == False and\
		isWeighted == False and\
		isMatrix == True:
			
			self.graphMatrix = [\
			[0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
			[1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],\
			[1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0],\
			[0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],\
			[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1],\
			[0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0],\
			[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0],\
			[0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],\
			[0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],\
			[0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],\
			[0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0] ] 
			
			return
		
		# This graph is directed and weighted
		if isDirected == True and\
		isWeighted == True and\
		isMatrix == False:
			self.vertexList = [\
			["0:[],[]"],\
			["1:[0],[2]"],\
			["2:[0],[2]"],\
			["3:[1,2,4],[1,8,2]"],\
			["4:[7,10],[8,2]"],\
			["5:[2,6],[3,2]"],\
			["6:[],[]"],\
			["7:[8,9],[4,4]"],\
			["8:[9],[15]"],\
			["9:[],[]"],\
			["10:[5],[2]"],\
			["11:[3,4,8],[3,9,1]"]]	
			
		# This graph is undirected and weighted
		if isDirected == False and\
		isWeighted == True and\
		isMatrix == False:
			self.vertexList = [\
			["0:[1,2],[2,2]"],\
			["1:[0,3],[2,1]"],\
			["2:[0,3,5],[2,8,3]"],\
			["3:[1,2,4,11],[1,8,2,3]"],\
			["4:[3,7,10,11],[2,8,2,9]"],\
			["5:[2,6,10],[3,2,2]"],\
			["6:[5],[2]"],\
			["7:[4,8,9],[8,4,4]"],\
			["8:[7,9,11],[4,15,1]"],\
			["9:[7,8],[4,15]"],\
			["10:[4,5],[2,2]"],\
			["11:[3,4,8],[3,9,1]"]]		
		
			return
			
		# This graph is undirected and unweighted
		if isDirected == True and\
		isWeighted == False and\
		isMatrix == False:
			self.vertexList = [\
			["0:[],[]"],\
			["1:[0],[1]"],\
			["2:[0],[1]"],\
			["3:[1,2,4],[1,1,1]"],\
			["4:[7,10],[1,1]"],\
			["5:[2,6],[1,1]"],\
			["6:[],[]"],\
			["7:[8,9],[1,1]"],\
			["8:[9],[1]"],\
			["9:[],[]"],\
			["10:[5],[1]"],\
			["11:[3,4,8],[1,1,1]"]]		
		
			return
			
		# This graph is undirected and unweighted
		if isDirected == False and\
		isWeighted == False and\
		isMatrix == False:
			self.vertexList = [\
			["0:[1,2],[1,1]"],\
			["1:[0,3],[1,1]"],\
			["2:[0,3,5],[1,1,1]"],\
			["3:[1,2,4,11],[1,1,1,1]"],\
			["4:[3,7,10,11],[1,1,1,1]"],\
			["5:[2,6,10],[1,1,1]"],\
			["6:[5],[1]"],\
			["7:[4,8,9],[1,1,1]"],\
			["8:[7,9,11],[1,1,1]"],\
			["9:[7,8],[1,1]"],\
			["10:[4,5],[1,1]"],\
			["11:[3,4,8],[1,1,1]"]]		
		
			return 
	# end function initializeGraphMatrixAndVertexList
	
	def printGraphMatrix( self ):
		for i in self.graphMatrix:
			print( i );
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
	
	# Count the number of states we come 
	# search across during 
	statesVisted = 0
	
	# The state we are currently expanding
	currentState = 0
	
	# The initial state, where the search 
	# begins from
	startState = 0
	
	# The goal state. Find a path from start state to goal state
	goalState = 0
	
	# Keep track of the expanded node
	# Do not expanded a node twice.
	# This is an improvement over the search
	# algorithm discussed in the class
	nodesVisited = []
	
	# Is the graph represneted by adjacency matrix
	# or a vector list. This has become obsolete in 
	# functions developed recently but it is needed
	# for legacy ones. I can get rid of it if I want
	# to.
	preferredRepresentation = "matrix"
	
	# Constructor
	def __init__( self, graph, startState, goalState ):
		self.graph = deepcopy( graph )
		self.graphRestore = deepcopy( graph )
		statesVisted = 0
		self.currentState = startState
		self.startState = startState
		self.goalState = goalState
		self.nodesVisited = []
		self.preferredRepresentation = "matrix"
	# end function constructor
	
	# Check if all nodes are searched and we still
	# haven't reached our goal
	def noRouteIsLeft( self ):
		
		extractedRow =\
		self.graph.getExtractedRowFromGraph(\
		self.startState, self.preferredRepresentation )
		
		if( sum( extractedRow[:] ) == 0 ):
			return True
				
		return False

	# end function noRouteIsLeft
	
	# This function receives the number of a node and
	# returns its alphabeticall value
	def mapStateNumbersToLetters( self, stateNumber ):	
		
		# These letters are in accordance with the graph
		# provided in the assignment.
		stateLabels = "ABCDEFGHPQRS"
		
		return stateLabels[stateNumber]
	
	# end function mapStateNumbersToLetters
	
	# This is where we have reached the goal
	def searchSuccessful( self ):
	
		# Print the full path from source the goal
		self.printStack()
		
		# Reset everything so that the searcher is
		# ready to start a new search
		self.reset()
	# end function searchSuccessful
	
	# This function restores the original values
	# It should be called when the search is terminated
	# to make the searcher ready for another search
	def reset( self ):
	
		# Reconnect all the nodes that we temporarily
		# disconnected during the last search
		self.graph = deepcopy( self.graphRestore )
		self.statesVisted = 0
		
		# Go back to the first node
		self.currentState = self.startState
		del self.nodesVisited[:]
		
	# end function reset
	
	def goalStateIsReached( self, strategy ):
		
		# If the current state is the goal state
		# the search is successful and the function
		# should be terminated
		if self.currentState == self.goalState:
			print ( 'Path found (', strategy, '):' )
			self.searchSuccessful()
			return True
			
	# end class goalStateIsReached
	
	def failedToFindAPath( self ):
		# If there is no node left to go to, then the
		# search is unsuccessful and it must be terminated
		if self.noRouteIsLeft():
			print( 'No path found!' )
			self.reset()
			return True 
	# end function failedToFindAPath
	
	def goToTheNextState( self ):

		# Get the best next node to explore
		nextStateToGo = self.graph.getNextLeastCostlyState(\
		self.currentState, self.nodesVisited,\
		self.preferredRepresentation )
		
		# If there is no node left to extend from the 
		# current node, we are in a deadend. Take a step
		# back to the previous state. If not, move to the
		# next state and extend it. Make sure no loop exists
		if nextStateToGo < 0:
			self.resolveDeadend()
			return False
		else:		
			self.makePathOneWayToAvoidLoops( nextStateToGo )
			return True
				
	# end function goToTheNextState
	
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

# Search a path from start state to goal state
# using uniform cost search algorithm
# The UCS is implemented using priority queue
# which in turn is implemented using heap
class UniformCostSearch( Search ):
	'Perform uniform cost search on the input graph'
	
	# Hold fringe and total cost here
	# Keep the lest expensive path on top of the queue
	priorityQueue = []
	
	# Need to make the priority queue work with
	# Strings. The reason the queue uses strings 
	# is because I use one queue to save both 
	# the aggregate cost and fringe in a single
	# queue. If the grpah is a lot biger than 
	# what we have (in order of hundreds or 
	# thousands) or if the costs are a lot higher
	# I should change the padding width to a higher
	# value
	paddingWidth = 4
	
	# Set this when the search is unsuccessful
	failure = False
	
	# Constructor
	def __init__( self, graph, startState, goalState ):
		Search.__init__( self, graph, startState, goalState )
		self.priorityQueue = []
		self.failure = False
	# end function constructor
	
	# Do some basic initializations 
	# This is requried if we run two UCS on 
	# the same graph consecuitively
	def initializations( self ):
		
		# Only delete it if it is not empty
		if len( self.priorityQueue ) != 0:
			del self.priorityQueue
		
		self.priorityQueue = []
		
		firstEntryInQueue = "0."
		firstEntryInQueue += str( self.startState )
		heappush( self.priorityQueue, firstEntryInQueue ) 
		self.failure = False
		
	# end function initializations
	
	# Perform uniform cost search using priority queue
	# implemented by heap. Most of the functionality is
	# recycled from the parent class
	def uniformCostSearch( self ):

		# Check to see if this is the very first
		# call of the function, if it is, reset 
		# the stack
		if self.statesVisted == 0:
			self.initializations()
	
		# Keep search untill either a path is found
		# Or the search fails.
		while self.failure == False:
			self.statesVisted += 1		
	
			# Check is search is successfully complete
			if self.goalStateIsReached( "Iterative UCS" ):
				return;
			
			# Expand all the nodes connected to the current state
			# And put them in the priority queue
			while self.goToTheNextState():
				pass
			
		# end while
		
	# end function uniformCostSearch
	
	# UFC when the graph is represented by 
	# the vector list. The difference is mostly
	# handled by the Graph class
	def uniformCostSearch_VertexList( self ):
		self.preferredRepresentation = "VertexList"
		self.uniformCostSearch()
		self.preferredRepresentation = "matrix"		
	# end function uniformCostSearch
	
	# Once a next state to go is returned, at it 
	# to the priority queue. The data is saved in
	# the queue as a string in a particular format:
	# The weight of the fringe is written first.
	# A '.' separates the weight from the actual 
	# fringe. The nodes in the fringe are separated
	# using ','. Because everything is a string, 
	# numbers are padded with zeros to make comparisons
	# work. Otherwise, the heap could not determine 
	# say 12 is biger than 8 (since 1 is smaller than 
	# 8). Padding zeros solves the proble 0012 vs 0008.
	# Most of the complexity in the function is for 
	# parsing the queue.
	def makePathOneWayToAvoidLoops( self, nextStateToGo ):
		
		firstEntryInQueue = self.priorityQueue[0]
		weightAndFringe = firstEntryInQueue.split( "." )
		fringeWeight = int( weightAndFringe[0] )
		fringeExtension = weightAndFringe[1]
		fringeExtension += ","
		nextStateToGoString = str( nextStateToGo )
		fringeExtension +=\
		nextStateToGoString.rjust( self.paddingWidth, '0' )
		transitionWeight = self.graph.getWeight(\
		self.currentState, nextStateToGo )
		fringeWeight += transitionWeight
		fringeWeightString = str( fringeWeight )
		fringeWeightString =\
		fringeWeightString.rjust( self.paddingWidth, '0' )
		newFringe = fringeWeightString + "." + fringeExtension
		heappush( self.priorityQueue, newFringe )
		
		# Update the graph matrix by eliminating the return path
		self.graph.destroyPath( nextStateToGo, self.currentState )
		self.graph.destroyPath( self.currentState, nextStateToGo )
			
	# end function makePathOneWayToAvoidLoops
	
	# Once a node is depleted, meaning that there is no
	# connected node to explore, transit to the next state.
	# The next state is the one with the lowest fringe cost.
	# The lowest value is always saved at the location 0 of
	# the heap
	def resolveDeadend( self ):
		
		# If the heap is empty and there is no
		# more nodes to expand, then no path exists
		if len( self.priorityQueue ) == 0:
			self.failure = True
			return
		
		# Get rid of the current fringe
		visitedNode = heappop( self.priorityQueue )
		
		# Parse the fringe
		visitedNode = visitedNode.split( "," )
		
		if len( visitedNode ) == 1:
			visitedNode = visitedNode[0].split( "." )
			
		visitedNode = int( visitedNode[-1] )
		
		# Add the fully expanded node into nodesVisted.
		# This allows us to make sure we do not expand
		# a node twice. This is an improvement to what
		# we discuss in the class
		if visitedNode not in self.nodesVisited:
			self.nodesVisited.append( visitedNode )
		
		# Set the current state to the last node in the 
		# fringe with the lowest cost (always saved atl
		# location 0)
		nextStateToExpand = self.priorityQueue[0]
		nextStateToExpand = nextStateToExpand.split( "," )
		nextStateToExpand = int( nextStateToExpand[-1] )
		self.currentState = nextStateToExpand
	
	# end function resolveDeadend
	
	# Print the path that is currently in the queue
	def printStack( self ):
		
		self.printNodesVisited()
		
		print( 'Path Returned: ', end="" )
		
		# Get the length of the stack
		fringeAndCost = self.priorityQueue[0]
		fringeAndCost = fringeAndCost.split( "." )
		fringe = fringeAndCost[1]
		fringe = fringe.split( "," )		
		statesCount = len( fringe )
		
		for i in range( statesCount ):

			# Convert numbers to letters
			stateLetter = \
			self.mapStateNumbersToLetters( int( fringe[i] ) )
			
			# The very last node does not need a transition
			if i < statesCount-1:
				print( stateLetter, '->', end="" )
			else:
				print( stateLetter );
		# end for i		
	# end function printStack
	
# end class UniformCostSearch

# Implement breadth first search algorithm to 
# find a path between start state and the goal 
# state. This class is compatible with both 
# adjacency matrix and vector list. The functions
# are implemented both recusively and iteratively.
class BreadthFirstSearch( Search ):
	'This class performs a search using BFS algorithm'
	
	# The backbone of BFS is the queue
	fringeQueue = deque()
	
	# Fringe
	fringeTail = "";

	# Constructor
	def __init__( self, graph, startState, goalState ):
		Search.__init__( self, graph, startState, goalState )
		self.fringeQueue = deque()
		self.nodesVisited = []
	# end function constructor

	# Set everything to its initial value
	# This is needed if we run BFS on the same graph
	# multiple times
	def initializations( self ):
	
		if len( self.fringeQueue ) != 0:
			del self.fringeQueue
		
		if len( self.nodesVisited ) != 0:
			del self.nodesVisited[:]	
			
		if len( self.fringeTail ) != 0:
			del self.fringeTail[:]
			
		self.nodesVisited.append( self.startState )
			
	# end function initializations
	
	# Search for a path from the start state to the 
	# goal state using breadth first search algorithm
	def breadthFirstSearch( self ):

		# Do some initializations
		self.initializations()
				
		while True:
		
			# Keep track of number of nodes visted
			self.statesVisted += 1
			
			# Check is search is successfully complete
			if self.goalStateIsReached( "Iterative BFS" ):
				return;
			
			# Go to the next state, if there is any
			self.goToTheNextState()
			
			# Terminate when search is unsuccessful
			if self.failedToFindAPath():
				return
			
		# end while
		
	# end function breadthFirstSearch
	
	# Search for a path from the start state to the 
	# goal state using recusive breadth first search 
	# algorithm
	def breadthFirstSearchRecursive( self ):

		# Check to see if this is the first run
		# If yes, do some initializations
		if self.statesVisted == 0:
			self.initializations()
		
		# Keep track of number of nodes visted
		self.statesVisted += 1
		
		# Check is search is successfully complete
		if self.goalStateIsReached( "Recursive BFS" ):
			return;
		
		# Go to the next state, if there is any
		self.goToTheNextState()
		
		# Terminate when search is unsuccessful
		if self.failedToFindAPath():
			return
		
		# Solve the same problem recursively.
		# This time with a new node that is hopefully
		# closer to the destination
		self.breadthFirstSearchRecursive()
		
	# end function breadthFirstSearchRecursive

	# Search for a path from the start state to the 
	# goal state using recusive breadth first search 
	# algorithm. Use vertex list
	def breadthFirstSearchRecursive_VertexList( self ):
		self.preferredRepresentation = "vertexList"
		self.breadthFirstSearchRecursive()
		self.preferredRepresentation = "matrix"		
	# end function breadthFirstSearchRecursive_VertexList
	
	# Search for a path from the start state to the 
	# goal state using breadth first search algorithm. 
	# Use vertex list
	def breadthFirstSearch_VertexList( self ):
		self.preferredRepresentation = "vertexList"
		self.breadthFirstSearch()
		self.preferredRepresentation = "matrix"		
	# end function breadthFirstSearch_VertexList
	
	# If there is no new nodes to go to from the 
	# current node, then we reach a deadend. To
	# resolve this, we go back to the previous node
	# by poping it from the queue
	def resolveDeadend( self ):
	
		# Next state to go is the previous one in
		# case of a deadend
		nextStateToGo = self.fringeQueue.popleft()
		
		# Extract nodes from the string
		# nodes are separated using ','s
		separatedNodes = nextStateToGo.split( "," )
		
		# This holds the path from the soure to
		# the current node
		self.fringeTail = "";
		
		# Add all the nodes from the source to the 
		# current node one by one
		for i in separatedNodes:
			self.fringeTail += i
			self.fringeTail += ","
		
		# Set the next node to go to the previous
		# node of the current node
		nextStateToGo = int( separatedNodes[-1] )
		self.currentState = nextStateToGo

	# end function resolveDeadend
	
	# When making a transition from a node to another
	# Delete the return path from the graph matrix to 
	# make sure no infinite loop is created
	def makePathOneWayToAvoidLoops( self, nextStateToGo ):
	
		# Update the graph matrix by eliminating the return path
		self.graph.destroyPath( nextStateToGo, self.currentState )
		
		if len( self.fringeTail ) == 0:
			fringe = str( self.currentState ) + "," + \
			str( nextStateToGo )
			
		else:
			fringe = self.fringeTail + \
			str( nextStateToGo )
		
		# Insert the next state in the queue
		self.fringeQueue.append( fringe )
		
		# Avoid duplicating an existing node in the possible final answer
		if nextStateToGo not in self.nodesVisited:
			self.nodesVisited.append( nextStateToGo );
			
	# end function makePathOneWayToAvoidLoops
	
	# Print the path that is currently in the queue
	def printStack( self ):
		
		self.printNodesVisited()
		
		print( 'Path Returned: ', end="" )
		# Get the length of the stack
		self.fringeTail = self.fringeTail.split( "," )
		statesCount = len( self.fringeTail )-1
		
		for i in range( statesCount ):

			# Convert numbers to letters
			stateLetter = \
			self.mapStateNumbersToLetters( int( self.fringeTail[i] ) )
			
			# The very last node does not need a transition
			if i < statesCount-1:
				print( stateLetter, '->', end="" )
			else:
				print( stateLetter );
		# end for i		
	# end function printStack	
	
# end class BreadthFirstSearch

# Search for path form the start state to the 
# goal state using depth first search algorithms
# The class is compatible with both adjacency matrix
# and vector list. It is implemented both recursively
# and iteratively
class DepthFirstSearch( Search ):
	'This class performs search using DFS algorithm'
	
	# The backbone of the DFS is the stack
	fringeStack = []
	
	# Constructor
	def __init__( self, graph, startState, goalState ):
		Search.__init__( self, graph, startState, goalState )
		self.fringeStack = []
		self.nodesVisited = []
	# end function constructor
	
	# Use depth first search to find a path from
    # start node to the goal node
	def depthFirstSearch( self ):
		
		self.fringeStack = []
	
		while True:
			
			# Put the current node in the stack
			self.fringeStack.append( self.currentState )
			self.statesVisted += 1
			
			# Keep track of expanded nodes
			if self.currentState not in self.nodesVisited:
				self.nodesVisited.append( self.currentState )

			# Terminate when search is unsuccessful
			if self.failedToFindAPath():
				return
				
			# Check is search is successfully complete
			if self.goalStateIsReached( "Iterative DFS" ):
				return;

			# Go to the next state, if there is any
			self.goToTheNextState()
				
			# end while 
			
	# end function depthFirstSearchRecursive
	
	# Use recursion and depth first search to find 
	# a path from start node to the goal node
	def depthFirstSearchRecursive( self ):
		
		# Check to see if this is the very first
		# call of the function, if it is, reset 
		# the stack
		if self.statesVisted == 0:
			self.fringeStack = []
			
		# Put the current node in the stack
		self.fringeStack.append( self.currentState )
		self.statesVisted += 1
		
		# Keep track of expanded nodes
		if self.currentState not in self.nodesVisited:
			self.nodesVisited.append( self.currentState )
		
		# Check is search is successfully complete
		if self.goalStateIsReached( "Recursive DFS" ):
			return;
		
		# Terminate when search is unsuccessful
		if self.failedToFindAPath():
			return

		# Go to the next state, if there is any
		self.goToTheNextState()
		
		# Now solve the same problem. The only difference
		# is that we have new start node this time that is 
		# hopefully closer to the goal
		self.depthFirstSearchRecursive()
		
	# end function depthFirstSearchRecursive
	
	def depthFirstSearchRecursive_VertexList( self ):
		self.preferredRepresentation = "vertexList"
		self.depthFirstSearchRecursive()
		self.preferredRepresentation = "matrix"
	# end function depthFirstSearchRecursive_VertexList
	
	def depthFirstSearch_VertexList( self ):
		self.preferredRepresentation = "vertexList"
		self.depthFirstSearch()
		self.preferredRepresentation = "matrix"
	# end function depthFirstSearchRecursive_VertexList
	
	def makePathOneWayToAvoidLoops( self, nextStateToGo ):
		self.currentState = nextStateToGo
	# end function makePathOneWayToAvoidLoops
	
	# Print the path that is currently in the stack
	def printStack( self ):
		
		self.printNodesVisited()
		print( 'Path Returned: ', end="" )
		
		# Get the length of the stack
		statesCount = len( self.fringeStack )
		
		for i in range( statesCount ):

			# Convert numbers to letters
			stateLetter = \
			self.mapStateNumbersToLetters( self.fringeStack[i] )
			
			# The very last node does not need a transition
			if i < statesCount-1:
				print( stateLetter, '->', end="" )
			else:
				print( stateLetter );
		# end for i		
	# end function printStack	
	
	# If we end up in a deadend, this function
	# enables us to take a step back to the 
	# previous state
	def resolveDeadend( self ):
	
		# Remove the bad node (deadend) from the
		# fringe stack
		badSate = self.fringeStack.pop()
			
		# Remove the parent node as well, we will
		# add the parent node agian in the search
		# function if it is good nod (with no deadend)
		previousState = self.fringeStack.pop()
		
		# Destroy any path from the parent node to 
		# this node in order to avoid getting there 
		# again
		self.graph.destroyPath( previousState, badSate )
		
		# Go back to the parent node
		self.currentState = previousState 
		return previousState
	# end function resolveDeadend
	
# end class DepthFirstSearch 

# Test BFS and DFS for undirected unweighted graph 
# represented by adjacency matrix (Recursive and non
# Recursive)
print( '\n-------------------Undirected \ Unweighted \ Matrix-----------------------' )
myGraph = GraphBasics( "matrix", "undirected", "unweighted" )
searcherBFS = BreadthFirstSearch( myGraph, startState, goalState )
searcherDFS = DepthFirstSearch( myGraph, startState, goalState )

searcherBFS.breadthFirstSearchRecursive()
searcherBFS.breadthFirstSearch()
searcherDFS.depthFirstSearchRecursive()
searcherDFS.depthFirstSearch()

# Test BFS and DFS for undirected unweighted graph 
# represented by vector list (Recursive and non
# Recursive)
print( '\n--------------------Undirected \ Unweighted \ Vertex List------------------' )
myGraph = GraphBasics( "vertexList", "undirected", "unweighted" )
searcherBFS = BreadthFirstSearch( myGraph, startState, goalState )
searcherDFS = DepthFirstSearch( myGraph, startState, goalState )

searcherBFS.breadthFirstSearchRecursive_VertexList()
searcherBFS.breadthFirstSearch_VertexList()
searcherDFS.depthFirstSearchRecursive_VertexList()
searcherDFS.depthFirstSearch_VertexList()

# Test BFS and DFS for directed unweighted graph 
# represented by adjacency matrix (Recursive and non
# Recursive)
print( '\n--------------------Directed \ Unweighted \ Matrix-------------------------' )
myGraph = GraphBasics( "matrix", "directed", "unweighted" )
searcherBFS = BreadthFirstSearch( myGraph, startState, goalState )
searcherDFS = DepthFirstSearch( myGraph, startState, goalState )

searcherBFS.breadthFirstSearchRecursive()
searcherBFS.breadthFirstSearch()
searcherDFS.depthFirstSearchRecursive()
searcherDFS.depthFirstSearch()

# Test BFS and DFS for directed unweighted graph 
# represented by Vertex List (Recursive and non
# Recursive)
print( '\n--------------------Directed \ Unweighted \ vertexList---------------------' )
myGraph = GraphBasics( "vertexList", "directed", "unweighted" )
searcherBFS = BreadthFirstSearch( myGraph, startState, goalState )
searcherDFS = DepthFirstSearch( myGraph, startState, goalState )

searcherBFS.breadthFirstSearchRecursive_VertexList()
searcherBFS.breadthFirstSearch_VertexList()
searcherDFS.depthFirstSearchRecursive_VertexList()
searcherDFS.depthFirstSearch_VertexList()

# Test UCS for undirected weighted graph represented by
# adjacency matrix
print( '\n--------------------Undirected \ Weighted \ Matrix-------------------------' )
myGraph = GraphBasics( "matrix", "undirected", "weighted" )
searcherUCS = UniformCostSearch( myGraph, startState, goalState )
searcherUCS.uniformCostSearch()

# Test UCS for undirected weighted graph represented by
# adjacency vertex list
print( '\n--------------------Undirected \ Weighted \ Vertex List---------------------' )
myGraph = GraphBasics( "vertexList", "undirected", "weighted" )
searcherUCS = UniformCostSearch( myGraph, startState, goalState )
searcherUCS.uniformCostSearch_VertexList()

# Test UCS for directed weighted graph represented by
# adjacency matrix
print( '\n--------------------Directed \ Weighted \ Matrix---------------------------' )
myGraph = GraphBasics( "matrix", "directed", "weighted" )
searcherUCS = UniformCostSearch( myGraph, startState, goalState )
searcherUCS.uniformCostSearch()

# Test UCS for directed weighted graph represented by
# adjacency vertex list
print( '\n--------------------Directed \ Weighted \ Vertex List----------------------' )
myGraph = GraphBasics( "vertexList", "directed", "weighted" )
searcherUCS = UniformCostSearch( myGraph, startState, goalState )
searcherUCS.uniformCostSearch_VertexList()