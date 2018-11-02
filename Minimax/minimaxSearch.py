from tree import *
import math

# Implementation of the minimax search
# Works only with full binary trees
# Pruning is also implemented
class MinimaxSearch:
    'Minimax search for full binary trees'
    
    # Constructor
    def __init__( self ):
        
        # Build an empty tree
        self.tree = BinaryTree( 0, None )
        
        # Reset all values to their default
        self.resetSearch()
           
    # end Constructor
    
    # Reset values to their default
    # This is important when we want to run
    # multiple searches in a row. Objects of
    # this class always reset themselves before
    # performing search
    def resetSearch( self ):
        
        # Pre-load the tree with the one provided for
        # this project
        self.tree.initializeTree()
        
        # Used only for pruning
        # alpha is the best option for maximizing agent
        # beta is the best option for minimizing agent
        self.alpha = -math.inf
        self.beta = math.inf 
        
    # end function resetSearch
    
    # Check if the agent is maximizing or minimizing
    # utility 
    def isThisAgentMax( self, rootLocation ):
        
        # True represent maximizing agent
        turn = True
        
        # Count total of nodes up to this one
        total = 0
        
        # The number of nodes in each layer increases
        # by a factor of 2
        for i in range( self.tree.getSize() ):
            
            total += 2**i
            
            # Start from the top and go down
            if rootLocation < total:
                return turn
            
            # Max and min happen every other layer
            # Keep toggling the turn
            turn = not turn
    
    # end function isThisAgentMax    
    
    # Compute the maximum value of the current root of the 
    # input tree. This only works with binary trees
    def getMaximumValue( self, tree ):
        
        # Initially max is the lowest value possible
        runningMax = -math.inf
        
        # Get the value of the left child
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        
        # Check if the left Child's value is higher than max
        # If so, update max
        runningMax = max( runningMax, leftChildValue )
        
        # Get the value of the right child
        rightChildValue = self.evaluateState( tree.getRightChild() )
        
        # If the value of the child on right is higher than
        # current max, update the max
        runningMax = max( runningMax, rightChildValue )
        
        # Retrun max
        return runningMax
        
    # end function getMaximumValue
    
    # Compute the minimum value of the root of the input
    # tree. This only works for binary trees
    def getMinimumValue( self, tree ):
        
        # A safe guess for the initial minimum
        runningMin = math.inf
        
        # Get the value of the left child
        # Update the running min if the value of this 
        # child is small enough
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMin = min( runningMin, leftChildValue )
        
        # Repet the above process for the right child
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMin = min( runningMin, rightChildValue )
        
        return runningMin
        
    # end function getMaximumValue
    
    # Return the maximum value of the root of the input 
    # tree. Use pruning to improve performance
    def getMaximumValueWithPruning( self, tree ):
        
        runningMax = -math.inf
        
        # Compare current value of maximum with the value
        # of the left child
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMax = max( runningMax, leftChildValue )
        
        # If value of the left child is bigger than the best
        # option for the min agent, then this node cannot 
        # change the outcome of its parent minimizing agent, 
        # no matter what. In this case, continue searching
        # is a waste of time
        if runningMax >= self.beta:
            return runningMax
        
        # Update the best option for maximizing agent
        self.alpha = max( self.alpha, runningMax )
        
        # Now check the value of the right child
        # If it is large enough, it can affect the 
        # outcome of the parent minimizing function
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMax = max( runningMax, rightChildValue )
        
        return runningMax
        
    # end function getMaximumValueWithPruning
    
    # Return the minimum value of the root of the input
    # tree. Compatible only with binary trees
    # Use pruning
    def getMinimumValueWithPruning( self, tree ):
        
        runningMin = math.inf
        
        # Get the value of the left child
        # Update the runningMin if need be
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMin = min( runningMin, leftChildValue )
        
        # If the outcome of this state is lower than the 
        # best choice for the parent maximizing agent, 
        # continue searching is a waste of resources.
        # As this node cannot change the outcome for the
        # parent maximizing agent regardless of the value
        # of its other children
        if runningMin <= self.alpha:
            return runningMin
        
        # Update the beset choice for the minimizing agent
        self.beta = min( self.beta, runningMin )
        
        # Check the right child
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMin = min( runningMin, rightChildValue )
        
        return runningMin
        
    # end function getMinimumValueWithPruning
    
    # Get the value of the current node
    # The values depends on whether this node
    # is a minimizing agent or a minimizing agent
    def evaluateState( self, tree = None ):
        
        # Only happens for the very first invocation 
        # in the recursion change
        # THis is basically to overload the function
        if tree == None:
            tree = self.tree
        
        # If the value exists, stop searching
        if tree.getNodeValue() != None:
            return tree.getNodeValue()    
             
        # Is this a maximizing agent
        # If so, get the maximum
        # Otherwise, get the minimum
        if self.isThisAgentMax( tree.getRoot() ):
            value = self.getMaximumValue( tree )
            
        else:
            value = self.getMinimumValue( tree )
        
        # Once the value of this node is computed 
        # Save it in the tree    
        tree.setNodeValue( value )
        
        return tree.getNodeValue()
        
    # end function evaluateState
    
    # Evaluate the value of each node in the tree
    # Invoke pruning max and min functions
    def evaluateStateWithPruning( self, tree = None ):
        
        # Happens for the very first call only
        if tree == None:
            tree = self.tree
        
        # If the root of the input tree has a valid value
        # stop searching and return that value
        if tree.getNodeValue() != None:
            return tree.getNodeValue()    
            
        # Is this a maximizing agent
        # If yes, find the maximum and use pruning 
        if self.isThisAgentMax( tree.getRoot() ):
            value = self.getMaximumValueWithPruning( tree )
            
        else:
            value = self.getMinimumValueWithPruning( tree )
            
        # Update the tree 
        tree.setNodeValue( value )
        
        return tree.getNodeValue()
        
    # end function evaluateStateWithPruning
    
    def minimaxSearchWithPruning( self ):
        
        self.tree.initializeTree()
        self.evaluateStateWithPruning()
        self.tree.printTree()
        
    # end function minimaxSearchWithPruning
    
    def minimaxSearch( self ):
        
        self.tree.initializeTree()
        self.evaluateState()
        self.tree.printTree()
        
    # end function minimaxSearch
     
# end class MinimaxSearch

searcher = MinimaxSearch()
searcher.minimaxSearch()
searcher.minimaxSearchWithPruning()
