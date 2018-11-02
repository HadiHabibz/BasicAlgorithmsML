from tree import *
import math
from concurrent.futures._base import RUNNING

class MinimaxSearch:
    
    def __init__( self ):
        
        self.tree = BinaryTree( 0, None )
        self.tree.initializeTree()
        self.currentNode = 0
        self.alpha = -math.inf
        self.beta = math.inf
                
    # end Constructor
    
    def isThisAgentMax( self, rootLocation ):
        
        turn = True
        total = 0
        
        for i in range( self.tree.getSize() ):
            
            total += 2**i
            
            if rootLocation < total:
                return turn
            
            turn = not turn
    
    # end function isThisAgentMax    
    
    def getMaximumValue( self, tree ):
        
        runningMax = -math.inf
        
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMax = max( runningMax, leftChildValue )
        
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMax = max( runningMax, rightChildValue )
        
        return runningMax
        
    # end function getMaximumValue
    
    def getMinimumValue( self, tree ):
        
        runningMin = math.inf
        
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMin = min( runningMin, leftChildValue )
        
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMin = min( runningMin, rightChildValue )
        
        return runningMin
        
    # end function getMaximumValue
    
    def getMaximumValueWithPruning( self, tree ):
        
        runningMax = -math.inf
        
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMax = max( runningMax, leftChildValue )
        
        if runningMax >= self.beta:
            return runningMax
        
        self.alpha = max( self.alpha, runningMax )
        
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMax = max( runningMax, rightChildValue )
        
        return runningMax
        
    # end function getMaximumValueWithPruning
    
    def getMinimumValueWithPruning( self, tree ):
        
        runningMin = math.inf
        
        leftChildValue = self.evaluateState( tree.getLeftChild() )
        runningMin = min( runningMin, leftChildValue )
        
        if runningMin <= self.alpha:
            return runningMin
        
        self.beta = min( self.beta, runningMin )
        
        rightChildValue = self.evaluateState( tree.getRightChild() )
        runningMin = min( runningMin, rightChildValue )
        
        return runningMin
        
    # end function getMinimumValueWithPruning
    
    def evaluateState( self, tree = None ):
        
        if tree == None:
            tree = self.tree
        
        if tree.getNodeValue() != None:
            return tree.getNodeValue()    
             
        if self.isThisAgentMax( tree.getRoot() ):
            value = self.getMaximumValue( tree )
            
        else:
            value = self.getMinimumValue( tree )
            
        tree.setNodeValue( value )
        
        return tree.getNodeValue()
        
    # end function evaluateState
    
    def evaluateStateWithPruning( self, tree = None ):
        
        if tree == None:
            tree = self.tree
        
        if tree.getNodeValue() != None:
            return tree.getNodeValue()    
             
        if self.isThisAgentMax( tree.getRoot() ):
            value = self.getMaximumValueWithPruning( tree )
            
        else:
            value = self.getMinimumValueWithPruning( tree )
            
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
#searcher.minimaxSearchWithPruning()
