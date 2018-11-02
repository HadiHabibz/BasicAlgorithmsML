from tree import *
import math

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
    
#     def getMaximumValue( self, tree ):
#         
#         leftChildValue = tree.getLeftChild().getNodeValue()
#         rightChildValue = tree.getRightChild().getNodeValue()
#         
#         if leftChildValue == None and\
#         rightChildValue == None:
#             return None
#         
#         if leftChildValue > rightChildValue:
#             return leftChildValue
#         
#         return rightChildValue
#         
#     # end function getMaximumValue
#     
#     def getMinimumValue( self, tree ):
#                 
#         leftChildValue = tree.getLeftChild().getNodeValue()
#         rightChildValue = tree.getRightChild().getNodeValue()
#         
#         if leftChildValue == None and\
#         rightChildValue == None:
#             return None
#         
#         if leftChildValue > rightChildValue:
#             return rightChildValue
#         
#         return leftChildValue
#         
#     # end function getMinimumValue
      
    def getStateValue( self, tree ):
        
        if self.isThisAgentMax( tree.getRoot() ):
            return self.getMaximumValue( tree )
            
        else:
            return self.getMinimumValue( tree )
        
    # end function getStateValue
    
    def updateIfNeedBe( self, valueOld, valueNew, tree ):
        
        if self.isThisAgentMax( tree.getRoot() ):
            return max( valueOld, valueNew )
        
        return min( valueOld, valueNew )
      
    # end function updateIfNeedBe
    
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
    
    def minimaxSearchPruning( self ):
        
        self.tree.initializeTree()
        
    # end function minimaxSearchPruning
    
    def minimaxSearch( self ):
        
        self.tree.initializeTree()
        self.evaluateState()
        self.tree.printTree()
        
    # end function minimaxSearch
     
# end class MinimaxSearch

searcher = MinimaxSearch()
searcher.minimaxSearch()
