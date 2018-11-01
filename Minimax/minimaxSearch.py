from tree import *
import math

class MinimaxSearch:
    
    def __init__( self ):
        
        self.tree = BinaryTree( 0, None )
        self.tree.initializeTree()
        self.currentNode = 0
                
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
        
        leftChildValue = tree.getLeftChild().getNodeValue()
        rightChildValue = tree.getRightChild().getNodeValue()
        
        if leftChildValue == None and\
        rightChildValue == None:
            return None
        
        if leftChildValue > rightChildValue:
            return leftChildValue
        
        return rightChildValue
        
    # end function getMaximumValue
    
    def getMinimumValue( self, tree ):
                
        leftChildValue = tree.getLeftChild().getNodeValue()
        rightChildValue = tree.getRightChild().getNodeValue()
        
        if leftChildValue == None and\
        rightChildValue == None:
            return None
        
        if leftChildValue > rightChildValue:
            return rightChildValue
        
        return leftChildValue
        
    # end function getMinimumValue
      
    def getStateValue( self, tree ):
        
        if self.isThisAgentMax( tree.getRoot() ):
            return self.getMaximumValue( tree )
            
        else:
            return self.getMinimumValue( tree )
        
    # end function getStateValue
    
    def evaluateState( self, tree = None ):
        
        if tree == None:
            tree = self.tree
            
        value = None

        while True:
            
            value = self.getStateValue( tree )
            
            if value != None:
                break
            
            if tree.getLeftChild() != None:
                self.evaluateState( tree.getLeftChild() )
                
            if self.tree.getRightChild() != None:
                self.evaluateState( tree.getRightChild() ) 
                
        # end while
        
        tree.setNodeValue( value )
        
    # end function evaluateState
    
    def minimaxSearch( self ):
        
        self.tree.initializeTree()
        self.evaluateState()
        self.tree.printTree()
        
    # end function minimaxSearch
     
# end class MinimaxSearch

searcher = MinimaxSearch()
searcher.minimaxSearch()
