class BinaryTree():
    
    def __init__( self, rootID, value = None ):
        
        self.left = None
        self.right = None
        self.rootid = rootID
        self.value = value
        self.size = 1;

    # end Constructor 
    
    def getLeftChild( self ):
        
        return self.left
    
    # end function getLeftChild 
    
    def getRightChild( self ):
        
        return self.right
    
    # end function getRightChild
    
    def setNodeValue( self, value ):
        
        self.value = value
    
    # end function setNodeValue
        
    def getNodeValue( self ):
        
        return self.value

    # end function getNodeValue
    
    def getRoot( self ):
        
        return self.rootid
        
    # end function getRoot
    
    
    def getSize( self ):
        
        return self.size
    
    # end function getSize
    
    def insertRight( self, newNode, value ):
        
        if self.right == None:
            self.right = BinaryTree( newNode, value )
            
        else:
            tree = BinaryTree( newNode, value )
            tree.right = self.right
            self.right = tree
            
        self.size += 1
            
    # end function insertRight

    def insertLeft( self, newNode, value ):
        
        if self.left == None:
            self.left = BinaryTree( newNode, value )
            
        else:
            tree = BinaryTree( newNode, value)
            tree.left = self.left
            self.left = tree
            
        self.size += 1

    # end function insertLeft
    
    def setLeftAndRightChildren( self, leftChild, rightChild ):
        
        if self.left != None:
            return
        
        if self.right != None:
            return
        
        self.left = leftChild
        self.right = rightChild
        
        self.size += 2
        
    # end function setLeftAndRightChildren
    
    def initializeTree( self ):
        
        trees = []
        values = [3, 10, 2, 9, 10, 7, 5, 9, 2, 5, 6, 4, 2, 7, 9, 1]
        
        for i in range( 8 ):
            treeID = BinaryTree( i + 7, None )
            treeID.insertLeft( i * 2 + 15, values[i * 2] )
            treeID.insertRight( i * 2 + 16, values[i * 2 + 1] )
            trees.append( treeID )
            
        tree3 = BinaryTree( 3, None )
        tree3.setLeftAndRightChildren( trees[0], trees[1] )
        tree4 = BinaryTree( 4, None )
        tree4.setLeftAndRightChildren( trees[2], trees[3] )
        tree5 = BinaryTree( 5, None )
        tree5.setLeftAndRightChildren( trees[4], trees[5] )
        tree6 = BinaryTree( 6, None )
        tree6.setLeftAndRightChildren( trees[6], trees[7] )
        
        tree1 = BinaryTree( 1, None )
        tree1.setLeftAndRightChildren( tree3, tree4 )
        tree2 = BinaryTree( 2, None )
        tree2.setLeftAndRightChildren( tree5, tree6 )
        
        self.setLeftAndRightChildren( tree1, tree2 )
        
    # end function initializeTree
    
    def printTree( self ):
        
        space = " "
        
        tree0 = self
        tree1 = self.getLeftChild()
        tree2 = self.getRightChild()
        tree3 = tree1.getLeftChild()
        tree4 = tree1.getRightChild()
        tree5 = tree2.getLeftChild()
        tree6 = tree2.getRightChild()
        tree7 = tree3.getLeftChild()
        tree8 = tree3.getRightChild()
        tree9 = tree4.getLeftChild()
        tree10 = tree4.getRightChild()
        tree11 = tree5.getLeftChild()
        tree12 = tree5.getRightChild()
        tree13 = tree6.getLeftChild()
        tree14 = tree6.getRightChild()
        tree15 = tree7.getLeftChild()
        tree16 = tree7.getRightChild()
        tree17 = tree8.getLeftChild()
        tree18 = tree8.getRightChild()
        tree19 = tree9.getLeftChild()
        tree20 = tree9.getRightChild()
        tree21 = tree10.getLeftChild()
        tree22 = tree10.getRightChild()
        tree23 = tree11.getLeftChild()
        tree24 = tree11.getRightChild()
        tree25 = tree12.getLeftChild()
        tree26 = tree12.getRightChild()
        tree27 = tree13.getLeftChild()
        tree28 = tree13.getRightChild()
        tree29 = tree14.getLeftChild()
        tree30 = tree14.getRightChild()
        
        print( tree0.getNodeValue() )
        
        print( tree1.getNodeValue(), end = " " )
        print( tree2.getNodeValue() )
        
        print( tree3.getNodeValue(), end = " " )
        print( tree4.getNodeValue(), end = " " )
        print( tree5.getNodeValue(), end = " " )
        print( tree6.getNodeValue() )
        
        print( tree7.getNodeValue(), end = " " )
        print( tree8.getNodeValue(), end = " " )
        print( tree9.getNodeValue(), end = " " )
        print( tree10.getNodeValue(), end = " " )
        print( tree11.getNodeValue(), end = " " )
        print( tree12.getNodeValue(), end = " " )
        print( tree13.getNodeValue(), end = " " )
        print( tree14.getNodeValue() )
        
        print( tree15.getNodeValue(), end = " " )
        print( tree16.getNodeValue(), end = " " )
        print( tree17.getNodeValue(), end = " " )
        print( tree18.getNodeValue(), end = " " )
        print( tree19.getNodeValue(), end = " " )
        print( tree20.getNodeValue(), end = " " )
        print( tree21.getNodeValue(), end = " " )
        print( tree22.getNodeValue(), end = " " )
        print( tree23.getNodeValue(), end = " " )
        print( tree24.getNodeValue(), end = " " )
        print( tree25.getNodeValue(), end = " " )
        print( tree26.getNodeValue(), end = " " )
        print( tree27.getNodeValue(), end = " " )
        print( tree28.getNodeValue(), end = " " )
        print( tree29.getNodeValue(), end = " " )
        print( tree30.getNodeValue(), end = " " )

    
# end class BinaryTree