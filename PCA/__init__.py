import numpy;
from copy import deepcopy

class PCA:
    'Extract Principal components from data matrix'
    
    def __init__( self, X = None ):
        
        # The matrix of inputs
        # This must be an m by n matrix
        # where m is the number of observations
        # and n is the number of features
        self.matrix = X;
        
        # The covariance matrix of the input data
        self.covarianceMatrix = None;
        
        # Per column mean. This is a 1 by n vector
        # where n is the number of features. It 
        # holds the mean of each column
        self.dataMean = None;
        
        # Eigen values associated with the covariance
        # matrix
        self.eigenValues = None;
        
        # Eigen vectors associated with the covariance
        # matrix
        self.eigenVectors = None;
        
        # Data projected on the principal axes
        self.projectedData = None;

        # If user does not have any data
        # use the sample matrix
        if X.size == 0:
            self.loadSampleMatrix();
            return;
                
    # end constructor
    
    # Load a sample matrix 
    def loadSampleMatrix( self ):
        
        # eight samples of 3D data
        self.matrix = numpy.matrix(\
            '-2, 1, 4, 6, 5, 3, 6, 2;\
            9, 3, 2, -1, -4, -2, -4, 5;\
            0, 7, -5, 3, 2, -3, 4, 6');
            
        # I would rather have observations in rows
        # Features in columns
        self.matrix = numpy.transpose( self.matrix );
        
    # end function loadSampleMatrix
    
    # Center data by subtracting from each column
    # its mean. 
    def centerMatrix( self ):
        
        self.dataMean = self.getMatrixMean();
        self.matrix = self.matrix - self.dataMean;
        
        return self.matrix;
        
    # end function centerMatrix
    
    # Computer the covariance matrix of the 
    # data, depending on whether the data is 
    # normalized or not, return normalized/
    # unnormalized covariance matrix.  
    def computeCovarianceMatrix( self ):
        
        self.covarianceMatrix = \
            numpy.dot( numpy.transpose( self.matrix ),\
            self.matrix, );
            
        return self.covarianceMatrix;
        
    # end function getCovarianceMatrix 
    
    # Get eigen values/vector of the covariance 
    # matrix
    def performEigenDecompositioin( self ):
        
        [self.eigenValues, self.eigenVectors] = \
            numpy.linalg.eig( self.covarianceMatrix );
        
    # end function computeEigenDecompositioin
    
    # Pick the eigen vectors of the covariance 
    # matrix corresponding with the N largest
    # eigen values.
    def getFirstNPrincipalComponents( self, N ):

        # Ensure N is valid
        # N must be less than number of features
        if N > self.eigenValues.shape[0]:
            print( "Warning: Too many PC's!" );
            print( "Set to maximum possible instead!" );
            N = self.eigenValues.shape[0];

        # Allocate memory for performance issues
        mainAxes = numpy.matrix( numpy.zeros(\
            shape = ( N, self.eigenVectors[0].size ) ) );
        
        mainAxesValue = numpy.array( numpy.zeros( shape = ( N ) ) );
        
        # Preserve data in a temporary variable
        eigenValuesCopy = deepcopy( self.eigenValues );
        
        # Extract the eigen vector associated with
        # the biggest eigen value
        for i in range( N ):
            
            highestEigenValueIndex = numpy.argmax( eigenValuesCopy );
            mainAxesValue[i] = numpy.max( eigenValuesCopy );
            mainAxes[i] = numpy.transpose( self.eigenVectors )[highestEigenValueIndex];
            eigenValuesCopy[highestEigenValueIndex] = -numpy.inf;
            
        # end for i 
        
        return [numpy.transpose( mainAxesValue ),\
                numpy.transpose( mainAxes )];
        
    # end getFirstNPrincipalComponents
    
    # Find the projection of data into PC
    # N is the number of principal components
    # It must less than the number of features
    def projectData( self, N = 1 ):
        
        if self.matrix.size == 0:
            print( "No data is loaded." );
            return;
        
        self.centerMatrix();
        self.computeCovarianceMatrix();
        self.performEigenDecompositioin();
         
        # w is the N biggest eigen values
        # V is the N eigen vectors corresponding to w
        w, v = \
            self.getFirstNPrincipalComponents( N );
             
        # Shift data to bring them to their original
        # location (the location before centering)
        self.projectedData = numpy.dot( self.matrix\
            + self.dataMean, v ); 
              
                
        return [self.projectedData, v, w];     
        
    # end function projectData 
    
    # Compute the mean of matrix
    def getMatrixMean( self ):
        
        return numpy.mean( self.matrix, 0 );
        
    # end getMatrixMean
    
# end class PCA

def mypca( X, k ):
    
    worker = PCA( numpy.transpose( X ) );
    
    [projectedData, principalComponent,\
        componentValue] = worker.projectData( k );
     
    return[numpy.transpose( projectedData ),\
            principalComponent, componentValue ];
    
# end function mypca

def printResults( rep, pc, pv ):
    
    print( "Rep: " )

    for row in rep:
        print( ["%5.2f" % element for element in\
            numpy.squeeze( numpy.array( row ) )] )
        
    print( "---------------------------------------------" );
    print( "pc: " )
    print( ["%5.2f" % element[0,0] for element in pc] )      
    print( "---------------------------------------------" );
    print( "pv: " )
    print( ["%5.2f" % element for element in pv] )

# end function printResults

X = numpy.matrix(\
    '-2, 1, 4, 6, 5, 3, 6, 2;\
    9, 3, 2, -1, -4, -2, -4, 5;\
    0, 7, -5, 3, 2, -3, 4, 6');
    
k = 1;
            
[rep, pc, pv] = mypca( X, k );
printResults( rep, pc, pv ); 
