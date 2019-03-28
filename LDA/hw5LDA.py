import numpy;
from scipy.linalg import eigh;

class LDA:
    '1D and binary classification using Fisher\
    linear discriminant'
    
    # Constructor
    def __init__( self ):
        
        # Data we want to classify
        # It must be a matrix with f rows
        # and n columns, where f is the 
        # number of features and n is the
        # number of samples
        self.data = None;
        
        # Data used to train the first 
        # class. It must be a matrix with
        # f rows and np columns, where np is
        # the number of training samples in
        # the class. This corresponds to Xp
        self.dataPositive = None;
        
        # Data used to train the second class.
        # It must be a matrix with f rows and
        # nn columns, where nn is the number of
        # samples in the second class. This 
        # corresponds to Xn
        self.dataNegative = None;
        
        # The mean of dataPositive
        # This is used for training only
        # This corresponds to mu_p
        self.meanVectorPositive = None;
        
        # The mean of dataNegative
        # This is used for training only
        # This corresponds to mu_n
        self.meanVectorNegative = None;
        
        # The covariance matrix of dataPositive
        # This is used for training only        
        # This corresponds to Sp
        self.covarianceMatrixDataPositive = None;
        
        # The covariance matrix of dataNegative
        # This is only used for training
        # This corresponds to Sn
        self.covarianceMatrixDataNegative = None;
        
        # This corresponds to Sb
        self.betweenClassScatteringMatrix = None;
        
        # This corresponds to Sw
        self.withinClassScatteringMatrix = None;
        
        # This corresponds to S
        self.overallCovarianceMatrix = None;
        
        # The optimal projection that maximizes
        # the discrimination between the two classes
        self.optimalProjection = [];
        
        # The optimal threshold
        self.threshold = 0;
        
        # Should we look left or right of the
        # threshold to determine positive or 
        # negative data?
        self.iequalityDirection = 0;
        
    # end constructor
    
    # Load classification data
    def loadClassificationData( self, data = [] ):
        
        # Make sure the input is not empty
        if len( data ) != 0:
            self.data = data;
            return;
        
    # end function loadClassificatioData
    
    # Load training data
    # Input data pertaining to each class separately
    def loadTrainingData( self, dataPositive = [],\
        dataNegative = [] ):
    
        # Make sure the input is not empty
        if len( dataPositive ) != 0 and\
           len( dataNegative ) != 0:
               
            self.dataPositive = dataPositive;
            self.dataNegative = dataNegative;
            return;
        
        # end if
        
    # end function loadTrainingData
    
    # Center data around the origin by subtracting the mean
    def centerData( self ):
        
        [self.meanVectorPositive, self.meanVectorNegative] =\
            self.computeMean();
            
        self.dataPositive -= numpy.transpose(\
            numpy.array( self.meanVectorPositive, ndmin = 2 ) );
        self.dataNegative -= numpy.transpose(\
            numpy.array( self.meanVectorNegative, ndmin = 2 ) );
        
    # end function centerData
    
    # Compute the mean of data
    def computeMean( self ):
        
        return \
            [numpy.mean( self.dataPositive, axis = 1 ),\
            numpy.mean( self.dataNegative, axis = 1 )];
        
    # end function computeMean
    
    # Compute covariance matrix for 
    # both classes. I use Sp and Sn in
    # the comments to refer to the covariance
    # matrices of positive and negative
    # classes, respectively.
    def computeCovarianceMatrices( self ):
        
        self.covarianceMatrixDataPositive = \
            numpy.matmul( self.dataPositive,\
            numpy.transpose( self.dataPositive ) );
                         
        self.covarianceMatrixDataNegative = \
            numpy.matmul( self.dataNegative,\
            numpy.transpose( self.dataNegative ) );
        
    # end function computeCovarianceMatrices
    
    # Compute between class scattering matrix 
    # Defined as Sb = (deltaMu) * (deltaMu)T, where
    # deltaMu is the difference between the mean
    # vector of two classes.
    def computeBetweenClassScatteringMatrix( self ):
        
        deltaMean = self.meanVectorPositive -\
            self.meanVectorNegative;
        
        # Make it a column vector
        deltaMean = numpy.transpose(\
            numpy.array( deltaMean, ndmin = 2 ) );
        
        # We transpose twice, but this keeps it 
        # consistent with analytic formulas
        self.betweenClassScatteringMatrix = numpy.matmul(\
            deltaMean, numpy.transpose( deltaMean ) );
                                                    
    # end function computeBetweenClassScatteringMatrix
    
    # Compute within class scattering matrix
    # defined as Sw = (np/n) * Sp + (nn/n) * Sn
    def computeWithinlassScatteringMatrix( self ):
        
        # Get the total number of samples
        totalSamplesCount = self.dataPositive.shape[1] + \
            self.dataNegative.shape[1];
            
        alpha1 = self.dataPositive.shape[1] / totalSamplesCount;
        alpha2 = self.dataNegative.shape[1] / totalSamplesCount;
        
        self.withinClassScatteringMatrix =\
            alpha1 * self.covarianceMatrixDataPositive + \
            alpha2 * self.covarianceMatrixDataNegative;
    
    # end computeWithinlassScatteringMatrix
    
    # Do eigen value decomposition. Because averages
    # are 1D, the rank will be 1 and there will be 
    # exactly one non-zero eigen value
    def computeEigenValueDecomposition( self ):
        
        # Must return exactly one none-zero eigen value
        [w, v] = eigh( self.betweenClassScatteringMatrix,\
            self.withinClassScatteringMatrix );
        
        # Pick the eigen vector corresponding to only 
        # non-zero eigen value.
        eigenValueIndex = numpy.argmax( w );
        eigenValue = w[eigenValueIndex]; 
        eigenVector = v[:,eigenValueIndex];
        eigenVector = numpy.array( eigenVector, ndmin = 2 );
        self.optimalProjection = numpy.transpose( eigenVector );
        
        return [eigenValue, self.optimalProjection];
        
    # end function computeEigenValueDecomposition
    
    # Compute the threshold that maximizes the 
    # classification accuracy for the training data.
    # Use a hill-climbing approach. 
    def computeThreshold( self ):
        
        numberOfSteps = 20;
         
        # Project the training data onto the optimal vector                                          
        [projectionPositive, projectionNegative,\
            projectedMeanPositive, projectedMeanNegative] =\
            self.projectTrainingData();
    
        # Should we look to the left or right?
        # It depends on the projected mean of the 
        # two clusters
        if projectedMeanPositive >= projectedMeanNegative:
            self.inequalityDirection = 0;
            
        else:
            self.inequalityDirection = 1;
            
        # Find the range of the training data
        # which determines the range in which
        # the optimal threshold lies. 
        minValue = numpy.min( numpy.append(\
            projectionNegative, projectionPositive,\
            axis = 1 ) );

        maxValue = numpy.max( numpy.append(\
            projectionNegative, projectionPositive,\
            axis = 1 ) );
            
        # The step size in hill climbing algorithm
        scale = ( maxValue - minValue ) / numberOfSteps;
        
        correctGuesses = numpy.zeros( shape = ( numberOfSteps+1, 1 ) );
        index = 0;
        currentValue = minValue;
        
        # Sweep the range and find the performance of each
        # threshold
        while currentValue < maxValue:
            
            # The powers of (-1) are used to control the direction
            # of inequality
            correctGuesses[index] = \
            numpy.sum( (-1)**self.inequalityDirection * projectionPositive\
                      > (-1)**self.inequalityDirection * currentValue ) +\
            numpy.sum( (-1)**self.inequalityDirection * projectionNegative\
                      <= (-1)**self.inequalityDirection * currentValue );
            
            index += 1;
            currentValue += scale;
            
        # end while
        
        # Pick the first best threshold
        # and check if it is unique or not.
        # If not, pick the threshold in the middle.
        maxIndexFirst = numpy.argmax( correctGuesses );
        strike = maxIndexFirst;
        strikeLength = 1;
        
        while True:
            
            strike += 1;
            
            if correctGuesses[strike] < correctGuesses[maxIndexFirst]:
                break;
                
            strikeLength += 1;
            
        # end while 
        
        # Can be at most one step off
        self.threshold = ( maxIndexFirst +\
            numpy.floor( strikeLength/2 ) ) *\
            scale + minValue;
        
    # end function computeThreshold 
    
    # Project the training data and their means
    # to the optimal vector
    def projectTrainingData( self ):
        
        projectionPositive =\
            numpy.matmul( numpy.transpose( self.optimalProjection ),\
            self.dataPositive + numpy.transpose( numpy.array(\
            self.meanVectorPositive, ndmin = 2 ) ) );
                                 
        projectionNegative =\
            numpy.matmul( numpy.transpose( self.optimalProjection ),\
            self.dataNegative + numpy.transpose( numpy.array(\
            self.meanVectorNegative, ndmin = 2 ) ) );
                         
        projectedMeanPositive = numpy.matmul(\
            numpy.transpose( self.optimalProjection ),\
            numpy.transpose( numpy.array(\
            self.meanVectorPositive, ndmin = 2 ) ) );
                                             
        projectedMeanNegative = numpy.matmul(\
            numpy.transpose( self.optimalProjection ),\
            numpy.transpose( numpy.array(\
            self.meanVectorNegative, ndmin = 2 ) ) );  
                                             
        return [projectionPositive, projectionNegative,\
                projectedMeanPositive,projectedMeanNegative];
        
    # end function projectData
    
    # Train the classifier based on the the
    # training data.
    def train( self ):
        
        self.centerData();
        self.computeCovarianceMatrices();
        self.computeBetweenClassScatteringMatrix();
        self.computeWithinlassScatteringMatrix();
        
        [eigenValues, eigenVectorMatrix] =\
            self.computeEigenValueDecomposition();
        
        self.computeThreshold();
        
        return self.optimalProjection;
        
    # end function train
    
    # Classify data
    def classify( self ):
        
        if len( self.optimalProjection ) == 0:
            print( "Could not classify! Train the classifier first." );
            return;
            
        projectedData = numpy.matmul(\
            numpy.transpose( self.optimalProjection ),\
            self.data );
           
        # The multiplication and division is a basic trick
        # to convert True and False to +1 and -1 respectively.
        # If the data sample belongs to the positive cluster,
        # return 1, otherwise return -1;
        if self.inequalityDirection == 1:
            results = 2 * ( projectedData <= self.threshold ) - 1;
            
        else:
            results = 2 * ( projectedData > self.threshold ) - 1;   
        
        return results;     
        
    # end function classify
    
# end class
    
# Training data
dataPositive = numpy.array( [\
    [4, 2, 2, 3, 4, 6, 3, 8],\
    [1, 4, 3, 6, 4, 2, 2, 3],\
    [0, 1, 1, 0, -1, 0, 1, 0]],\
    dtype = numpy.float64 );

dataNegative = numpy.array( [\
    [9, 6, 9, 8, 10],\
    [10, 8, 5, 7, 8],\
    [1, 0, 0, 1, -1]],\
    dtype = numpy.float64 );
               
# Classification data
data = numpy.array( [\
    [1.3, 2.4, 6.7, 2.2, 3.4, 3.2],
    [8.1, 7.6, 2.1, 1.1, 0.5, 7.4],
    [-1, 2.0, 3.0, -2.0, 0.0, 2.0]],\
    dtype = numpy.float64 );
 
# Do your thing!
worker = LDA();
worker.loadTrainingData( dataPositive, dataNegative );
worker.loadClassificationData( data );
optimalProjection = worker.train();
results = worker.classify();
print( results );