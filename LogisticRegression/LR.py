import numpy;
import matplotlib.pyplot as plt;
import time;
from fileinput import filename;

class LogisticRegression:
    'Binary classification using logistic regression'
    
    def __init__( self ):
        
        # The number of features including 
        # the intercept, which is appended
        # to the data later on
        self.featuresCount = 0;
        
        # Number of samples in the 
        # training data
        self.samplesCount = 0;
        
        # The matrix holding training
        # data. It should be N by P,
        # where N is the number of 
        # samples and P is the feature
        # size.
        self.trainingData = None;
        
        # Labels associated with each
        # training sample. It must be
        # a vector of size N. The i-th
        # entry is the label of sample
        # i. Labels should be either 1
        # or 0.
        self.labels = None;
        
        # The parameters of the hyperplane
        # separating two classes. It is
        # a vector and it contains P+1 
        # elements. The additional parameter
        # is for the intercept
        self.weight = None;
        
        # Basically, determines how small the
        # gradient must be to stop searching
        self.convergenceThreshold = 1e-6;
        
    # end constructor 
    
    # Load training data from give file
    def loadTrainingData( self, filename ):
        
        trainingFile = open( filename, "r" );
        self.trainingData = self.loadData( trainingFile );
        
        # The extra dimension is for the intercept
        self.featuresCount = self.trainingData.shape[1] + 1;
        self.samplesCount = self.trainingData.shape[0];
    
    # end function loadTrainingData
    
    # Perform training
    # The output is the parameters of self.weight
    # data member, which defines a hyper-plane 
    def train( self ):
        
        # Add 1 to the very first column of the
        # data for the intercept. Hence, the first
        # parameter in self.weight is the intercept.
        extendedData = self.appendIntercept();
        
        # Learn the parameters of the optimal hyper-plane
        # using gradient descent.
        self.optimizeWeight_GradientDescent( extendedData );
        self.plot();
        
        # Learn the parameters of the optimal hyper-plane
        # using Newton's method
        self.optimizeWeight_Newton( extendedData );
        self.plot();

    # end function train
    
    # Perform optimization using hill climbing
    # The function is concave having one global
    # minimum
    def optimizeWeight_GradientDescent( self, extendedData ):
        
        # Convert [0,1] labels to [-1,1]
        labels = self.labels * 2 - 1;
        
        # Pick a random value for weight
        self.weight = numpy.zeros( extendedData[0].shape );
        
        # Increasing stepSize reduces the difference 
        # between estimated minimum and actual minimum.
        # It reduces the convergence speed though!
        stepSize = 1;
        
        # Not necessary, but it is nice to know
        # the number of iterations
        iterationCounter = 0;
        
        start = time.time();
        
        # After each iteration, self.weight converges to 
        # its minimum
        while True:
            
            # Compute the gradient of the objective function
            exponent = numpy.matmul( self.weight, \
                numpy.transpose( extendedData ) );
                   
            exponent = numpy.multiply( labels[:,0],\
                exponent );
                
            exponent = numpy.exp( exponent ) + 1;          
            exponent = 1 / exponent; 
            exponent = ( labels[:,0] * exponent );
                   
            gradientPerData = numpy.reshape( exponent,\
                ( self.samplesCount, 1 ) ) *\
                extendedData;
                
            totalGradient = sum( gradientPerData ) /\
                self.samplesCount;
             
            # Move toward a direction that lowers the
            # gradient                  
            self.weight = self.weight +\
                stepSize * totalGradient;
            
            iterationCounter += 1;
            
            # Break if gradient is close to zero
            if numpy.linalg.norm( totalGradient ) <=\
                self.convergenceThreshold:
                break;
            
        # end while
        
        stop = time.time();
        
        print( " Gradient Descent:\n %d Steps in %0.2f ms"\
            % ( iterationCounter, ( stop - start ) * 1000 ) );
        print( " Estimated Weight Matrix (w1.x1 = -w2.x2 - w0):\n",\
            ["%0.3f" % w for w in self.weight] );
        print( "----------------------------------------------" );
            
    # end function 
    
    # Perform optimization using Newton's method
    def optimizeWeight_Newton( self, extendedData ):
        
        # Start from a random point say zero
        self.weight = numpy.zeros( extendedData[0].shape );
        
        # Not necessary but good to know
        iterationCounter = 0;
         
        start = time.time();
               
        while True:
            
            # keep track of the previous step, needed
            # to determine the convergence of the algorithm
            oldWeight = self.weight;
            
            # Find the ratio of first and second derivatives
            numerator = numpy.exp( numpy.matmul( self.weight,\
                numpy.transpose( extendedData ) ) );
                
            denominator = 1 + numerator;
            
            probability = numpy.divide( numerator, denominator );
            weight = numpy.diag( numpy.multiply( probability,\
                1-probability ) );
            
            response = numpy.matmul( extendedData, self.weight ) +\
                numpy.matmul( numpy.linalg.inv( weight ),\
                self.labels[:,0] - probability );
                
            self.weight = numpy.matmul( numpy.transpose(\
                extendedData ), weight );
                
            self.weight = numpy.matmul( self.weight, extendedData );
            self.weight = numpy.linalg.inv( self.weight );
            
            self.weight = numpy.matmul( self.weight,\
                numpy.transpose( extendedData ) );
                
            self.weight = numpy.matmul( self.weight, weight );
            self.weight = numpy.matmul( self.weight, response );
            
            iterationCounter += 1;
            
            if numpy.linalg.norm( oldWeight - self.weight ) <\
                self.convergenceThreshold:
                break;
            
            # end while
            
        stop = time.time();
        
        print( " Newton's Method:\n %d Steps in %0.2f ms"\
            % ( iterationCounter, ( stop - start ) * 1000 ) );
        print( " Estimated Weight Matrix (w1.x1 = -w2.x2 - w0):\n",\
            ["%0.3f" % w for w in self.weight] );
        print( "----------------------------------------------" );        
    # end function optimizeWeight
    
    # Add one to the first column of each data sample
    # This accounts for the intercept 
    def appendIntercept( self ):
        
        extendedData = numpy.ones( [self.trainingData.shape[0],\
            self.trainingData.shape[1] + 1] );    
        
        extendedData[:, 1:] = self.trainingData;
        
        return extendedData;
            
    # end appendIntercept
    
    # Load lables
    def loadLabels( self, filename ):
        
        labelsFile = open( filename, "r" );
        self.labels = self.loadData( labelsFile );
    
    # end loadLabels
    
    # Load data
    def loadData( self, dataFile ):
    
        # Go through the file to see how many
        # entries there are. Each line holds 
        # the feature for one sample so the 
        # number of lines is the same as the 
        # number of samples.
        self.samplesCount = len( dataFile.readlines() );
        
        # Go back to the beginning of the file
        dataFile.seek( 0, 0 );
        
        # Get the feature size. Read the first line
        # and see how many numbers are separated by
        # space. 
        self.featuresCount = len( dataFile.readline().split() );
        
        # Allocate memory for data based on the number
        # of features and samples. This is done to avoid
        # dynamically appending data, hence improving
        # the performance
        dataMatrix = numpy.zeros( shape =\
        ( self.samplesCount, self.featuresCount ) );
                      
        sampleCounter = 0;
        dataFile.seek( 0, 0 );
        
        # Each the file line by line and
        # load data to memory
        for line in dataFile:
            dataMatrix[ sampleCounter, :] = line.split();
            sampleCounter = sampleCounter + 1;
        
        # end for
        
        return dataMatrix;
    
    # end function loadData  
    
    # This is a very bad plotting function 
    # Works only for the given dataset
    # Everything is hard-coded
    def plot( self ):
        
        # Plot the hyper-plane in the given range
        x2 = numpy.arange( 600 ) - 600/2;
        x2 = x2 / 100;
        
        x1 = self.weight[2] * x2 + self.weight[0];
        x1 = -x1 / self.weight[1];
              
        plt.figure();
        
        plt.plot( self.trainingData[0:49, 0],\
            self.trainingData[0:49, 1], "go",\
            alpha = 0.7, label = "Label 0" );
            
        plt.plot( self.trainingData[50:, 0],
            self.trainingData[50:, 1], "bs",\
            alpha = 0.7, label = "Label 1" );
            
        plt.plot( x1, x2, 'r-', linewidth = 3,
            alpha = 0.5, label = "Hyper-plane" );
            
        plt.ylabel( "X_2" );
        plt.xlabel( "X_1" );
        
        plt.legend()
        
        
    # end plot
    
# end class

worker = LogisticRegression();
worker.loadTrainingData( "hw6x.dat" );
worker.loadLabels( "hw6y.dat" );
worker.train();
plt.show();