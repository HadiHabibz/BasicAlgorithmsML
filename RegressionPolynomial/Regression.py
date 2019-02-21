import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import lineStyles

class Regression():
    'Polynomial curve fitting using regression'
    
    # Constructor
    def __init__( self, filename ):
        
        # The input data. These are the observations
        # x is an m by n matrix, where m is the number
        # of samples and n is the number of features
        self.x = np.array( [], float );
        
        # The actual output. y is ground truth. It 
        # should be a vector with m rows, where m
        # is the number of samples in the training
        # dataset
        self.y = np.array( [], float );
        
        # Parameters (typically referred to theta in 
        # other references). This is a vector with 
        # d + 1 rows, where d is the degree of the 
        # polynomial. The additional input is for 
        # the intercept.
        self.p = np.array( [], float );
        
        # This is a m by d+1 matrix, where 
        # m is the number of observations in
        # the training set and d is the degree
        # of the polynomial. The element ij in
        # this matrix is computed as (x_i)^(j-1).
        self.X = np.array( [], float );
        
        # The output approximated for each sample
        # by regression
        self.ybar = np.array( [], float );
        self.loadDataFromFile( filename )
    
    # Load all data from the file
    def loadDataFromFile( self, filename ):
        
        data = open( filename, "r" );
        
        for line in data:
            [x, y] = line.split();
            self.x = np.append( self.x, float( x ) );
            self.y = np.append( self.y, float( y ) );

    # Plot results
    # Actual data are shown by filled bubbles
    # The approximation is shown by a solid curve
    def plotResults( self, norm ):
        
        plt.figure( );
        plt.plot( self.x, self.y, 'o' );
        plt.plot( self.x, self.ybar, linewidth=2.0 )
        plt.ylabel( "Y" );
        plt.xlabel( "X" );
        plt.title( "Error: " + "%0.2f" % ( self.computeError( 2 ) ) +
                   " - Curve Degree: " + str(  self.p.shape[0] - 1  ) + 
                   "\nParameters: [" + ", ".join( ["%0.1f" % p for p in self.p] ) + 
                   "]" )
        plt.show();
    
    # Compute matrix X. When using polynomials, the 
    # entry ij is computed as (x_i)^(j-1).
    # This only works for polynomial with a given degree
    def computeX_polynomial( self, degree ):
        
        if degree <= 0:
            degree = 1;
        
        # Add one to include the intercept
        degree += 1;
        xMatrix = [];
        
        for x in self.x:
            newRow = [x**i for i in range( degree )];
            xMatrix.append( newRow )
            
        self.X = np.array( xMatrix );
        
    # Compute p using the following formula
    # P = ((X^T)X)^(-1)X^Ty 
    def computeP( self ):
        Xinv = np.linalg.inv( ( np.dot(
             np.transpose( self.X ), self.X ) ) );
             
        self.p = np.dot( np.dot(
             Xinv, np.transpose( self.X ) ), self.y );
        
    # Compute ybar using the following formula
    # ybar = XP
    def computeYBar( self ):
        self.ybar = np.dot( self.X, self.p );    
    
    # Compute the error. Use norm 2 by default
    # error = || y - ybar ||_{norm}
    def computeError( self, norm = 2 ):
        return np.linalg.norm( self.y - self.ybar, norm )
    
    def printAndPlotResults( self, degree, norm ):
        
#         print( "Regression using degree " +
#                 str( degree ) + " polynomial." );
#                 
#         print( "Error --> " +
#                 "%0.1f" % regression.computeError( norm) );
#                 
#         print( "Parameters:", end = "" );
#         
# 
#         print( ["%0.2f" % p for p in self.p] );
#         
        regression.plotResults( norm );
#         print( "------------------------------------------" );
        
    # API provided to the user
    def fitCurve( self, degree = 1, norm = 2 ):
        
        self.computeX_polynomial( degree );
        self.computeP();
        self.computeYBar();
        self.printAndPlotResults( degree, norm );
            

degrees = [1, 3, 5, 7];
norm = 2;

for degree in degrees:
    regression = Regression( "hw2.dat" ) 
    regression.fitCurve( degree, norm );