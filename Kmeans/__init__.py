import numpy as np;
import matplotlib.pyplot as plt;

class KMeans():
    'Cluster data to K groups'

    # Constructor
    def __init__( self ):
        
        # The number of clusters. 
        # This is a hyper-parameter
        self.k = 1;
        
        # The number of features
        self.featuresCount = 0;
        
        # The number of samples
        self.samplesCount = 0;
        
        # The norm used for computing the
        # distance from centroids
        self.norm = 2;
        
        # A hyper-parameter that determines
        # the convergence of the algorithm.
        # This number should be between 0 and
        # 1. The closer it is to zero, the 
        # more iterations the algorithm will
        # go through before it terminates.
        self.convergenceAccuracy = 1e-15;
        
        # S by F data, where S is the number of
        # samples and F is the number of features
        self.data = None;
        
        # The coordinates of centroids
        # This is a k by F matrix, where 
        # k is the number of clusters and
        # F is the number of features
        self.centroids = None;
        
        # A S by 2 matrix, where S is the 
        # number of samples. This variable
        # shows which cluster every data
        # belongs to. Each entry s is associated
        # with two number in the second dimension.
        # The first one determines the cluster data
        # sample s belongs to while the second 
        # shows its distance from centroid
        # Computed as l-norm squared.
        self.assignedClusters = None;
    
    # end constructor

    # Load data from the file
    # The data must be saved in a S by F 
    # matrix in a txt file. S is the
    # number of samples, while F is the 
    # number of features. The features 
    # must be separated with a space.
    def loadData( self, filename ):
    
        # Open file
        data = open( filename, "r" );
        
        # Go through the file to see how many
        # entries there are. Each line holds 
        # the feature for one sample so the 
        # number of lines is the same as the 
        # number of samples.
        self.samplesCount = len( data.readlines() );
        
        # Go back to the beginning of the file
        data.seek( 0, 0 );
        
        # Get the feature size. Read the first line
        # and see how many numbers are separated by
        # space. 
        self.featuresCount = len( data.readline().split() );
        
        # Allocate memory for data based on the number
        # of features and samples. This is done to avoid
        # dynamically appending data, hence improving
        # the performance
        self.data = np.zeros( shape =\
        ( self.samplesCount, self.featuresCount ) );
        
        self.assignedClusters =\
        np.zeros( shape = ( self.samplesCount, 2 ) );
              
        sampleCounter = 0;
        data.seek( 0, 0 );
        
        # Each the file line by line and
        # load data to memory
        for line in data:
            self.data[ sampleCounter, :] = line.split();
            sampleCounter = sampleCounter + 1;
        
    # end function loadData
    
    # Pick random data samples as centroids
    def initializeCentroids( self ):
        
        #np.random.seed(12349)
        
        # Allocate memory based on number of cluster
        # and the feature size. 
        self.centroids = np.zeros(\
            shape = ( self.k, self.featuresCount ) );
    
        randomizedIndex = np.arange( self.samplesCount );
        np.random.shuffle( randomizedIndex );
                
        # Set the coordinate for the centroids one by one
        for featureIndex in range( self.featuresCount ):
            
            # Make sure centroids are initially somewhere 
            # close to the data. This is not required but
            # might help with the performance
            featureMin = min( self.data[:, featureIndex] );
            featureMax = max( self.data[:, featureIndex] );
            
            for centroid in range( self.k ):
                self.centroids[centroid, featureIndex] =\
                    self.data[randomizedIndex[centroid],\
                    featureIndex];
                
            # end for centroid
            
        # end for featureIndex     
                
    # end function initializeCentroids
    
    # Determine the cluster each data sample
    # belongs to 
    def determineClusters( self ):
         
        indexCounter = 0;
        
        for data in self.data:
            
            # Find the centroid closest to the 
            # data and assign the cluster associated
            # with that centroid to the data sample
            self.assignedClusters[indexCounter, :] =\
            self.getClosestCentroid( data );
            
            indexCounter += 1;
             
        # end for data
         
    # end function determineClusters

    # Find the centroid that is closes the 
    # given data sample. Also return the 
    # distance between data sample and its
    # closest centroid. The distance is 
    # used later for detecting the convergence
    # and the performance of the algorithm
    def getClosestCentroid( self, data ):
        
        # Keep how far each sample is from every 
        # centroid
        distances = np.zeros( shape = ( self.k ) );
        
        indexCounter = 0;
        
        for centroid in self.centroids:
   
            distances[indexCounter] = \
            np.linalg.norm( data - centroid, self.norm );             
            indexCounter = indexCounter + 1;    
        
        return np.array( [np.argmin( distances ),\
        np.min( distances )] );
        
    # end getClosestCentroid   
    
    # Update the centroid after re-assigning the
    # data samples
    # The algorithm returns True if it detects
    # a convergence. A convergence is detected
    # if none of the existing centroids move
    # that much (based on a give threshold). 
    # The algorithm can get stuck in a local 
    # extremum. 
    def reEvaluateCentroids( self ):
        
        centroidIndex = 0;

        # Old distance is used to detect the 
        # convergence. Later on in this function
        # this is compared with new distance
        # If the difference is less than a threshold
        # the algorithm terminates.
        oldDistances = self.getDistance();
        
        # Before reevaluating centroid, make
        # sure new clusters are assigned to data
        # samples
        self.determineClusters();
                
        # Compute the center of the new cluster
        for centroid in self.centroids:
            
            # Only work with samples that are in 
            # relevant cluster    
            dataInClusterLocation = np.array(\
            [self.assignedClusters[:,0] == centroidIndex] );
            
            dataInClusterLocation = np.transpose( dataInClusterLocation );
            samplesCount = np.sum( dataInClusterLocation );
            
            # Avoid division by zero
            if( samplesCount == 0 ):
                break;
            
            newX = np.dot( self.data[:, 0], dataInClusterLocation ) / samplesCount;
            newY = np.dot( self.data[:, 1], dataInClusterLocation ) / samplesCount;            
            self.centroids[centroidIndex, :] = [newX, newY] ;
            centroidIndex += 1;
            
        # end for centroid
        
        newDistances = self.getDistance();
        difference = np.abs( newDistances - oldDistances );
        
        if any( difference[:] > self.convergenceAccuracy ):
            return False;
         
        return True;
            
    # end function reEvaluateCentroids
    
    # Compute the objective function of k means
    # This returns a vector, where the ith element
    # is the total sum of l-norm distance of all 
    # samples in the ith cluster from their centroid.
    def getDistance( self ):
        
        distance = np.zeros( shape = ( self.k ) );
        
        for data in self.assignedClusters:
            distance[int( data[0] )] += data[1];
            
        # end for
        
        return distance;
        
    # end function getDistance
    
    # Plot results. Works only with 2D data.
    def plot( self, i = 1 ):
        
        plt.figure( i );
        #plt.plot( self.centroids[:,1], self.centroids[:,0],  );
        distance = self.getDistance();
        colors = ["b", "g", "r", "c", "m", "b"];
        
        for centroidIndex in range( self.centroids.shape[0] ):
            presenceFlag = self.assignedClusters[:,0] == centroidIndex;
            
            dataIndex = 0;
            dataX = [];
            dataY = [];
            
            for flag in presenceFlag:
                if flag == True:
                    dataX.append( self.data[dataIndex, 0] );
                    dataY.append( self.data[dataIndex, 1] );
                    
                dataIndex += 1;
            
            plt.plot( dataY, dataX, "o", label='cluster' +\
            str( centroidIndex ), alpha=.6, color = colors[centroidIndex]  );
            
            plt.plot( self.centroids[centroidIndex, 1], \
            self.centroids[centroidIndex, 0], "*", markersize = 15,\
            color = colors[centroidIndex] );
            
        plt.title( "K = " + str( self.k ) + "\nDistance = [" +\
        " ".join( ["%0.1f" % d for d in distance] ) + "]" );
        plt.xlabel( "X" );
        plt.ylabel( "Y" );
        plt.legend( loc='upper right' );        
        plt.draw();
        
    # end function plot
    
    # Keep reevaluating the centroids until
    # the program converges.
    def clusterify( self, filename, k ):
        
        if k > 0:
            self.k = k;
            
        self.loadData( filename );
        self.initializeCentroids();
 
        iterationsCounter = 0;
        
        # The function returns true if a convergence
        # is detect. 
        while self.reEvaluateCentroids() == False:
            iterationsCounter += 1;              
                    
        # Count the number of iterations
        # Depends on the data and initial selection
        # of the centroids.
        print( str( iterationsCounter ), " iterations!" )
        self.plot(k);
        
    # end function clusterify
    
# end class KMeans

clustersCount = [2, 3, 4, 5];

for k in clustersCount:
    worker = KMeans();
    worker.clusterify( "hw3.dat", k );
        
plt.show();
