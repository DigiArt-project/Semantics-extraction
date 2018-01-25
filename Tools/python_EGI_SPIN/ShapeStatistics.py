#Programmer: Chris Tralie
#Purpose: Reference solutions for group assignment 2
import sys
sys.path.append("S3DGLPy")
from Primitives3D import *
from PolyMesh import *

import numpy as np
import matplotlib.pyplot as plt

POINTCLOUD_CLASSES = ['biplane', 'desk_chair', 'dining_chair', 'fighter_jet', 'fish', 'flying_bird', 'guitar', 'handgun', 'head', 'helicopter', 'human', 'human_arms_out', 'potted_plant', 'race_car', 'sedan', 'shelves', 'ship', 'sword', 'table', 'vase']

NUM_PER_CLASS = 10

#########################################################
##                UTILITY FUNCTIONS                    ##
#########################################################

#Purpose: Export a sampled point cloud into the JS interactive point cloud viewer
#Inputs: Ps (3 x N array of points), Ns (3 x N array of estimated normals),
#filename: Output filename
def exportPointCloud(Ps, Ns, filename):
    N = Ps.shape[1]
    fout = open(filename, "w")
    fmtstr = "%g" + " %g"*5 + "\n"
    for i in range(N):
        fields = np.zeros(6)
        fields[0:3] = Ps[:, i]
        fields[3:] = Ns[:, i]
        fout.write(fmtstr%tuple(fields.flatten().tolist()))
    fout.close()

#Purpose: To sample a point cloud, center it on its centroid, and
#then scale all of the points so that the RMS distance to the origin is 1
def samplePointCloud(mesh, N):
    (Ps, Ns) = mesh.randomlySamplePoints(N)
    ##TODO: Center the point cloud on its centroid and normalize
    #by its root mean square distance to the origin.  Note that this
    #does not change the normals at all, only the points, since it's a
    #uniform scale
    Ps = Ps - np.mean(Ps, 1, keepdims=True)
    Ps = Ps*np.sqrt(Ps.shape[1]/np.sum(Ps**2))
    return (Ps, Ns)

# Returns a 3 x 1 matrix
def getCentroid(PC):
    # mean of column vectors (axis 1) 
    return np.mean(PC, 1, keepdims=True)

#Purpose: To sample the unit sphere as evenly as possible.  The higher
#res is, the more samples are taken on the sphere (in an exponential 
#relationship with res).  By default, samples 66 points
def getSphereSamples(res = 2):
    m = getSphereMesh(1, res)
    return m.VPos.T

#Purpose: To compute PCA on a point cloud
#Inputs: X (3 x N array representing a point cloud)
def doPCA(X):
    #TODO: Fill this in for a useful helper function
    C = X.dot(X.T)
    [eigs, V] = np.linalg.eig(C)
    idx = np.argsort(-eigs)
    V = V[:, idx]
    eigs = eigs[idx]
    #eigs = np.array([1, 1, 1]) #Dummy value
    #V = np.eye(3) #Dummy Value
    return (eigs, V)

#########################################################
##                SHAPE DESCRIPTORS                    ##
#########################################################

#Purpose: To compute a shape histogram, counting points
#distributed in concentric spherical shells centered at the origin
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals) (not needed here
#but passed along for consistency)
#NShells (number of shells), RMax (maximum radius)
#Returns: hist (histogram of length NShells)
def getShapeHistogram(Ps, Ns, NShells, RMax):
    print("My Shape histogram")
    hist = np.zeros(NShells)
    ##TODO: Finish this; fill in hist
    PDists = np.sqrt(np.sum(Ps**2, 0))
    Rs = np.linspace(0, RMax, NShells+1)
    for i in range(0, NShells):
        hist[i] = np.sum((PDists>=Rs[i])*(PDists<Rs[i+1]))
    return hist
    
#Purpose: To create shape histogram with concentric spherical shells and
#sectors within each shell, sorted in decreasing order of number of points
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals) (not needed here
#but passed along for consistency), NShells (number of shells), 
#RMax (maximum radius), SPoints: A 3 x S array of points sampled evenly on 
#the unit sphere (get these with the function "getSphereSamples")
def getShapeShellHistogram(Ps, Ns, NShells, RMax, SPoints):
    NSectors = SPoints.shape[1] #A number of sectors equal to the number of
    #points sampled on the sphere
    #Create a 2D histogram that is NShells x NSectors
    hist = np.zeros((NShells, NSectors))
    ##TODO: Finish this; fill in hist, then sort sectors in descending order
    PDists = np.sqrt(np.sum(Ps**2, 0))
    Rs = np.linspace(0, RMax, NShells+1)
    for i in range(0, NShells):
        PSub = Ps[:, (PDists>=Rs[i])*(PDists<Rs[i+1])]
        scores = (PSub.T).dot(SPoints)
        idx = np.argmax(scores, 1)
        for k in range(NSectors):
            hist[i, k] = np.sum(idx == k)
    hist = np.sort(hist, 1)
    return hist.flatten() #Flatten the 2D histogram to a 1D array

#Purpose: To create shape histogram with concentric spherical shells and to 
#compute the PCA eigenvalues in each shell
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals) (not needed here
#but passed along for consistency), NShells (number of shells), 
#RMax (maximum radius), sphereRes: An integer specifying points on thes phere
#to be used to cluster shells
def getShapeHistogramPCA(Ps, Ns, NShells, RMax):
    #Create a 2D histogram, with 3 eigenvalues for each shell
    hist = np.zeros((NShells, 3))
    ##TODO: Finish this; fill in hist
    PDists = np.sqrt(np.sum(Ps**2, 0))
    Rs = np.linspace(0, RMax, NShells+1)
    for i in range(0, NShells):
        PSub = Ps[:, (PDists>=Rs[i])*(PDists<Rs[i+1])]
        hist[i, :] = doPCA(PSub)[0]
    return hist.flatten() #Flatten the 2D histogram to a 1D array

#Purpose: To create shape histogram of the pairwise Euclidean distances between
#randomly sampled points in the point cloud
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals) (not needed here
#but passed along for consistency), DMax (Maximum distance to consider), 
#NBins (number of histogram bins), NSamples (number of pairs of points sample
#to compute distances)
def getD2Histogram(Ps, Ns, DMax, NBins, NSamples):
    N = Ps.shape[1]
    P1 = Ps[:, np.random.random_integers(0, N-1, NSamples)]
    P2 = Ps[:, np.random.random_integers(0, N-1, NSamples)]
    D = np.sqrt(np.sum((P1-P2)**2, 0))
    hist = np.histogram(D, bins=NBins, range=(0, DMax))[0]
    return hist

#Purpose: To create shape histogram of the angles between randomly sampled
#triples of points
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals) (not needed here
#but passed along for consistency), NBins (number of histogram bins), 
#NSamples (number of triples of points sample to compute angles)
def getA3Histogram(Ps, Ns, NBins, NSamples):
    N = Ps.shape[1]
    P1 = Ps[:, np.random.random_integers(0, N-1, NSamples)]
    P2 = Ps[:, np.random.random_integers(0, N-1, NSamples)]    
    P3 = Ps[:, np.random.random_integers(0, N-1, NSamples)] 
    dV1 = P2 - P1
    dV2 = P3 - P1
    dV1 = dV1/np.sqrt(np.sum(dV1**2, 0))[None, :]
    dV2 = dV2/np.sqrt(np.sum(dV2**2, 0))[None, :]
    dots = np.sum(dV1*dV2, 0)
    angles = np.arccos(dots)
    hist = np.histogram(angles, bins=NBins, range=(0, np.pi))[0]
    return hist

#Purpose: To create the Extended Gaussian Image by binning normals to
#sphere directions after rotating the point cloud to align with its principal axes
#Inputs: Ps (3 x N point cloud) (use to compute PCA), Ns (3 x N array of normals), 
#SPoints: A 3 x S array of points sampled evenly on the unit sphere used to 
#bin the normals
def getEGIHistogram(Ps, Ns, SPoints):
    S = SPoints.shape[1]
    hist = np.zeros(S)
    (eigs, V) = doPCA(Ps)
    NsTrans = (V.T).dot(Ns) #Project onto PCA Axes
    scores = (NsTrans.T).dot(SPoints)
    idx = np.argmax(scores, 1)
    for k in range(S):
        hist[k] = np.sum(idx == k)
    return hist

def getSpinImage(Ps, Ns, NAngles, Extent, Dim, showImage = True):
    #Create an image
    hist = np.zeros((Dim, Dim))
    (eigs, V) = doPCA(Ps)
    print("eigs = ", eigs)
    PA = ((V.T).dot(Ps))[1:, :]
    
    for a in np.linspace(0, 2*np.pi, NAngles+1)[0:NAngles]:
        [C, S] = [np.cos(a), np.sin(a)]
        R = np.array([[C, -S], [S, C]])
        PR = R.dot(PA)
        hist += np.histogram2d(PR[0, :], PR[1, :], Dim, [[-Extent, Extent], [-Extent, Extent]])[0]
        '''
        if (a == 0.0):
            plt.subplot(121)
            plt.plot(PR[0, :], PR[1, :], '.')
            plt.hold(True)
            plt.scatter([0], [0], 50, 'r')
            plt.subplot(122)
            plt.imshow(hist.T, interpolation = 'none', aspect = 'auto')
            plt.gca().invert_yaxis()
            plt.show()
        '''
    if showImage:
        plt.imshow(hist, interpolation = 'none', aspect = 'auto')
        plt.gca().invert_yaxis()
        plt.show()
    hist = hist/np.sum(hist) #Normalize before returning
    return hist.flatten()


def getSpinImage2(Ps, Ns, NAngles, Extent, Dim):
    #Create an image
    hist = np.zeros((Dim, Dim))
    (eigs, V) = doPCA(Ps)
    print("eigs = ", eigs)
    PA = (V.T).dot(Ps)
    
    for a in np.linspace(0, 2*np.pi, NAngles+1)[0:NAngles]:
        [C, S] = [np.cos(a), np.sin(a)]
        R = np.array([[1, 0, 0], [0, C, -S], [0, S, C]])
        PR = R.dot(PA)
        hist += np.histogram2d(PR[0, :], PR[1, :], Dim, [[-Extent, Extent], [-Extent, Extent]])[0]
#        if (a == 0.0):
#            plt.subplot(121)
#            plt.plot(PR[0, :], PR[1, :], '.')
#            plt.hold(True)
#            plt.scatter([0], [0], 50, 'r')
#            plt.subplot(122)
#            plt.imshow(hist.T, interpolation = 'none', aspect = 'auto')
#            plt.gca().invert_yaxis()
#            plt.show()
#    plt.subplot(121)
#    plt.imshow(hist, interpolation = 'none', aspect = 'auto')
#    plt.gca().invert_yaxis()
#    plt.subplot(122)
#    plt.imshow(np.flipud(hist), interpolation = 'none', aspect = 'auto')
#    plt.gca().invert_yaxis()
#    plt.show()
    hist = hist/np.sum(hist) #Normalize before returning
    return hist.flatten()

#Purpose: To create a histogram of spherical harmonic magnitudes in concentric
#spheres after rasterizing the point cloud to a voxel grid
#Inputs: Ps (3 x N point cloud), Ns (3 x N array of normals, not used here), 
#VoxelRes: The number of voxels along each axis (for instance, if 30, then rasterize
#to 30x30x30 voxels), Extent: The number of units along each axis (if 2, then 
#rasterize in the box [-1, 1] x [-1, 1] x [-1, 1]), NHarmonics: The number of spherical
#harmonics, NSpheres, the number of concentric spheres to take
def getSphericalHarmonicMagnitudes(Ps, Ns, VoxelRes, Extent, NHarmonics, NSpheres):
    hist = np.zeros((NSpheres, NHarmonics))
    #TODO: Finish this
    
    return hist.flatten()

#Purpose: Utility function for wrapping around the statistics functions.
#Inputs: PointClouds (a python list of N point clouds), Normals (a python
#list of the N corresponding normals), histFunction (a function
#handle for one of the above functions), *args (addditional arguments
#that the descriptor function needs)
#Returns: AllHists (A KxN matrix of all descriptors, where K is the length
#of each descriptor)
def makeAllHistograms(PointClouds, Normals, histFunction, *args):
    N = len(PointClouds)
    #Call on first mesh to figure out the dimensions of the histogram
    h0 = histFunction(PointClouds[0], Normals[0], *args)
    K = h0.size
    AllHists = np.zeros((K, N))
    AllHists[:, 0] = h0
    for i in range(1, N):
        print("Computing histogram %i of %i..."%(i+1, N))
        AllHists[:, i] = histFunction(PointClouds[i], Normals[i], *args)
    return AllHists

#########################################################
##              HISTOGRAM COMPARISONS                  ##
#########################################################

#Purpose: To compute the euclidean distance between a set
#of histograms
#Inputs: AllHists (K x N matrix of histograms, where K is the length
#of each histogram and N is the number of point clouds)
#Returns: D (An N x N matrix, where the ij entry is the Euclidean
#distance between the histogram for point cloud i and point cloud j)
def compareHistsEuclidean(AllHists):
    AllHists = AllHists/np.sum(AllHists, 0)[None, :]
    h = np.sum(AllHists**2, 0)
    D = h[:, None] + h[None, :] - 2*(AllHists.T).dot(AllHists)
    D[D < 0] = 0
    D = np.sqrt(D)
    return D

def compareHistsSpinEuclidean(AllHists, Dim):
    N = AllHists.shape[1]
    D = np.zeros((N, N))
    for i in range(N):
        I = np.reshape(AllHists[:, i], [Dim, Dim])
        for j in range(N):
            J = np.reshape(AllHists[:, j], [Dim, Dim])
            J2 = np.flipud(J)
            d1 = np.sqrt(np.sum((I-J)**2))
            d2 = np.sqrt(np.sum((I-J2)**2))
            print(d1, ", ", d2)
            D[i, j] = np.min(d1, d2)
    return D

#Purpose: To compute the cosine distance between a set
#of histograms
#Inputs: AllHists (K x N matrix of histograms, where K is the length
#of each histogram and N is the number of point clouds)
#Returns: D (An N x N matrix, where the ij entry is the cosine
#distance between the histogram for point cloud i and point cloud j)
def compareHistsCosine(AllHists):
    h = np.sqrt(np.sum(AllHists**2, 0))
    AllHists = AllHists/h[None, :]
    D = (AllHists.T).dot(AllHists)
    #D = np.arccos(D)
    return D

#Purpose: To compute the cosine distance between a set
#of histograms
#Inputs: AllHists (K x N matrix of histograms, where K is the length
#of each histogram and N is the number of point clouds)
#Returns: D (An N x N matrix, where the ij entry is the chi squared
#distance between the histogram for point cloud i and point cloud j)
def compareHistsChiSquared(AllHists):
    N = AllHists.shape[1]
    K = AllHists.shape[0]
    D = np.zeros((N, N))
    for k in range(K):
        h = AllHists[k, :].flatten()
        num = 0.5*(h[:, None]-h[None, :])**2
        denom = (h[:, None] + h[None, :])
        num[denom <= 0] = 0
        denom[denom <= 0] = 1
        D += num/denom
    return D

#Purpose: To compute the 1D Earth mover's distance between a set
#of histograms (note that this only makes sense for 1D histograms)
#Inputs: AllHists (K x N matrix of histograms, where K is the length
#of each histogram and N is the number of point clouds)
#Returns: D (An N x N matrix, where the ij entry is the earth mover's
#distance between the histogram for point cloud i and point cloud j)
def compareHistsEMD1D(AllHists):
    N = AllHists.shape[1]
    K = AllHists.shape[0]
    CS = np.cumsum(AllHists, 0)
    D = np.zeros((N, N))
    for k in range(K):
        c = CS[k, :]
        D += np.abs(c[:, None] - c[None, :])
    return D


#########################################################
##              CLASSIFICATION CONTEST                 ##
#########################################################

#Purpose: To implement your own custom distance matrix between all point
#clouds for the point cloud clasification contest
#Inputs: PointClouds, an array of point cloud matrices, Normals: an array
#of normal matrices
#Returns: D: A N x N matrix of distances between point clouds based
#on your metric, where Dij is the distnace between point cloud i and point cloud j
def getMyShapeDistances(PointClouds, Normals):
    #TODO: Finish this
    #This is just an example, but you should experiment to find which features
    #work the best, and possibly come up with a weighted combination of 
    #different features
    HistsD2 = makeAllHistograms(PointClouds, Normals, getD2Histogram, 3.0, 30, 100000)
    DEuc = compareHistsEuclidean(HistsD2)
    return DEuc

#########################################################
##                     EVALUATION                      ##
#########################################################

#Purpose: To return an average precision recall graph for a collection of
#shapes given the similarity scores of all pairs of histograms.
#Inputs: D (An N x N matrix, where the ij entry is the earth mover's distance
#between the histogram for point cloud i and point cloud j).  It is assumed
#that the point clouds are presented in contiguous chunks of classes, and that
#there are "NPerClass" point clouds per each class (for the dataset provided
#there are 10 per class so that's the default argument).  So the program should
#return a precision recall graph that has 9 elements
#Returns PR, an (NPerClass-1) length array of average precision values for all 
#recalls
def getPrecisionRecallMy(D, NPerClass = 10):
    PR = np.zeros(NPerClass-1)
    DI = np.argsort(D, 1)
    for i in range(DI.shape[0]):
        pr = np.zeros(NPerClass-1)
        recall = 0
        for j in range(1, DI.shape[1]): #Skip the first point (don't compare to itself)
            if DI[i, j]/NPerClass == i/NPerClass:
                pr[recall] = float(recall+1)/j
                recall += 1
        PR += pr
    return PR/float(DI.shape[0])


def getPrecisionRecall(D, NPerClass = 10):
    PR = np.zeros(NPerClass-1)
    N = D.shape[0]
    #TODO: Finish this, compute average precision recall graph
    #using all point clouds as queries
    for i in range(N-1):
        currRow = D[i, :] #look at the ith row
        ans = np.argsort(currRow) #sort the row
        numerator = 0
        denominator = 0
        
        for x in np.nditer(ans): #look at each element in the row
            if (i != x): #bc don't want to exclude shape itself from comparison
                denominator += 1 #keeps record of the position we are in
            if (i != x) and (np.floor(x/10) == np.floor(i/10)):
                numerator += 1 #adds to precision
                if (numerator - 1 == 9):
                    continue;
                PR[numerator-1] += float(numerator) / denominator
    PR = PR/float(N)
    return PR

#########################################################
##                     MAIN TESTS                      ##
#########################################################

if __name__ == '__main__':  
    NRandSamples = 10000 #You can tweak this number
    np.random.seed(100) #For repeatable results randomly sampling
    #Load in and sample all meshes
    PointClouds = []
    Normals = []
    for i in range(len(POINTCLOUD_CLASSES)):
        print("LOADING CLASS %i of %i..."%(i, len(POINTCLOUD_CLASSES)))
        PCClass = []
        for j in range(NUM_PER_CLASS):
            m = PolyMesh()
            filename = "models_off/%s%i.off"%(POINTCLOUD_CLASSES[i], j)
            print("Loading ", filename)
            m.loadOffFileExternal(filename)
            (Ps, Ns) = samplePointCloud(m, NRandSamples)
            PointClouds.append(Ps)
            Normals.append(Ns)
    SPoints = getSphereSamples(2)
    HistsSpin2 = makeAllHistograms(PointClouds, Normals, getSpinImage2, 100, 2, 40)
    HistsSpin = makeAllHistograms(PointClouds, Normals, getSpinImage, 100, 2, 40,False)
    HistsEGI = makeAllHistograms(PointClouds, Normals, getEGIHistogram, SPoints)
    HistsA3 = makeAllHistograms(PointClouds, Normals, getA3Histogram, 30, 100000)
    HistsD2 = makeAllHistograms(PointClouds, Normals, getD2Histogram, 3.0, 30, 100000)
    
    DSpin = compareHistsEuclidean(HistsSpin)
    DSpin2 = compareHistsEuclidean(HistsSpin2)
    DSpin2Flip = compareHistsSpinEuclidean(HistsSpin2, 40)
    DEGI = compareHistsEuclidean(HistsEGI)
    DA3 = compareHistsEuclidean(HistsA3)
    DD2 = compareHistsEuclidean(HistsD2)
    recalls = np.linspace(1.0/9.0, 1.0, 9)

    
    plt.subplot(321)
    plt.imshow(DSpin); plt.title("Spin")
    plt.subplot(322)
    plt.imshow(DSpin2); plt.title("Spin2")
    plt.subplot(323)
    plt.imshow(DSpin2Flip); plt.title("Spin2 Flip")
    plt.subplot(324)
    plt.imshow(DA3); plt.title("A3")
    plt.subplot(325)
    plt.imshow(DD2); plt.title("D2")
    plt.show()
    
    PRSpin = getPrecisionRecall(DSpin)
    PRSpin2 = getPrecisionRecall(DSpin2)
    PRSpin2Flip = getPrecisionRecall(DSpin2Flip)
    PREGI = getPrecisionRecall(DEGI)
    PRA3 = getPrecisionRecall(DA3)
    PRD2 = getPrecisionRecall(DD2)
    
    
    plt.plot(recalls, PREGI, 'c', label='EGI')
    plt.hold(True)
    plt.plot(recalls, PRA3, 'k', label='A3')
    plt.plot(recalls, PRD2, 'r', label='D2')
    plt.plot(recalls, PRSpin, 'b', label='Spin')
    plt.plot(recalls, PRSpin2, 'm', label='Spin2')
    plt.plot(recalls, PRSpin2Flip, 'y', label='Spin2Flip')
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.legend()
    plt.show()
