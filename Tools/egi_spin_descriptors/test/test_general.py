import sys
import os
here = os.path.dirname(os.path.realpath(__file__))
sys.path.append(here + "/../S3DGLPy")
sys.path.append(here + "/../")
from PolyMesh import *
import numpy as np
import matplotlib.pyplot as plt
import ShapeStatistics as shp
import pypcd




def main():
    resolution = 2
    cmap = plt.get_cmap('jet') # color ramp
    pc = pypcd.PointCloud.from_path(sys.argv[1])
    x = np.array(pc.pc_data['x'])
    y = np.array(pc.pc_data['y'])
    z = np.array(pc.pc_data['z'])

    normal_x = np.array(pc.pc_data['normal_x'])
    normal_y = np.array(pc.pc_data['normal_y'])
    normal_z = np.array(pc.pc_data['normal_z'])

    pts = [x, y, z]
    pts = np.asarray(pts)
    normals = [normal_x, normal_y, normal_z]
    normals = np.asarray(normals)


    N = pts.shape[1]
    centroid = shp.getCentroid(pts)
    pts = pts - centroid
    scale = 1 / (np.sqrt(np.sum(np.square(pts)) / N))
    pts = np.multiply(scale, pts)
    RMS = np.sqrt(np.sum(np.square(pts)) / N)


    # 360, 1.8, 180) #100, 2, 40

    #TESTING GET-SPIN-IMAGE
    NAngles = 360#720
    Extent = 2#2
    Dim = 35
    histogram = shp.getSpinImage(pts, normals, NAngles, Extent, Dim, True)

    sphere = getSphereMesh(1, resolution)
    #Get Extended Gaussian Image
    hist = shp.getEGIHistogram(pts, normals, sphere.VPos.T)
    hist = hist / np.max(hist)
    sphere.VColors = np.array(np.round(255.0*cmap(hist)[:, 0:3]), dtype=np.int64)
    #sphere.saveOffFile(sys.argv[2], output255 = True)
    #print sphere
    
    #print pts
    #print normals
    '''
    m = PolyMesh()
    m.loadOffFileExternal(sys.argv[1]) #Load a mesh
    (Ps, Ns) = samplePointCloud(m, 20000) #Sample 20,000 points and associated normals
    #print Ps.shape
    #exportPointCloud(Ps, Ns, sys.argv[2]) #Export point cloud
    '''
    


if __name__ == '__main__':
    main()

