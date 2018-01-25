//
//  OctomapHandler.hpp
//  RECONNAISSANCE
//
//  Created by Lirone Samoun on 27/04/2016.
//
//

#ifndef VOLUMEESTIMATION_hpp
#define VOLUMEESTIMATION_hpp

#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OccupancyOcTreeBase.h>

#include <octomap/OcTreeBaseImpl.h>


using namespace octomap;


template<typename PointT>
class VolumeEstimation {
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    
    
public:
    /** \brief cconstructor
     */
    VolumeEstimation(){
        m_xLength = 0;
        m_yLength = 0;
        m_zLength = 0;
        m_volumeCube = 0;
        m_volumeVoxelObject = 0;
        m_numberLeaf = 0;
        m_tree = 0;
        m_relativeEntropy = 0;
    }
    
    /** \brief create an octree from a pointCloud data
     * \param resolution in meters
     */
    void createOctoMap(PointCloudT & inputCloud, float resolution);
    
    /** \brief Save tree to the computer
     * \param path to save the data (need to be a .bt file)
     */
    void saveTree(std::string pathToSave);
    
    /** \brief check is the octree is empty or not
     */
    bool treeIsEmpty();
    
    /** \brief get number of leafs of the tree
     */
    int getNumberLeaf();
    
    float getVolumeBox(){
        return m_volumeCube;
    }
    float getVolumeObject(){
        return m_volumeVoxelObject;
    }
    
    octomap::OcTree getTree();
    
    
    /** \brief delete all the data
     */
    void clearData();
    
    void computeRelativeEntropy(OcTree & tree1, OcTree & tree2);
    
    ~VolumeEstimation();
    
protected:
    void computeVolume();
    
    
protected:
    double m_xLength;
    double m_yLength;
    double m_zLength;
    //Volume of the cube surrounding the object
    double m_volumeCube;
    // Volume of the object
    double m_volumeVoxelObject;
    
    double m_relativeEntropy;
    
    //Number of leaf of the tree
    int m_numberLeaf;
    
    //The tree
    octomap::OcTree* m_tree;
    
    
};

template<typename PointT>
void VolumeEstimation<PointT>::computeVolume(){
    
    //gives  the dimensions of the minimum bounding boxaround all known voxels (occupied and free).
    //determine the overall volume, which corresponds to the volume you wouldneed to cover with a 3D array or voxel grid (including free and unknown space).
    m_tree->getMetricSize(m_xLength, m_yLength, m_zLength);
    
    
    std::cout << "x, y, z in meter: " << m_xLength << " , " << m_yLength << " , " << m_zLength << std::endl;
    // std::cout << "Volume of the entire box : " << m_xLength * m_yLength * m_zLength << std::endl;
    
    m_volumeCube =  m_xLength * m_yLength * m_zLength;
    
    //count the occupied volume of the individual occupied voxels. If there's a hollow space within an object, however, you won't use it in the volume computation
    double volumeTotal = 0;
    for(octomap::OcTree::leaf_iterator it = m_tree->begin_leafs(),end=m_tree->end_leafs(); it!= end; ++it)
    {
        // check whether the node is occupied
        if (m_tree->isNodeOccupied(*it)){
            //get the side length of the current voxel
            double size = it.getSize();
            //Compute volume voxel using formula of volume cube
            double volumeCurrentVoxel = size * size * size;
            volumeTotal = volumeTotal + volumeCurrentVoxel;
            
        }
    }
    //std::cout << " \n Volume Total Object: " << volumeTotal << std::endl;
    m_volumeVoxelObject = volumeTotal;
}

template<typename PointT>
void VolumeEstimation<PointT>::createOctoMap(PointCloudT & inputCloud, float resolution){
    //Create the tree
    m_tree = new octomap::OcTree(resolution);
    
    //PointCloudPtrT cloudObject (new  PointCloudT());
    
    
    for(typename pcl::PointCloud<PointT>::iterator it = inputCloud.begin(); it!= inputCloud.end(); ++it )
    {
        pcl::PointXYZ pt(it->x,it->y,it->z);
        point3d point (pt.x ,pt.y,pt.z);
        m_tree->updateNode(point, true); // integrate 'occupied' measurement
        
    }
    computeVolume();
    
}
/*
template<typename PointT>
void VolumeEstimation<PointT>::computeRelativeEntropy(OcTree & tree1, OcTree & tree2){
    double kld_sum = 0.0;
    for (OcTree::leaf_iterator it = tree1.begin_leafs(),
         end = tree1.end_leafs();  it != end; ++it)
    {
        OcTreeNode* n = tree2.search(it.getKey());
        if (!n){
            OCTOMAP_ERROR("Could not find coordinate of 1st octree in 2nd octree\n");
        } else{
            // check occupancy prob:
            double p1 = it->getOccupancy();
            double p2 = n->getOccupancy();
            
            //      if (p1 > 0.1 || p2 > 0.1)
            if (p1 > 0.001 && p2 < 0.001)
                OCTOMAP_WARNING("p2 near 0, p1 > 0 => inf?");
            if (p1 < 0.999 && p2 > 0.999)
                OCTOMAP_WARNING("p2 near 1, p1 < 1 => inf?");
            
            double kld = 0;
            if (p1 < 0.0001)
                kld =log((1-p1)/(1-p2))*(1-p1);
            else if (p1 > 0.9999)
                kld =log(p1/p2)*p1;
            else
                kld +=log(p1/p2)*p1 + log((1-p1)/(1-p2))*(1-p1);
            
            if (isnan(kld)){
                OCTOMAP_ERROR("KLD is nan! KLD(%f,%f)=%f; sum = %f", p1, p2, kld, kld_sum);
                exit(-1);
            }
            
            kld_sum+=kld;
            
            //if (p1 <)
            //      if (fabs(p1-p2) > 1e-6)
            //        cout << "diff: " << p1-p2 << endl;
        }
        
    }
    std::cout << "Relative entropy: " << kld_sum << std::endl;
}*/

template<typename PointT>
void VolumeEstimation<PointT>::saveTree(std::string pathToSave){
    
    if (treeIsEmpty()){
        std::cerr << "Nothing to save - The tree is empty " << std::endl;
    }else {
        m_tree->write(pathToSave);
        std::cout << "Tree has being save" << std::endl;
    }
}

template<typename PointT>
bool VolumeEstimation<PointT>::treeIsEmpty(){
    if (m_tree->size() == 0){
        return true;
    }else {
        return false;
    }
}

template<typename PointT>
int VolumeEstimation<PointT>::getNumberLeaf(){
    return this->m_numberLeaf;
}

template<typename PointT>
octomap::OcTree VolumeEstimation<PointT>::getTree(){
    return *this->m_tree;
}

template<typename PointT>
VolumeEstimation<PointT>::~VolumeEstimation(){
    m_tree = 0;
}

template<typename PointT>
void VolumeEstimation<PointT>::clearData(){
    m_xLength = 0;
    m_yLength = 0;
    m_zLength = 0;
    m_volumeCube = 0;
    m_volumeVoxelObject = 0;
    m_numberLeaf = 0;
    m_tree = 0;
    m_relativeEntropy = 0;
}


#endif /* VOLUMESTIMATION_hpp */
