//
//  KeypointsDetector.hpp
//  Visual_Search
//
//  Created by Lirone Samoun on 29/09/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#ifndef KeypointsDetector_hpp
#define KeypointsDetector_hpp

#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>


#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/susan.h>
#include "typedefs.h"



class KeypointsDetector {
    
    
public:
    static const  int KEYPOINTS_NO_KEYPOINTS = 0;
    static const  int KEYPOINTS_HARRIS3D = 1;
    static const  int KEYPOINTS_SIFTNORMALGRADIENT = 2;
    static const  int UNIFORM_SAMPLING = 3;
    static const  int VOXEL_SAMPLING = 4;

    
    
    
public:
    //Constructors
    KeypointsDetector();
    KeypointsDetector(const int typeDetector);
    
    /**
     * @brief Detects key points in the input point cloud
     * @param input the input point cloud
     * @param keypoints the resulting key points. They are not necessarily a subset of the input cloud
     */
    pcl::PointCloud<PointType>::Ptr detectKeypoints (pcl::PointCloud<PointType>::Ptr& inputCloud, float radiusSearch = 0.01, float radius = 0.01);
    
    
    void setTypeDetector(const int typeDetector);
    
public:
    
    //SIFT detector
    pcl::PointCloud<PointType>::Ptr siftNormalGradient( pcl::PointCloud<PointType>::Ptr inputCloud, pcl::NormalEstimation<PointType, pcl::PointNormal> ne,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, float radiusSearch = 0.04f, float min_scale = 0.01f,  int n_octaves = 3, int min_contrast = 0.001f, int n_scales_per_octave = 4);
    //Harris detector
    pcl::PointCloud<PointType>::Ptr harrisDetector3D(pcl::PointCloud<PointType>::Ptr inputCloud, float radiusSearch = 0.04,float radius = 0.04, const int typeDetector = KEYPOINTS_HARRIS3D);
   
    //Sampling
    pcl::PointCloud<PointType>::Ptr  uniformSampling(pcl::PointCloud<PointType>::Ptr cloudInput, float radiusSearch = 0.01f);
    pcl::PointCloud<PointType>::Ptr  voxelGridSampling(pcl::PointCloud<PointType>::Ptr cloudInput,float voxelGridsize = 0.01f);
    
    
    
    
private:
    double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);
    
    
private:
    int m_typeDetector;
    pcl::PointCloud<PointType>::Ptr m_inputCloud;
    pcl::PointCloud<PointType>::Ptr m_keypointsCloud;
    
};


#endif /* KeypointsDetector_hpp */
