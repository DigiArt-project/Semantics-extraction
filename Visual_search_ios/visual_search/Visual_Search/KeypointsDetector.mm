//
//  KeypointsDetector.cpp
//  Visual_Search
//
//  Created by Lirone Samoun on 29/09/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#include "KeypointsDetector.h"


#pragma mark -
#pragma mark constructors
KeypointsDetector::KeypointsDetector(){
    
}

KeypointsDetector::KeypointsDetector(const int typeDetector){
    if (typeDetector == KeypointsDetector::KEYPOINTS_HARRIS3D){
        this->m_typeDetector = KeypointsDetector::KEYPOINTS_HARRIS3D;
        
    }else if (typeDetector == KeypointsDetector::KEYPOINTS_SIFTNORMALGRADIENT){
        this->m_typeDetector =  KeypointsDetector::KEYPOINTS_SIFTNORMALGRADIENT;
        
    }else if (typeDetector == KeypointsDetector::UNIFORM_SAMPLING){
        this->m_typeDetector =  KeypointsDetector::UNIFORM_SAMPLING;
        
    }
    else if (typeDetector == KeypointsDetector::VOXEL_SAMPLING){
        this->m_typeDetector =  KeypointsDetector::VOXEL_SAMPLING;
        
    }else {
        this->m_typeDetector =  KeypointsDetector::KEYPOINTS_NO_KEYPOINTS;
        
    }
}

#pragma mark -
#pragma mark keypoints detection
pcl::PointCloud<PointType>::Ptr KeypointsDetector::detectKeypoints (pcl::PointCloud<PointType>::Ptr& inputCloud, float radiusSearch, float radius)
{
    

    
    pcl::PointCloud<PointType>::Ptr cloud = inputCloud;
    pcl::PointCloud<PointType>::Ptr cloudObjectKeypoints;
    //std::cout << "#### keypoint detection..." << std::flush;
    if (this->m_typeDetector ==  KeypointsDetector::KEYPOINTS_HARRIS3D){
        cloudObjectKeypoints = harrisDetector3D(cloud,radiusSearch, radius, KeypointsDetector::KEYPOINTS_HARRIS3D);
        
    }else if (this->m_typeDetector == KeypointsDetector::KEYPOINTS_SIFTNORMALGRADIENT){
        pcl::NormalEstimation<PointType, pcl::PointNormal> ne;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
        cloudObjectKeypoints = siftNormalGradient(cloud, ne, cloud_normals);
        
    }else if (this->m_typeDetector == KeypointsDetector::UNIFORM_SAMPLING){
        cloudObjectKeypoints = uniformSampling(cloud,0.03f);
        
    }else if (this->m_typeDetector == KeypointsDetector::VOXEL_SAMPLING){
        cloudObjectKeypoints = voxelGridSampling(cloud, 0.01f);
        
    }
    else {
        std::cerr << "ERROR DETECTION KEYPOINTS " << std::endl;
        exit(1);
    }
    
    this->m_keypointsCloud = cloudObjectKeypoints;
    cout << "Number of keypoints found: " <<  this->m_keypointsCloud->points.size() << endl;
    if (m_keypointsCloud->empty()){
        std::cerr << "Not keypoints found for this object " << std::endl;
        // exit(1);
    }
    
    return this->m_keypointsCloud;
    
}



#pragma mark -
#pragma mark sift
pcl::PointCloud<PointType>::Ptr KeypointsDetector::siftNormalGradient( pcl::PointCloud<PointType>::Ptr inputCloud, pcl::NormalEstimation<PointType, pcl::PointNormal> ne,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,float radiusSearch, float min_scale, int n_octaves,int min_contrast, int n_scales_per_octave){
    
    std::cout << "#### SIFT 3D NORMAL KEYPOINT DETECTION" << std::endl;
    /* This example shows how to estimate the SIFT points based on the
     * Normal gradients i.e. curvature than using the Intensity gradient
     * as usually used for SIFT keypoint estimation.
     */
    pcl::search::KdTree<PointType>::Ptr tree_n(new pcl::search::KdTree<PointType>());
    // Parameters for sift computation
    
    ne.setInputCloud(inputCloud);
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(radiusSearch);
    ne.compute(*cloud_normals);
    
    // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i<cloud_normals->points.size(); ++i)
    {
        cloud_normals->points[i].x = inputCloud->points[i].x;
        cloud_normals->points[i].y = inputCloud->points[i].y;
        cloud_normals->points[i].z = inputCloud->points[i].z;
    }
    
    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(result);
    
    std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;
    
    
    // Copying the pointwithscale to pointxyz so as visualize the cloud
    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<PointType>::Ptr keyPointsSIFTXYZ (new pcl::PointCloud<PointType>);
    copyPointCloud(result, *keyPointsSIFTXYZ);
    
    
    // std::cout << "SIFT points in the cloud_temp are " << keyPointsSIFTXYZ->points.size () << std::endl;
    
    return keyPointsSIFTXYZ;
}


#pragma mark -
#pragma mark HARRIS 3D
pcl::PointCloud<PointType>::Ptr KeypointsDetector::harrisDetector3D(pcl::PointCloud<PointType>::Ptr inputCloud, float radiusSearch, float radius, const int typeDetector ){
    
    std::cout << "#### Harris 3D KEYPOINT DETECTION" << std::endl;
    std::cout << "--> " << typeDetector << std::endl;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsHarris(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius(radius);
    //detector.setRadiusSearch(radiusSearch);
    detector.setInputCloud(inputCloud);
    
    
    
    detector.setMethod(pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS);
    
    
    
    detector.compute(*keypointsHarris);
    
    
    
    //std::cout << "No of Harris 3D keypoints detected: " << keypointsHarris->size() << std::endl;
    
    pcl::PointXYZ tmp;
    double max = 0,min=0;
    
    for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypointsHarris->begin(); i!= keypointsHarris->end();){
        //  std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
        if ((*i).intensity>max ){
            //std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
            max = (*i).intensity;
        }
        if ((*i).intensity<min){
            min = (*i).intensity;
        }
        
        //Delete bad points
        if ( (*i).x < -2 ||  (*i).y <  -2||  (*i).z <  -2 ){
            i = keypointsHarris->erase(i);
            
        }else {
            ++i;
        }
    }
    
    
    
    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<PointType>::Ptr keyPointsHarrisXYZ (new pcl::PointCloud<PointType>);
    copyPointCloud(*keypointsHarris, *keyPointsHarrisXYZ);
    
    
    return keyPointsHarrisXYZ;
}

#pragma mark -
#pragma mark Sampling
pcl::PointCloud<PointType>::Ptr  KeypointsDetector::voxelGridSampling(pcl::PointCloud<PointType>::Ptr cloudInput,float voxelGridsize){
    
    pcl::PointCloud<PointType>::Ptr cloudOutput(new pcl::PointCloud<PointType>);
    // Filter object.
    pcl::VoxelGrid<PointType> filter;
    filter.setInputCloud(cloudInput);
    // We set the size of every voxel to be 1x1x1cm
    // (only one point per every cubic centimeter will survive).
    filter.setLeafSize(voxelGridsize, voxelGridsize, voxelGridsize);
    
    filter.filter(*cloudOutput);
    
    return cloudOutput;
}

pcl::PointCloud<PointType>::Ptr  KeypointsDetector::uniformSampling(pcl::PointCloud<PointType>::Ptr cloudInput, float radiusSearch){
    std::cout << "######  DOWNSAMPLING THE CLOUD " << std::endl;
    pcl::PointCloud<PointType>::Ptr cloudOutput(new pcl::PointCloud<PointType>);
    // Uniform sampling object.
    pcl::UniformSampling<PointType> filter;
    filter.setInputCloud(cloudInput);
    // We set the size of every voxel to be 1x1x1cm
    // (only one point per every cubic centimeter will survive).
    filter.setRadiusSearch(radiusSearch);
    // We need an additional object to store the indices of surviving points.
    pcl::PointCloud<int> keypointIndicesInput;
    filter.filter(*cloudOutput);
    
    return cloudOutput;
    
}


#pragma mark -
#pragma mark getter setter
void KeypointsDetector::setTypeDetector(const int typeDetector){
    this->m_typeDetector = typeDetector;
}
