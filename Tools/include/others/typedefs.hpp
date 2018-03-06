//
//  typedefs.h
//  OBJECT_MATCHING_TEST
//
//  Created by Lirone Samoun on 01/04/2016.
//
//

#ifndef typedefs_h
#define typedefs_h


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;


/* Define some custom types to make the rest of our code easier to read */
typedef pcl::ReferenceFrame RFType;

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudT;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;
//Vector of point cloud
typedef std::vector<PointCloudPtr> cloudVectorT;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "LocalDescriptors" to be a pcl::PointCloud of FPFH points. Can change
//typedef pcl::SHOT352 LocalDescriptor;
typedef pcl::FPFHSignature33 LocalDescriptor;
typedef pcl::PointCloud<LocalDescriptor> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptor>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptor>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptor;
typedef pcl::PointCloud<GlobalDescriptor> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptor>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptor>::ConstPtr GlobalDescriptorsConstPtr;




#endif /* typedefs_h */
