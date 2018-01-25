#ifndef USC_ESTIMATION
#define USC_ESTIMATION

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/usc.h>

template<typename PointT, typename DescriptorT = pcl::UniqueShapeContext1960 >
class USCEstimation
{
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    
    // A handy typedef.
    pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
    
    
    void estimate();
    
    USCEstimation():nr_(0.03f), tree_(new pcl::search::KdTree<PointT>){}
    pcl::PointCloud<pcl::UniqueShapeContext1960> getResultDescriptors();
    void setUSCRadiusSearch(float nr){
        nr_ = nr;
    }
    void setInputCloud(PointCloudT& cloud){
        cloud_ = cloud;
    }
    
    void setDescriptors(DescriptorCloudT& desc){
        descriptors_ = desc;
    }
    
    
    
protected:
    // USC estimation object.
    typename pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc_;
    pcl::PointCloud<pcl::UniqueShapeContext1960> descriptors_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
    PointCloudT cloud_;
};



template<typename PointT, typename DescriptorT>
pcl::PointCloud<pcl::UniqueShapeContext1960> USCEstimation<PointT, DescriptorT>::getResultDescriptors()
{
    return descriptors_;
}

template<typename PointT, typename DescriptorT>
void USCEstimation<PointT, DescriptorT>::estimate()
{
    
    
    usc_.setInputCloud(cloud_.makeShared());
    // Search radius, to look for neighbors. It will also be the radius of the support sphere.
    usc_.setRadiusSearch(nr_);
    // The minimal radius value for the search sphere, to avoid being too sensitive
    // in bins close to the center of the sphere.
    usc_.setMinimalRadius(nr_ / 10.0);
    // Radius used to compute the local point density for the neighbors
    // (the density is the number of points within that radius).
    usc_.setPointDensityRadius(nr_ / 5.0);
    // Set the radius to compute the Local Reference Frame.
    usc_.setLocalRadius(nr_);
    
    // Object for storing the USC descriptors for each point.
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descriptors(new pcl::PointCloud<pcl::UniqueShapeContext1960>());
    
    usc_.compute(*descriptors);
    setDescriptors(*descriptors);
    
    
}

#endif
