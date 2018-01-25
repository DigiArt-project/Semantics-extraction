#ifndef SPIN_ESTIMATION
#define SPIN_ESTIMATION

#include "descriptor_estimation.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

template<typename PointT, typename DescriptorT = pcl::Histogram<153>>
class SPINEstimation
{
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename pcl::PointCloud<DescriptorT>::Ptr DescriptorCloudPtrT;
    typedef typename DescriptorCloudT::Ptr x;
    typedef typename pcl::PointCloud<pcl::Normal>::Ptr NormalsT;
    
    // A handy typedef.
    typedef pcl::Histogram<153> SpinImage;
    
    
    void estimate();
    
    SPINEstimation():nr_(0.03f),imageWidth_(8), descriptor_(0), tree_(new pcl::search::KdTree<PointT>){}
    pcl::PointCloud<DescriptorT> getResultDescriptors();
    void setNormalRadiusSearch(float nr){
        nr_ = nr;
    }
    void setImageWidth(int iw){
        imageWidth_ = iw;
    }
    void setInputCloud(PointCloudT& cloud){
        cloud_ = cloud;
    }
    void setDescriptors(DescriptorCloudPtrT& desc){
        descriptor_ = desc;
    }
    void setNormals(NormalsT& normals){
        normals_ = normals;
    }
    void estimateNormals();
    void setIndexes(std::vector<int> idx){
        indexes_ = idx;
    }
    std::vector<int> getIndexex(){
        return indexes_;
    }

    
protected:
    typename pcl::SpinImageEstimation<PointT, pcl::Normal,SpinImage> spin_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_normals_;
    pcl::PointCloud<SpinImage>::Ptr descriptor_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
    int imageWidth_;
    PointCloudT cloud_;
    NormalsT normals_;
    std::vector<int> indexes_;
};


template<typename PointT, typename DescriptorT>
void SPINEstimation<PointT, DescriptorT>::estimateNormals()
{
    
    pcl::NormalEstimationOMP<PointT, pcl::Normal> nest;
    
    nest.setRadiusSearch(nr_);
    nest.setInputCloud(cloud_.makeShared());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    nest.compute(*normals);
    /*//remove NaNs
    std::vector<int> indexes;
    pcl::removeNaNNormalsFromPointCloud(*normals,*normals,indexes);
    setIndexes(indexes);
*/
    setNormals(normals);
}

template<typename PointT, typename DescriptorT>
pcl::PointCloud<DescriptorT> SPINEstimation<PointT, DescriptorT>::getResultDescriptors()
{
    return *descriptor_;
}

template<typename PointT, typename DescriptorT>
void SPINEstimation<PointT, DescriptorT>::estimate()
{
    pcl::console::TicToc tt;
    tt.tic ();
    estimateNormals();
   /* pcl::ExtractIndices<PointT> eifilter; // Initializing with true will allow us to extract the removed indices
    eifilter.setInputCloud (cloud_.makeShared());
    eifilter.filter(indexes_);*/

    spin_.setInputCloud(cloud_.makeShared());
    spin_.setInputNormals(normals_);
    // Radius of the support cylinder.
    spin_.setRadiusSearch(0.02);
    // Set the resolution of the spin image (the number of bins along one dimension).
    // Note: you must change the output histogram size to reflect this.
    spin_.setImageWidth(imageWidth_);
    // Object for storing the spin image for each point.
    pcl::PointCloud<SpinImage>::Ptr descriptor(new pcl::PointCloud<SpinImage>());
    spin_.compute(*descriptor);
    setDescriptors(descriptor);
    
}



#endif
