#ifndef CRH_ESTIMATION
#define CRH_ESTIMATION

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/features/crh.h>


template<typename PointT, typename DescriptorT = pcl::Histogram<90>>
class CRHEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
    //Ref https://ece.uwaterloo.ca/~dwharder/aads/Tutorial/2z/
    using DescriptorEstimation<PointT, DescriptorT>::clusters_;
    using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    CRHEstimation():nr_(0.03f), tree_(new pcl::search::KdTree<PointT>){}
    void estimate();
    void setNormalRadiusSearch(float nr){nr_ = nr;}
    void estimateClustersNormals();
    
    
protected:
    typename pcl::CRHEstimation<PointT,pcl::Normal, DescriptorT> crh_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_nms_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
};

template<typename PointT, typename DescriptorT>
void CRHEstimation<PointT, DescriptorT>::estimateClustersNormals()
{
    clusters_nms_.clear();
    
    pcl::NormalEstimationOMP<PointT, pcl::Normal> nest;
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
        nest.setRadiusSearch(nr_);
        nest.setInputCloud(clusters_[i].makeShared());
        pcl::PointCloud<pcl::Normal> cluster_nm;
        nest.compute(cluster_nm);
        clusters_nms_.push_back(cluster_nm);
    }
}

template<typename PointT, typename DescriptorT>
void CRHEstimation<PointT, DescriptorT>::estimate()
{
    estimateClustersNormals();
    
    descriptors_.clear();
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
        crh_.setInputCloud(clusters_[i].makeShared());
        crh_.setInputNormals(clusters_nms_[i].makeShared());
        crh_.setSearchMethod(tree_);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(clusters_[i], centroid);
        crh_.setCentroid(centroid);
        typename pcl::PointCloud<DescriptorT>::Ptr descriptor(new pcl::PointCloud<DescriptorT>);
        crh_.compute(*descriptor);
        
        descriptors_.push_back(descriptor->at(0));
    }
    
}


#endif
