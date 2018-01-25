#ifndef SC_ESTIMATION
#define SC_ESTIMATION

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>

template<typename PointT, typename DescriptorT = pcl::ShapeContext1980 >
class SC3DEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
    
    using DescriptorEstimation<PointT, DescriptorT>::clusters_;
    using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    
    
    SC3DEstimation():nr_(0.03f), tree_(new pcl::search::KdTree<PointT>){}
    void estimate();
    void setNormalRadiusSearch(float nr){nr_ = nr;}
    void estimateClustersNormals();
    void setIndexes(std::vector<int> idx){
        indexes_ = idx;
    }
    std::vector<int> getIndexex(){
        return indexes_;
    }
    
    
    typename pcl::ShapeContext3DEstimation<PointT, pcl::Normal, DescriptorT> sc3d_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_nms_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
    std::vector<int> indexes_;
};




template<typename PointT, typename DescriptorT>
void SC3DEstimation<PointT, DescriptorT>::estimateClustersNormals()
{
    clusters_nms_.clear();
    
    pcl::NormalEstimationOMP<PointT, pcl::Normal> nest;
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
        nest.setRadiusSearch(nr_);
        nest.setInputCloud(clusters_[i].makeShared());
        pcl::PointCloud<pcl::Normal> cluster_nm;
        nest.compute(cluster_nm);
        /*//remove NaNs
        std::vector<int> indexes;
        pcl::removeNaNNormalsFromPointCloud(cluster_nm,cluster_nm,indexes);
         setIndexes(indexes);*/
        clusters_nms_.push_back(cluster_nm);
    }
}


template<typename PointT, typename DescriptorT>
void SC3DEstimation<PointT, DescriptorT>::estimate()
{
    estimateClustersNormals();
    
    descriptors_.clear();
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
       /* pcl::ExtractIndices<PointT> eifilter; // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (clusters_[i].makeShared());
        eifilter.filter(indexes_);*/

        sc3d_.setInputCloud(clusters_[i].makeShared());
        sc3d_.setInputNormals(clusters_nms_[i].makeShared());
        sc3d_.setSearchMethod(tree_);
        
        // Search radius, to look for neighbors. It will also be the radius of the support sphere.
        sc3d_.setRadiusSearch(0.05);
        // The minimal radius value for the search sphere, to avoid being too sensitive
        // in bins close to the center of the sphere.
        sc3d_.setMinimalRadius(0.05 / 10.0);
        // Radius used to compute the local point density for the neighbors
        // (the density is the number of points within that radius).
        sc3d_.setPointDensityRadius(0.05 / 5.0);
        
        // Object for storing the USC descriptors for each point.
        pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptor(new pcl::PointCloud<pcl::ShapeContext1980>());
        
        sc3d_.compute(*descriptor);
        descriptors_.push_back(descriptor->at(0));


    }
}

    

#endif
