#ifndef GRSD_ESTIMATION
#define GRSD_ESTIMATION

#include "descriptor_estimation.hpp"
#include <pcl/features/grsd.h>

template<typename PointT, typename DescriptorT = pcl::GRSDSignature21>
class GRSDEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
	//Ref https://ece.uwaterloo.ca/~dwharder/aads/Tutorial/2z/
	using DescriptorEstimation<PointT, DescriptorT>::clusters_;
	using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
	
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
	typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    GRSDEstimation():nr_(0.03f),radiusGRSD_(0.05f), tree_(new pcl::search::KdTree<PointT>){}
    void estimate();
    void setNormalRadiusSearch(float nr){nr_ = nr;}
    void estimateClustersNormals();
    void setIndexes(std::vector<int> idx){
        indexes_ = idx;
    }
    std::vector<int> getIndexex(){
        return indexes_;
    }

	
protected:
	typename pcl::GRSDEstimation<PointT,pcl::Normal, DescriptorT> grsd_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_nms_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
    float radiusGRSD_;
    std::vector<int> indexes_;
};

template<typename PointT, typename DescriptorT>
void GRSDEstimation<PointT, DescriptorT>::estimateClustersNormals()
{
    clusters_nms_.clear();
    
    pcl::NormalEstimationOMP<PointT, pcl::Normal> nest;
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
        
        nest.setRadiusSearch(nr_);
        nest.setInputCloud(clusters_[i].makeShared());
        pcl::PointCloud<pcl::Normal> cluster_nm;
        nest.compute(cluster_nm);
        /*
        //remove NaNs
        std::vector<int> indexes;
        pcl::removeNaNNormalsFromPointCloud(cluster_nm,cluster_nm,indexes);
        setIndexes(indexes);*/
        clusters_nms_.push_back(cluster_nm);
    }
}

template<typename PointT, typename DescriptorT>
void GRSDEstimation<PointT, DescriptorT>::estimate()
{
    estimateClustersNormals();
    
    descriptors_.clear();
    for(unsigned int i = 0; i < clusters_.size(); ++i)
    {
        /*
        pcl::ExtractIndices<PointT> eifilter; // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (clusters_[i].makeShared());
        eifilter.filter(indexes_);
         */

        grsd_.setInputCloud(clusters_[i].makeShared());
        grsd_.setInputNormals(clusters_nms_[i].makeShared());
        grsd_.setSearchMethod(tree_);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        grsd_.setRadiusSearch(radiusGRSD_);
        typename pcl::PointCloud<DescriptorT>::Ptr descriptor(new pcl::PointCloud<DescriptorT>);
        grsd_.compute(*descriptor);
        
        descriptors_.push_back(descriptor->at(0));
    }

}

#endif
