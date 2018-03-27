#ifndef GSHOT_ESTIMATION
#define GSHOT_ESTIMATION

#include "descriptor_estimation.h"
#include <pcl/features/gshot.h>
#include <pcl/features/normal_3d_omp.h>

template<typename PointT, typename DescriptorT = pcl::SHOT352>
class GSHOTEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
	//Ref https://ece.uwaterloo.ca/~dwharder/aads/Tutorial/2z/
	using DescriptorEstimation<PointT, DescriptorT>::clusters_;
	using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
	
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
	typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    GSHOTEstimation():nr_(0.03f), tree_(new pcl::search::KdTree<PointT>),ksearch_(10){}

	void estimate();
    void setNormalRadiusSearch(float nr){nr_ = nr;}
    void setRadiusKSearch(float ks){ksearch_ = ks;}

    void estimateClustersNormals();
    void setIndexes(std::vector<int> idx){
        indexes_ = idx;
    }
    std::vector<int> getIndexex(){
        return indexes_;
    }


	
protected:
	typename pcl::GSHOTEstimation<PointT, pcl::Normal, DescriptorT> gshot_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_nms_;
    typename pcl::search::KdTree<PointT>::Ptr tree_;
    float nr_;
    float ksearch_;
    std::vector<int> indexes_;
};

template<typename PointT, typename DescriptorT>
void GSHOTEstimation<PointT, DescriptorT>::estimateClustersNormals()
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
        setIndexes(indexes);
*/
        clusters_nms_.push_back(cluster_nm);
    }
}


template<typename PointT, typename DescriptorT>
void GSHOTEstimation<PointT, DescriptorT>::estimate()
{
    
    estimateClustersNormals();
    
	descriptors_.clear();
	for(unsigned int i = 0; i < clusters_.size(); ++i)
	{
        /*pcl::ExtractIndices<PointT> eifilter; // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (clusters_[i].makeShared());
        eifilter.filter(indexes_);*/

		gshot_.setInputCloud(clusters_[i].makeShared());
        gshot_.setInputNormals(clusters_nms_[i].makeShared());
        gshot_.setKSearch(0);
        gshot_.setRadiusSearch(0);
        gshot_.setSearchMethod(tree_);
        
		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor (new pcl::PointCloud<pcl::SHOT352> ());
		gshot_.compute (*descriptor);
		
		descriptors_.push_back(descriptor->at(0));
	}

}

#endif
