#ifndef OURCVFH_ESTIMATION
#define OURCVFH_ESTIMATION

#include "descriptor_estimation.h"
#include <pcl/features/our_cvfh.h>
#include <pcl/features/normal_3d_omp.h>

template<typename PointT, typename DescriptorT = pcl::VFHSignature308>
class OURCVFHEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
	//Ref https://ece.uwaterloo.ca/~dwharder/aads/Tutorial/2z/
	using DescriptorEstimation<PointT, DescriptorT>::clusters_;
	using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
	
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
	typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
	OURCVFHEstimation():nr_(0.01f), tree_(new pcl::search::KdTree<PointT>){}
	void estimate();
	void setNormalRadiusSearch(float nr){nr_ = nr;}
	void estimateClustersNormals();
	
protected:
	typename pcl::OURCVFHEstimation<PointT, pcl::Normal, DescriptorT> ourcvfh_;
	pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_nms_;
	typename pcl::search::KdTree<PointT>::Ptr tree_;
	float nr_;
};

template<typename PointT, typename DescriptorT>
void OURCVFHEstimation<PointT, DescriptorT>::estimateClustersNormals()
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
void OURCVFHEstimation<PointT, DescriptorT>::estimate()
{
	estimateClustersNormals();
	
	descriptors_.clear();
	
	for(unsigned int i = 0; i < clusters_.size(); ++i)
	{
		ourcvfh_.setInputCloud(clusters_[i].makeShared());
		ourcvfh_.setInputNormals(clusters_nms_[i].makeShared());
		ourcvfh_.setSearchMethod(tree_);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor (new pcl::PointCloud<pcl::VFHSignature308> ());
		ourcvfh_.compute(*descriptor);
		
		descriptors_.push_back(descriptor->at(0));
	}
}

#endif