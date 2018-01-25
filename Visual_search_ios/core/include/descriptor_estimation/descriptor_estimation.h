#ifndef DESCRIPTOR_ESTIMATION
#define DESCRIPTOR_ESTIMATION

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

template<typename PointT, typename DescriptorT>
class DescriptorEstimation
{
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename PointCloudT::CloudVectorType CloudVectorT;
	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
	
	DescriptorEstimation(){}
	void setInputClusters(CloudVectorT& clusters);
	void setInputCluster(PointCloudT& cluster);
	virtual void estimate() = 0;
	void getResultDescriptors(DescriptorCloudT& descriptors);
	
protected:
	CloudVectorT clusters_;
	DescriptorCloudT descriptors_;
};

template<typename PointT, typename DescriptorT>
void DescriptorEstimation<PointT, DescriptorT>::setInputClusters(CloudVectorT& clusters)
{
	clusters_.clear();
	clusters_ = clusters;
}

template<typename PointT, typename DescriptorT>
void DescriptorEstimation<PointT, DescriptorT>::setInputCluster(PointCloudT& cluster)
{
	clusters_.clear();
	clusters_.push_back(cluster);
}

template<typename PointT, typename DescriptorT>
void DescriptorEstimation<PointT, DescriptorT>::getResultDescriptors(DescriptorCloudT& descriptors)
{
	pcl::copyPointCloud<DescriptorT>(descriptors_, descriptors);
}

#endif