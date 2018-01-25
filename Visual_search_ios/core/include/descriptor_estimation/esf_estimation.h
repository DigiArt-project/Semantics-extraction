#ifndef ESF_ESTIMATION
#define ESF_ESTIMATION

#include "descriptor_estimation.h"
#include <pcl/features/esf.h>

template<typename PointT, typename DescriptorT = pcl::ESFSignature640>
class ESFEstimation : public DescriptorEstimation<PointT, DescriptorT>
{
	//Ref https://ece.uwaterloo.ca/~dwharder/aads/Tutorial/2z/
	using DescriptorEstimation<PointT, DescriptorT>::clusters_;
	using DescriptorEstimation<PointT, DescriptorT>::descriptors_;
	
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
	typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
	void estimate();
	
protected:
	typename pcl::ESFEstimation<PointT, DescriptorT> esf_;
};

template<typename PointT, typename DescriptorT>
void ESFEstimation<PointT, DescriptorT>::estimate()
{
	descriptors_.clear();
	for(unsigned int i = 0; i < clusters_.size(); ++i)
	{
		esf_.setInputCloud(clusters_[i].makeShared());
		pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor (new pcl::PointCloud<pcl::ESFSignature640> ());
		esf_.compute (*descriptor);
		
		descriptors_.push_back(descriptor->at(0));
	}
}

#endif