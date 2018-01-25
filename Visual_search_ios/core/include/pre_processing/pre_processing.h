#ifndef PRE_PROCESSING
#define PRE_PROCESSING

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/filesystem.hpp>
#include <fstream>

template<typename PointT>
class PreProcessing
{
public:
	typedef typename pcl::PointCloud<PointT> PointCloudT;
	typedef typename PointCloudT::Ptr PointCloudPtrT;
	typedef typename PointCloudT::CloudVectorType CloudVectorT;
	
	PreProcessing();
	~PreProcessing(){}
	
	//Main
	void setInputScene(PointCloudPtrT cloud_in);
	bool setROIBoundary(std::string file);
	void process();
	void getCurrentResult(PointCloudPtrT& cloud_out);
	void getResultClusters(CloudVectorT& clusters){clusters = clusters_;}
	
	//Advanced
	void roi();
	
	void down_sampling();
	void setLeafSize(float leaf){leaf_ = leaf;}
	
	void outlier_removal();
	void setMeanK(int meanK){meanK_ = meanK;}
	void setStddevMulThresh(float stddev_mult){stddev_mult_ = stddev_mult;}
	
	void plane_segmentation();
	void setDistanceThreshold(float distance_threshold){distance_threshold_ = distance_threshold;}
	void getPlaneCoeff(pcl::ModelCoefficients::Ptr& plane_coeff);
	
	void cluster_extraction();
	void setClusterSize(int min_pts, int max_pts){min_pts_ = min_pts; max_pts_ = max_pts;}
	void setClusterTolerance(float cluster_tolerance){cluster_tolerance_ = cluster_tolerance;}
	
protected:
	PointCloudPtrT cloud_;
	
	pcl::PassThrough<PointT> roi_;
	float x_;
	float X_;
	float y_;
	float Y_;
	float z_;
	float Z_;
	
	pcl::VoxelGrid<PointT> down_sampling_;
	float leaf_;
	
	pcl::StatisticalOutlierRemoval<PointT> outlier_removal_;
	int meanK_;
	float stddev_mult_;
	
	pcl::SACSegmentation<PointT> plane_segmentation_;
	float distance_threshold_;
	pcl::ModelCoefficients::Ptr plane_coeff_;
	
	pcl::EuclideanClusterExtraction<PointT> cluster_extraction_;
	CloudVectorT clusters_;
	int min_pts_;
	int max_pts_;
	float cluster_tolerance_;
	
	pcl::ExtractIndices<PointT> extract_indices_;
};

//Constructor
template<typename PointT>
PreProcessing<PointT>::PreProcessing():cloud_(new PointCloudT), plane_coeff_(new pcl::ModelCoefficients)
{
	//Down sampling
	leaf_ = 0.005;
	
	//Outlier removal
	meanK_ = 50;
	stddev_mult_ = 1.0;
	
	//Plane segmentation
	distance_threshold_ = 1.5*leaf_;
	
	//Cluster extraction
	min_pts_ = 300;
	max_pts_ = 1500;
	cluster_tolerance_ = 0.02;
}

//Main
template<typename PointT>
void PreProcessing<PointT>::process()
{
	clusters_.clear();
	
	if(cloud_->size() == 0)
		return;
	
	roi();
	if(cloud_->size() == 0)
		return;
	
	down_sampling();
	if(cloud_->size() == 0)
		return;
	
	plane_segmentation();
	if(cloud_->size() == 0)
		return;
	
	/*
	outlier_removal();
	if(cloud_->size() == 0)
		return;
	*/
	
	cluster_extraction();
}

template<typename PointT>
void PreProcessing<PointT>::setInputScene(PointCloudPtrT cloud_in)
{
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud<PointT>(*cloud_in, *cloud_, indices);
}

template<typename PointT>
void PreProcessing<PointT>::getCurrentResult(PointCloudPtrT& cloud_out)
{
	pcl::copyPointCloud<PointT>(*cloud_, *cloud_out);
}

//ROI segmentation
template<typename PointT>
void PreProcessing<PointT>::roi()
{
	roi_.setInputCloud(cloud_);
	roi_.setFilterFieldName("z");
	roi_.setFilterLimits(z_, Z_);
	roi_.filter(*cloud_);
	//只留下PointCloud中y值介於介於y~Y之間的點
	roi_.setInputCloud(cloud_);
	roi_.setFilterFieldName("y");
	roi_.setFilterLimits(y_, Y_);
	roi_.filter(*cloud_);
	//只留下PointCloud中x值介於介於x~X之間的點
	roi_.setInputCloud(cloud_);
	roi_.setFilterFieldName("x");
	roi_.setFilterLimits(x_, X_);
	roi_.filter(*cloud_);
}

template<typename PointT>
bool PreProcessing<PointT>::setROIBoundary(std::string file)
{
	std::ifstream fin(file.c_str());
	if(!fin)
		return false;
	
	/* file format
	 * x: x_ X_
	 * y: y_ Y_
	 * z: z_ Z_
	 */
	
	std::string head;
	fin>>head;
	fin>>x_;
	fin>>X_;
	
	fin>>head;
	fin>>y_;
	fin>>Y_;
	
	fin>>head;
	fin>>z_;
	fin>>Z_;
	
	fin.close();
	
	return true;
}

//Down sampling
template<typename PointT>
void PreProcessing<PointT>::down_sampling()
{
	down_sampling_.setLeafSize (leaf_, leaf_, leaf_);
	down_sampling_.setInputCloud(cloud_);
	down_sampling_.filter(*cloud_);
}

//Outlier removal
template<typename PointT>
void PreProcessing<PointT>::outlier_removal()
{
	outlier_removal_.setMeanK(meanK_);
	outlier_removal_.setStddevMulThresh(stddev_mult_);
	outlier_removal_.setInputCloud(cloud_);
	outlier_removal_.filter (*cloud_);
	
}

//Plane segmentation
template<typename PointT>
void PreProcessing<PointT>::plane_segmentation()
{
	plane_segmentation_.setModelType(pcl::SACMODEL_PLANE);
	plane_segmentation_.setMethodType(pcl::SAC_RANSAC);
	plane_segmentation_.setDistanceThreshold(distance_threshold_);
	plane_segmentation_.setInputCloud(cloud_);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	plane_segmentation_.segment(*inliers, *plane_coeff_);
	//printf("Plane : (%f, %f, %f, %f)\n", plane_coeff_->values[0], plane_coeff_->values[1], plane_coeff_->values[2], plane_coeff_->values[3]);
	
	extract_indices_.setInputCloud(cloud_);
	extract_indices_.setIndices(inliers);
	extract_indices_.setNegative(true);
	extract_indices_.filter(*cloud_);
	
}

template<typename PointT>
void PreProcessing<PointT>::getPlaneCoeff(pcl::ModelCoefficients::Ptr& plane_coeff)
{
	plane_coeff->values = plane_coeff_->values;
}

//Cluster extraction
template<typename PointT>
void PreProcessing<PointT>::cluster_extraction()
{
	cluster_extraction_.setMinClusterSize(min_pts_);
	cluster_extraction_.setMaxClusterSize(max_pts_);
	cluster_extraction_.setClusterTolerance(cluster_tolerance_);
	typename pcl::search::KdTree<PointT>::Ptr ece_tree(new typename pcl::search::KdTree<PointT>);
	ece_tree->setInputCloud(cloud_);
	cluster_extraction_.setSearchMethod(ece_tree);
	cluster_extraction_.setInputCloud(cloud_);
	std::vector<pcl::PointIndices> all_clusters_indices;
	cluster_extraction_.extract(all_clusters_indices);
	
	PointCloudT cluster_i;
	clusters_.clear();
	for(unsigned int i = 0; i < all_clusters_indices.size(); ++i)
	{
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);
		*indices = all_clusters_indices[i];
		extract_indices_.setInputCloud(cloud_);
		extract_indices_.setIndices(indices);
		extract_indices_.setNegative(false);
		extract_indices_.filter(cluster_i);
		clusters_.push_back(cluster_i);
	}
	
}

#endif