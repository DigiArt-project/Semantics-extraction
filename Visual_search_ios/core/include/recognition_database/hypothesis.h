#ifndef HYPOTHESIS
#define HYPOTHESIS

#include <string>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointT>
struct Hypothesis
{
	std::string type;
	typename pcl::PointCloud<PointT> cloud;
	Eigen::Matrix4f pose;
	float fitness;//the higher the better
};

template<typename PointT>
bool compareHypothesis(Hypothesis<PointT> h1, Hypothesis<PointT> h2){return h1.fitness > h2.fitness;};

#endif