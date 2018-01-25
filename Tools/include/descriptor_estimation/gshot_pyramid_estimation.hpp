#ifndef GSHOT_PYRAMID_ESTIMATION
#define GSHOT_PYRAMID_ESTIMATION

#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <cstdlib>
#include <sys/timeb.h>

#include <mgl2/mgl.h>//Desactivate on server
#include "fpyramid.hpp"

//Testing parameters
int octaves = 3;
int subsections = 2;
float starting_resolution = 0.01;
float starting_kp_reso = 0.2;
float starting_descriptor_radius = 0.4;


template<typename PointT, typename DescriptorT = pcl::SHOT352>
class GSHOTPyramidEstimation
{
    
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename pcl::PointCloud<DescriptorT>::Ptr DescriptorCloudPtrT;
    typedef typename DescriptorCloudT::Ptr x;
    typedef typename pcl::PointCloud<pcl::Normal>::Ptr NormalsT;
    
    // A handy typedef.
    typedef pcl::SHOT352 shot;
    
    GSHOTPyramidEstimation():nr_(0.03f),keypoint_grid_(0.1f),descr_rad_(0.2f),resolution_(0.01f),keypoint_grid_size_(1.0f),scale_(true), descriptor_(0){}
    pcl::PointCloud<DescriptorT> getResultDescriptors();
    typename pcl::PointCloud<PointT>::Ptr compute_keypoints(typename pcl::PointCloud<PointT>::Ptr input, float grid_reso, PointT min, PointT max);
    typename pcl::PointCloud<DescriptorT>::Ptr  compute_histogram(typename pcl::PointCloud<PointT>::Ptr object);
    
    void estimate();
    
    void setNormalRadiusSearch(float nr){
        nr_ = nr;
    }
    void setInputCloud(PointCloudT& cloud){
        cloud_ = cloud;
    }
    void setDescriptors(DescriptorCloudPtrT& desc){
        descriptor_ = desc;
    }
    void setIndexes(std::vector<int> idx){
        indexes_ = idx;
    }
    std::vector<int> getIndexex(){
        return indexes_;
    }
    
    
protected:
    typename pcl::SHOTEstimationOMP<PointT, pcl::Normal,shot> shot_;
    pcl::PointCloud<pcl::Normal>::CloudVectorType clusters_normals_;
    pcl::PointCloud<shot>::Ptr descriptor_;
    float nr_;
    float scale_;
    float keypoint_grid_;
    float descr_rad_;
    float resolution_;
    float keypoint_grid_size_;
    double THRESHOLD = 3;
    
    PointCloudT cloud_;
    NormalsT normals_;
    std::vector<int> indexes_;
};

std::vector<double> minMaxScaler(std::vector<double> data){
    std::vector<double> result_min_max;
    auto max = std::max_element(std::begin(data), std::end(data));
    auto min = std::min_element(std::begin(data), std::end(data));
    for (int i = 0; i < data.size(); i++){
        double new_value = (data.at(i) - *min)/(*max - *min);
        result_min_max.push_back(new_value);
    }
    return result_min_max;
}


template<typename PointT, typename DescriptorT>
typename pcl::PointCloud<PointT>::Ptr GSHOTPyramidEstimation<PointT, DescriptorT>::compute_keypoints(typename pcl::PointCloud<PointT>::Ptr input, float grid_reso, PointT min, PointT max) {
    
    int pt_nb_x = (int)((max.x-min.x)/grid_reso+1);
    int pt_nb_y = (int)((max.y-min.y)/grid_reso+1);
    int pt_nb_z = (int)((max.z-min.z)/grid_reso+1);
    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;
    typename pcl::PointCloud<PointT>::Ptr keypoints (new pcl::PointCloud<PointT> (pt_nb,1,PointT()));
    
    for(unsigned i=0;i<pt_nb_x;i++){
        for(unsigned j=0;j<pt_nb_y;j++){
            for(unsigned k=0;k<pt_nb_z;k++){
                PointT p = PointT(255,255,255);
                p.x = min.x + i*grid_reso;
                p.y = min.y + j*grid_reso;
                p.z = min.z + k*grid_reso;
                keypoints->at(pt_nb_y*pt_nb_z*i + pt_nb_z*j + k) = p;
            }
        }
    }
    
    return keypoints;
}

template<typename PointT, typename DescriptorT>
typename pcl::PointCloud<DescriptorT>::Ptr GSHOTPyramidEstimation<PointT, DescriptorT>::compute_histogram(typename pcl::PointCloud<PointT>::Ptr object){
    
    
    Fpyramid<DescriptorT, PointT>* pyr(new Fpyramid<DescriptorT, PointT>(object, octaves, subsections, starting_resolution, starting_kp_reso,starting_descriptor_radius));
    pyr->toString();
    
    std::vector<double> feature_vector_final(352);
    std::vector<double> data_tmp,feature_vector_final_scaled;
    std::fill(feature_vector_final.begin(), feature_vector_final.end(), 0);
    
    
    
    // Pyramid validation using histograms
    for(unsigned int w=0;w<pyr->get_height();w++){
        pcl::PointCloud<pcl::SHOT352> descr_cloud = pyr->get_descriptors_layer(w);
        std::vector<double> feature_vector(352);
        for(unsigned int i=0;i<descr_cloud.size();i++){
            for(unsigned int j=0;j<352;j++) {
                if (!pcl_isfinite (descr_cloud[i].descriptor[j])) // Replacing NaNs by zeros
                {
                    continue;
                }
                else {
                    feature_vector[j] += descr_cloud[i].descriptor[j];
                }
            }
        }
        
        float max = 0;
        for(unsigned int i=0;i<352;i++){
            if(feature_vector[i]>max)
                max = feature_vector[i];
        }
        
        //Sum element wise all descriptor
        std::transform (feature_vector_final.begin(), feature_vector_final.end(), feature_vector.begin(), feature_vector_final.begin(), std::plus<double>());
        
        
    }
    if (scale_){
        feature_vector_final = minMaxScaler(feature_vector_final);
    }
    //pcl::PointCloud<pcl::SHOT352> gshots;
    typename pcl::PointCloud<DescriptorT>::Ptr gshots(new  pcl::PointCloud<DescriptorT>());
    gshots->width    = 1;
    gshots->height   = 1;
    gshots->is_dense = false;
    gshots->points.resize (gshots->width * gshots->height);
    for (int j = 0; j < feature_vector_final.size(); j++){
        gshots->points[0].descriptor[j] = feature_vector_final.at(j);
    }
    //Check NANValue
    bool containNANValues = false;
    for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
    {
        if (pcl_isnan(gshots->points[0].descriptor[i])){
            containNANValues = true;
        }
    }
    
    
    return gshots;
}



template<typename PointT, typename DescriptorT>
pcl::PointCloud<DescriptorT> GSHOTPyramidEstimation<PointT, DescriptorT>::getResultDescriptors()
{
    return *descriptor_;
}

template<typename PointT, typename DescriptorT>
void GSHOTPyramidEstimation<PointT, DescriptorT>::estimate()
{
    
    typename pcl::PointCloud<DescriptorT>::Ptr shot_desc(new pcl::PointCloud<DescriptorT>());
    shot_desc = compute_histogram(cloud_.makeShared());
    
    //copyPointCloud(*shot_desc,descriptor);
    setDescriptors(shot_desc);
    
    
}

#endif
