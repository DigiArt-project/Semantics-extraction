#ifndef FPYRAMID_HPP
#define FPYRAMID_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/common_headers.h>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

// Ideas : put a verbose parameter?

/*
 * 3D generalization of the concept of feature pyramid. It posesses two main
 * attributes : a point cloud containing the keypoints position and a point
 * cloud containing the keypoints description.
 */

template <typename DescriptorT, typename PointT> class Fpyramid
{
public:    
    // Default constructor
    Fpyramid();

    // Destructor
    ~Fpyramid();

    // Copy constructor
    Fpyramid(const Fpyramid<DescriptorT, PointT>&);

    /*
     * Main constructor, using the point cloud input, the descriptor applied, the number of octaves and the number of subsections by octave.
     * We use a starting resolution to pre sample the point cloud, in order to reduce computation and define a standard.
     * Need to take a container of parameters instead of the descr_rad to generalize to different descriptors.
     */
    Fpyramid(typename pcl::PointCloud<PointT>::Ptr input,  int octaves, int subsections, float starting_resolution, float starting_kp_grid_reso, float descr_rad);

    // Getters
    pcl::PointCloud<DescriptorT> get_descriptors_layer(unsigned int);
    pcl::PointCloud<PointT> get_keypoints_layer(unsigned int);
    int get_octaves();
    int get_sub_between_octaves();
    int get_height();
    float get_original_resolution();
    float get_layer_resolution (int i);

    //Type accessors
    static const char* get_descriptor_type();
    static const char* get_point_type();

    // Pyramid informations
    void toString();

    // Define the subsampling used to compute the keypoints (need to determine a standard)
private:

    // Method for computing feature spaces. Will have different implementations depending on the descriptor used.
    typename pcl::PointCloud<DescriptorT>::Ptr compute_space(typename pcl::PointCloud<PointT>::Ptr input, typename pcl::PointCloud<PointT>::Ptr keypoints, float descr_rad);

    // Method creating the keypoint grid using the min/max values of the input
    typename pcl::PointCloud<PointT>::Ptr compute_keypoints(typename pcl::PointCloud<PointT>::Ptr input, float grid_reso, PointT min, PointT max);

    // Container of the different descriptor layers from 0 (original resolution) to n (lowest resolution, last octave)
    std::vector<typename pcl::PointCloud<DescriptorT>::Ptr >* _descriptors;

    // Container of the different keypoint layers from 0 (original resolution) to n (lowest resolution, last octave)
    std::vector<typename pcl::PointCloud<PointT>::Ptr >* _keypoints;

    int _octaves;
    float _original_resolution;
};

template<typename DescriptorT, typename PointT>
Fpyramid<DescriptorT, PointT>::Fpyramid(): _octaves(0), _original_resolution(0)
{
    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorT>::Ptr >();
    _keypoints = new std::vector<typename pcl::PointCloud<PointT>::Ptr >();
}
template<typename DescriptorT, typename PointT>
Fpyramid<DescriptorT, PointT>::~Fpyramid()
{
    _descriptors->clear();
    _keypoints->clear();
}
template<typename DescriptorT, typename PointT>
Fpyramid<DescriptorT, PointT>::Fpyramid(const Fpyramid<DescriptorT, PointT> &fp)
{
    *this = fp;
}

template<typename DescriptorT, typename PointT>
Fpyramid<DescriptorT, PointT>::Fpyramid(typename pcl::PointCloud<PointT>::Ptr input, int octaves, int subsections, float starting_resolution, float starting_kp_grid_reso, float starting_descr_rad): _octaves(octaves), _original_resolution(starting_resolution)
{
    if(octaves < 0 || subsections < 0 || starting_resolution < 0) {
        std::cerr<<"Arguments of the constructor cannot be negative!"<<std::endl;
        exit(0);
    }
    if(octaves ==0) {
        std::cerr<<"Not very useful to build a pyramid with no octaves..."<<std::endl;
        exit(0);
    }

    _descriptors = new std::vector<typename pcl::PointCloud<DescriptorT>::Ptr >();
    _keypoints = new std::vector<typename pcl::PointCloud<PointT>::Ptr >();

    PointT min;
    PointT max;

    typename pcl::PointCloud<PointT>::Ptr normalized_input(new pcl::PointCloud<PointT>());

    //Define a resolution standard. Not advised to use full resolution.
    if(starting_resolution>0) {

        pcl::UniformSampling<PointT> pre_sampling;
        pre_sampling.setInputCloud (input);
        pre_sampling.setRadiusSearch (starting_resolution);
        pre_sampling.filter (*normalized_input);

        pcl::getMinMax3D(*normalized_input,min,max);

        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
        _descriptors->push_back(this->compute_space(normalized_input,_keypoints->at(0),starting_descr_rad));
    }
    else {
        typename pcl::PointCloud<PointT>::Ptr normalized_input = input;

        pcl::getMinMax3D(*normalized_input,min,max);

        _keypoints->push_back(this->compute_keypoints(normalized_input, starting_kp_grid_reso, min, max));
        _descriptors->push_back(this->compute_space(normalized_input,_keypoints->at(0),starting_descr_rad));
    }
    float resolution;
    float kp_resolution;
    float descr_rad;
    for(unsigned int i=1;i<=octaves;i++){
        resolution = starting_resolution*pow(2,i-1);
        kp_resolution = starting_kp_grid_reso*i;
        descr_rad = starting_descr_rad*i;
        for(unsigned int j=0;j<=subsections;j++){
            typename pcl::PointCloud<PointT>::Ptr subspace(new typename pcl::PointCloud<PointT>());
            float subresolution = resolution;
            float sub_kp_resolution = kp_resolution;
            float sub_descr_rad = descr_rad;
            pcl::UniformSampling<PointT> sampling;

            subresolution += (j+1)*resolution/(subsections+1);
            sub_kp_resolution += (j+1)*kp_resolution/(subsections+1);
            sub_descr_rad += (j+1)*descr_rad/(subsections+1);

            sampling.setInputCloud(normalized_input);
            sampling.setRadiusSearch (subresolution);
            sampling.filter(*subspace);

            _keypoints->push_back(compute_keypoints(normalized_input, sub_kp_resolution, min, max));
            _descriptors->push_back(compute_space(subspace,_keypoints->back(),sub_descr_rad));
        }
    }
}
/*
 * WARNING : need to build a sub structure to partially specialize the pyramid
 */
template<typename DescriptorT, typename PointT>
typename pcl::PointCloud<DescriptorT>::Ptr Fpyramid<DescriptorT, PointT>::compute_space(typename pcl::PointCloud<PointT>::Ptr input, typename pcl::PointCloud<PointT>::Ptr keypoints, float descr_rad)
{
    typename pcl::PointCloud<DescriptorT>::Ptr descriptors (new pcl::PointCloud<DescriptorT> ());
    typename pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
    //Desactivate on server normalestimationomp
#if defined(_OPENMP)
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setNumberOfThreads(omp_get_max_threads());
    norm_est.setKSearch (8);
#else
    //pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> norm_est;
    pcl::NormalEstimation<PointT, pcl::Normal> norm_est;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (0.03);
#endif
    
    norm_est.setInputCloud (input);

    norm_est.compute (*normals);

#if defined(_OPENMP)
    pcl::SHOTEstimationOMP<PointT, pcl::Normal,DescriptorT> descr_est;
    descr_est.setNumberOfThreads(omp_get_max_threads());
#else
    pcl::SHOTEstimation<PointT, pcl::Normal,DescriptorT> descr_est;
#endif

    descr_est.setRadiusSearch (descr_rad);
    descr_est.setInputCloud (keypoints);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (input);
    descr_est.compute (*descriptors);

    return descriptors;
}

template<typename DescriptorT, typename PointT>
typename pcl::PointCloud<PointT>::Ptr Fpyramid<DescriptorT, PointT>::compute_keypoints(typename pcl::PointCloud<PointT>::Ptr input, float grid_reso, PointT min, PointT max) {

    int pt_nb_x = (int)((max.x-min.x)/grid_reso+1);
    int pt_nb_y = (int)((max.y-min.y)/grid_reso+1);
    int pt_nb_z = (int)((max.z-min.z)/grid_reso+1);
    int pt_nb = pt_nb_x*pt_nb_y*pt_nb_z;
    typename pcl::PointCloud<PointT>::Ptr keypoints (new pcl::PointCloud<PointT> (pt_nb,1,PointT()));

    unsigned int i;
#if defined(_OPENMP)
#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
    for(i=0;i<pt_nb_x;i++){
        unsigned int j;
        for(j=0;j<pt_nb_y;j++){
            unsigned int k;
            for(k=0;k<pt_nb_z;k++){
                PointT p = PointT();
                p.x = min.x + i*grid_reso;
                p.y = min.y + j*grid_reso;
                p.z = min.z + k*grid_reso;
                float val = pt_nb_y*pt_nb_z*i + pt_nb_z*j + k;
                //std::cout << "Index : " << val << std::endl;
                //std::cout << "Keypoints size " << keypoints->size() << std::endl;
                keypoints->at(pt_nb_y*pt_nb_z*i + pt_nb_z*j + k) = p;
            }
        }
    }

    return keypoints;
}

template<typename DescriptorT, typename PointT>
pcl::PointCloud<DescriptorT> Fpyramid<DescriptorT, PointT>::get_descriptors_layer(unsigned int i)
{
    return *(_descriptors->at(i));
}
template<typename DescriptorT, typename PointT>
pcl::PointCloud<PointT> Fpyramid<DescriptorT, PointT>::get_keypoints_layer(unsigned int i)
{
    return *(_keypoints->at(i));
}
template<typename DescriptorT, typename PointT>
int Fpyramid<DescriptorT, PointT>::get_octaves()
{
    return _octaves;
}
template<typename DescriptorT, typename PointT>
int Fpyramid<DescriptorT, PointT>::get_sub_between_octaves()
{
    return (_descriptors->size()-1)/(_octaves+1);
}
template<typename DescriptorT, typename PointT>
int Fpyramid<DescriptorT, PointT>::get_height()
{
    return _keypoints->size();
}
template<typename DescriptorT, typename PointT>
float Fpyramid<DescriptorT, PointT>::get_original_resolution()
{
    return _original_resolution;
}
template<typename DescriptorT, typename PointT>
float Fpyramid<DescriptorT, PointT>::get_layer_resolution(int i)
{
    int oct = (int) (i/get_octaves());
    int pos_in_octave = i%get_octaves();
    float dist_btw_octave = _original_resolution/pow(2,oct) - _original_resolution/pow(2,oct+1);
    return _original_resolution/pow(2,oct) - dist_btw_octave*pos_in_octave/(get_sub_between_octaves()+1);
}
/*
 * Need to declare every descriptor we intend to use
 */
template<typename DescriptorT, typename PointT>
const char* Fpyramid<DescriptorT, PointT>::get_descriptor_type()
{
    return typeid(DescriptorT).name();
}
template<typename DescriptorT, typename PointT>
const char* Fpyramid<DescriptorT, PointT>::get_point_type()
{
    return typeid(PointT).name();
}
template<>
const char* Fpyramid<pcl::SHOT352, pcl::PointXYZRGB>::get_descriptor_type()
{
    return "SHOT352";
}
template<>
const char* Fpyramid<pcl::SHOT352, pcl::PointXYZRGB>::get_point_type()
{
    return "PointXYZRGB";
}

template<typename DescriptorT, typename PointT>
void Fpyramid<DescriptorT, PointT>::toString()
{

    std::cout<<"Pyramid specifications :"<<std::endl<<std::endl;
    std::cout<<"Descriptor type : "<<this->get_descriptor_type()<<std::endl;
    std::cout<<"Point type : "<<this->get_point_type()<<std::endl;
    std::cout<<"Number of octaves : "<<this->_octaves<<std::endl;
    std::cout<<"Height : "<<this->_descriptors->size()<<" layers ("<<get_sub_between_octaves()<<" subsections between octaves)"<<std::endl;
    if(_original_resolution==0) {
        std::cout<<"No presampling used, pyramid starting at the original resolution."<<std::endl;
    }
    else {
        std::cout << "Resolution of the first floor : "<<_original_resolution<<std::endl;
    }
}

#endif // FPYRAMID_HPP
