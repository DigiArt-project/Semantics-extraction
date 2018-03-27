// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <others/typedefs.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>

//Descriptors
#include "descriptor_estimation/vfh_estimation.hpp"
#include "descriptor_estimation/cvfh_estimation.hpp"
#include "descriptor_estimation/ourcvfh_estimation.hpp"
#include "descriptor_estimation/esf_estimation.hpp"
#include "descriptor_estimation/spin_estimation.hpp"
#include "descriptor_estimation/usc_estimation.hpp"
#include "descriptor_estimation/sc_estimation.hpp"
#include "descriptor_estimation/crh_estimation.hpp"
#include "descriptor_estimation/grsd_estimation.hpp"
#include "descriptor_estimation/gshot_estimation.hpp"
#include "descriptor_estimation/gshot_pyramid_estimation.hpp"
#include "descriptor_estimation/volume_area_estimation.hpp"
#include "descriptor_estimation/good_estimation.cpp"
#include <pcl/features/shot.h>
//EIGEN
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"

//BOOST
#include <boost/algorithm/string.hpp>

//Timer
#include <others/Timer.hpp>

#include <math.h>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ PointT;

std::string output = "descriptor.pcd";
//If want to scale descriptor between 0 and 1
bool scale_descriptor = false;
bool normalize_cloud = false;
//If instead of computing one point cloud, you want to compute inside folder
bool compute_all = true;
std::string dataset_folder = "/Users/lironesamoun/digiArt/Datasets/dataset_potterymix_normalized";
bool onFull = true;
bool enable_filtering = true;
float leaf_size = 0.01f;



boost::shared_ptr<pcl::visualization::PCLVisualizer> visualise_both_cloud (pcl::PointCloud<PointT>::ConstPtr cloud1,pcl::PointCloud<PointT>::ConstPtr cloud2)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Resolution Viewer"));
    viewer->initCameraParameters ();
    
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    std::string number_points_cloud1 = "Number of points : " + std::to_string(cloud1->size());
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color1, "sample cloud1", v1);
    
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    std::string number_points_cloud2 = " Number of points : " + std::to_string(cloud2->size());
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2", v2);
    
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    
    
    return (viewer);
}
std::vector<float> minMaxScaler(std::vector<float> data){
    std::vector<float> result_min_max;
    auto max = std::max_element(std::begin(data), std::end(data));
    auto min = std::min_element(std::begin(data), std::end(data));
    for (int i = 0; i < data.size(); i++){
        float new_value = (data.at(i) - *min)/(*max - *min);
        result_min_max.push_back(new_value);
    }
    return result_min_max;
}



//Read point cloud from a path
int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud)
{
    std::string extension = boost::filesystem::extension(object_path);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else if (extension == ".ply" || extension == ".PLY")
    {
        if (pcl::io::loadPLYFile(object_path , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else
    {
        std::cout << "\n file extension is not correct. Syntax is: compute_descriptor_cloud_main <path/file_name.pcd> [--nogui] or compute_descriptor_cloud_main <path/file_name.ply> [--nogui]" << std::endl;
        return -1;
    }
    return 1;
}
//Check if the descriptor is valid
bool checkValidityDescriptor(std::string name_desc){
    boost::algorithm::to_lower(name_desc);
    if (name_desc != "esf" && name_desc != "vfh" && name_desc != "cvfh" && name_desc != "ourcvfh" && name_desc != "good" && name_desc != "gshot"
        && name_desc != "shot" && name_desc != "good" && name_desc != "grsd" && name_desc != "usc" && name_desc != "sc3d"){
        
        std::cout << "\n Descriptor not valid : " << name_desc << std::endl;
        return false;
    }else {
        return true;
    }
}

void removeSubstrs(string& s, string& p) {
    string::size_type n = p.length();
    for (string::size_type i = s.find(p);
         i != string::npos;
         i = s.find(p))
        s.erase(i, n);
}

void normalizePC( const char* folderPath, const char* newFilePath){
    std::cout<<"Open old file ..."<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>());
    if( pcl::io::loadPLYFile( folderPath, *cloud) < 0){
        std::cerr << "Error loading point cloud " << folderPath << std::endl << std::endl;
        return;
    }
    
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);
    std::cout<<"centroid :"<<centroid.x<<", "<<centroid.y<<", "<<centroid.z<<std::endl;
    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud( cloud, NULL);
    kdtree.nearestKSearch( centroid, cloud->size(), indices, distances);
    std::cout<<"scale factor :"<<distances.back()<<std::endl;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << -centroid.x/sqrt(distances.back()), -centroid.y/sqrt(distances.back()), -centroid.z/sqrt(distances.back());
    transform.scale(1/sqrt(distances.back()));
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    pcl::io::savePLYFile( newFilePath, *transformed_cloud);
}

void normalizePointCloud(pcl::PointCloud<PointT>::Ptr& cloud){
    //Compute Centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    //Demean Point cloud
    pcl::PointCloud<PointT>::Ptr cloud_demean (new pcl::PointCloud<PointT>);
    pcl::demeanPointCloud(*cloud,centroid,*cloud_demean);
    //Normalize by its root mean square distance to the origin
    int numberPoints = cloud_demean->size();
    float value_scale = 1;
    
    float accumX = 0.;
    float accumY = 0.;
    float accumZ = 0.;
    for(pcl::PointCloud<PointT>::iterator it = cloud_demean->begin(); it!= cloud_demean->end(); it++){
        float x = it->x;
        float y = it->y;
        float z = it->z;
        accumX += x * x;
        accumY += y * y;
        accumZ += z * z;
    }
    //Ps = Ps*np.sqrt(Ps.shape[1]/np.sum(Ps**2))
    float sumTotalNorm = accumX + accumY + accumZ;
    float res = numberPoints/sumTotalNorm;
    res =  sqrt (res);
    float scale = value_scale/(sqrt(sumTotalNorm));
    
    for(size_t i = 0; i < numberPoints; ++i){
        float x = cloud_demean->points[i].x;
        float y = cloud_demean->points[i].y;
        float z = cloud_demean->points[i].z;
        
        cloud_demean->points[i].x = x * res * scale ;
        cloud_demean->points[i].y = y * res  * scale ;
        cloud_demean->points[i].z = z * res  * scale ;
        
    }
    cloud = cloud_demean;
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud( cloud, NULL);
    kdtree.nearestKSearch( pcl::PointXYZ(0,0,0), cloud->size(), indices, distances);
    std::cout<<"scale factor :"<<distances.back()<<std::endl;
    
    
    
}

Eigen::Vector4f computeCentroid( pcl::PointCloud<PointT>::Ptr input_cloud){
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud, centroid);
    
    return centroid;
}



void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Compute global descriptor of point cloud              *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << "-cloud pointCloud -descriptor -scale toScale" << std::endl << std::endl;
    std::cout << "Descriptors available : ESF, VFH, CVFH, OURCVFH, GSHOT, GOOD, GRSD, USC, SC3D" << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -scale:                 true/false if scale descriptor between 0 and 1 (defaut : true)" << std::endl;
    std::cout << "     -output:                location to save the descriptor (defaut : in the current directory)" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

int
main (int argc, char** argv)
{
    
    VFHEstimation<PointT> vfh_estimation;
    CVFHEstimation<PointT> cvfh_estimation;
    OURCVFHEstimation<PointT> ourcvfh_estimation;
    ESFEstimation<PointT> esf_estimation;
    SPINEstimation<PointT> spin_estimation;
    USCEstimation<PointT> usc_estimation;
    SC3DEstimation<PointT> sc3D_estimation;
    GRSDEstimation<PointT> grsd_estimation;
    CRHEstimation<PointT> crh_estimation;
    GSHOTEstimation<PointT> gshot_estimation;
    GSHOTPyramidEstimation<PointT> gshot_pyramid_estimation;
    GOODEstimation<PointT> good_estimation;
    
    if (compute_all){
        
        std::string fullOrViews = "";
        if (onFull){
            fullOrViews="/full/";
        }else {
            fullOrViews="/views/";
        }
        std::cout << "COMPUTE ALL" << std::endl;
        //Activate the descriptor you want
        bool enable_esf = true;
        bool enable_vfh = true;
        bool enable_cvfh = false;
        bool enable_ourcvfh = true;
        bool enable_grsd = false;
        bool enable_gshot = true;
        bool enable_gshot_pyramid = false;
        bool enable_good = true;
        bool enable_usc = false;
        bool enable_sc3D = false;
        
        if (!boost::filesystem::exists (dataset_folder) && !boost::filesystem::is_directory (dataset_folder)){
            std::cerr << "Error with the directory" << std::endl;
            return -1;
        }
        
        for(boost::filesystem::recursive_directory_iterator it(dataset_folder); it!=boost::filesystem::recursive_directory_iterator(); ++it)
        {
            
            std::string path_current = it->path().c_str();
            
            if (path_current.find(fullOrViews) != std::string::npos){
                std::cout << path_current << std::endl;
                
                if (it->path().extension().string() == ".ply" ){
                    //Path to the cloud
                    std::string cloud_path = it->path().string();
                    std::string parent_directory = it->path().parent_path().string();
                    
                    std::string filename_parent_directory = it->path().parent_path().stem().string();
                    std::cout << "Parent directory : " << parent_directory << std::endl;
                    
                    std::string descriptor_folder  = parent_directory + "/descriptors/";
                    if (!boost::filesystem::exists (descriptor_folder) && !boost::filesystem::is_directory (descriptor_folder)){
                        boost::filesystem::create_directory(descriptor_folder);
                    }
                    
                    std::string filename=it->path().stem().string();;
                    
                    std::string pattern = "view_";
                    removeSubstrs(filename, pattern);
                    
                    std::cout << "Name cloud : " << filename << std::endl;
                    std::cout << "Processing : " << cloud_path << std::endl;
                    
                    
                    
                    if (!boost::filesystem::exists (descriptor_folder) && !boost::filesystem::is_directory (descriptor_folder)){
                        boost::filesystem::create_directory(descriptor_folder);
                        //std::cerr << "Error with the descriptor folder. You must create it." << std::endl;
                    }
                    
                    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
                    if (readPointCloud( cloud_path,  cloud)==-1)
                        continue;
                    
                    
                    if (cloud->size() > 18){
                        
                        
                        if (enable_filtering){
                            //down sample the current view
                            pcl::VoxelGrid<pcl::PointXYZ> down;
                            down.setLeafSize (leaf_size, leaf_size, leaf_size);
                            down.setInputCloud (cloud);
                            down.filter (*cloud);
                            
                        }
                        
                        
                        Timer timer_alldescriptor;
                        timer_alldescriptor.startTimer();
                        
                        if (enable_esf){
                            
                            Timer timerESF;
                            timerESF.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            std::cout << "Computing ESF" << std::endl;
                            std::string descriptor_folder_esf = descriptor_folder + "esf/";
                            
                            std::string output = descriptor_folder_esf + name_descriptor;
                            if ( !boost::filesystem::exists(output) )
                            {
                                if (!boost::filesystem::exists (descriptor_folder_esf)){
                                    boost::filesystem::create_directory(descriptor_folder_esf);
                                }
                                esf_estimation.setInputCluster(*cloud);
                                esf_estimation.estimate();
                                pcl::PointCloud<pcl::ESFSignature640> esfs;
                                esf_estimation.getResultDescriptors(esfs);
                                
                                timerESF.stopTimer();
                                std::cout << "[INFO] Time : " << timerESF.getTime() << std::endl;
                                
                                
                                if (scale_descriptor){
                                    std::vector<float> data_tmp,value_descriptor_scaled;
                                    for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
                                    {
                                        data_tmp.push_back(esfs.points[0].histogram[i]);
                                    }
                                    value_descriptor_scaled = minMaxScaler(data_tmp);
                                    for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                        esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                                    }
                                }
                                bool containNANValues = false;
                                for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
                                {
                                    if (pcl_isnan(esfs.points[0].histogram[i])){
                                        containNANValues = true;
                                        //std::cout << "NAAAAAN VALUE" << std::endl;
                                    }
                                }
                                if (!containNANValues){
                                    
                                    
                                    std::cout << "Save to " << output << std::endl;
                                    pcl::io::savePCDFileASCII<pcl::ESFSignature640>(output, esfs);
                                    
                                }else {
                                    continue;
                                }
                            }
                        }
                        if (enable_vfh){
                            Timer timerVFH;
                            timerVFH.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing VFH" << std::endl;
                            std::string descriptor_folder_vfh = descriptor_folder + "vfh/";
                            std::string output = descriptor_folder_vfh + name_descriptor;
                            if ( !boost::filesystem::exists(output) )
                            {
                                
                                if (!boost::filesystem::exists (descriptor_folder_vfh)){
                                    boost::filesystem::create_directory(descriptor_folder_vfh);
                                    std::cout << "CREATE" << std::endl;
                                }
                                
                                vfh_estimation.setInputCluster(*cloud);
                                vfh_estimation.estimate();
                                pcl::PointCloud<pcl::VFHSignature308> vfhs;
                                vfh_estimation.getResultDescriptors(vfhs);
                                
                                timerVFH.stopTimer();
                                std::cout << "[INFO] Time : " << timerVFH.getTime() << std::endl;
                                
                                
                                if (scale_descriptor){
                                    std::vector<float> data_tmp,value_descriptor_scaled;
                                    for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                                    {
                                        data_tmp.push_back(vfhs.points[0].histogram[i]);
                                    }
                                    value_descriptor_scaled = minMaxScaler(data_tmp);
                                    for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                        vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                                    }
                                }
                                bool containNANValues = false;
                                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                                {
                                    if (pcl_isnan(vfhs.points[0].histogram[i])){
                                        containNANValues = true;
                                        //std::cout << "NAAAAAN VALUE" << std::endl;
                                    }
                                }
                                if (!containNANValues){
                                    
                                    
                                    pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, vfhs);
                                }else {
                                    continue;
                                }
                            }
                        }
                        if (enable_ourcvfh){
                            Timer timerOURCVFH;
                            timerOURCVFH.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            //ourcvfh
                            std::cout << "Computing OURCVFH" << std::endl;
                            std::string descriptor_folder_ourcvfh = descriptor_folder + "ourcvfh/";
                            if (!boost::filesystem::exists (descriptor_folder_ourcvfh)){
                                boost::filesystem::create_directory(descriptor_folder_ourcvfh);
                            }
                            
                            ourcvfh_estimation.setInputCluster(*cloud);
                            ourcvfh_estimation.estimate();
                            pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
                            ourcvfh_estimation.getResultDescriptors(ourcvfhs);
                            
                            
                            timerOURCVFH.stopTimer();
                            std::cout << "[INFO] Time : " << timerOURCVFH.getTime() << std::endl;
                            
                            
                            if (scale_descriptor){
                                std::vector<float> data_tmp,value_descriptor_scaled;
                                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                                {
                                    data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
                                }
                                value_descriptor_scaled = minMaxScaler(data_tmp);
                                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                    ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                                }
                            }
                            bool containNANValues = false;
                            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                            {
                                if (pcl_isnan(ourcvfhs.points[0].histogram[i])){
                                    containNANValues = true;
                                    //std::cout << "NAAAAAN VALUE" << std::endl;
                                }
                            }
                            if (!containNANValues){
                                std::string output = descriptor_folder_ourcvfh + name_descriptor;
                                pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, ourcvfhs);
                            }else {
                                continue;
                            }
                            
                        }
                        
                        if (enable_cvfh){
                            Timer timerCVFH;
                            timerCVFH.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing CVFH" << std::endl;
                            std::string descriptor_folder_cvfh = descriptor_folder + "cvfh/";
                            if (!boost::filesystem::exists (descriptor_folder_cvfh)){
                                boost::filesystem::create_directory(descriptor_folder_cvfh);
                            }
                            std::string output = descriptor_folder_cvfh + name_descriptor;
                            /* if (boost::filesystem::exists(output ) )
                             {
                             std::cout << "Already computed" << std::endl;
                             continue;
                             }*/
                            cvfh_estimation.setInputCluster(*cloud);
                            cvfh_estimation.estimate();
                            pcl::PointCloud<pcl::VFHSignature308> cvfhs;
                            cvfh_estimation.getResultDescriptors(cvfhs);
                            
                            timerCVFH.stopTimer();
                            std::cout << "[INFO] Time : " << timerCVFH.getTime() << std::endl;
                            
                            
                            if (scale_descriptor){
                                std::vector<float> data_tmp,value_descriptor_scaled;
                                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                                {
                                    data_tmp.push_back(cvfhs.points[0].histogram[i]);
                                }
                                value_descriptor_scaled = minMaxScaler(data_tmp);
                                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                    cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                                }
                            }
                            bool containNANValues = false;
                            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                            {
                                if (pcl_isnan(cvfhs.points[0].histogram[i])){
                                    containNANValues = true;
                                    //std::cout << "NAAAAAN VALUE" << std::endl;
                                }
                            }
                            if (!containNANValues){
                                
                                
                                pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, cvfhs);
                            }else {
                                continue;
                            }
                            
                        }
                        if (enable_grsd){
                            
                            Timer timerGRSD;
                            timerGRSD.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing GRSD" << std::endl;
                            std::string descriptor_folder_grsd = descriptor_folder + "grsd/";
                            if (!boost::filesystem::exists (descriptor_folder_grsd)){
                                boost::filesystem::create_directory(descriptor_folder_grsd);
                            }
                            
                            grsd_estimation.setInputCluster(*cloud);
                            grsd_estimation.estimate();
                            pcl::PointCloud<pcl::GRSDSignature21> grsd;
                            grsd_estimation.getResultDescriptors(grsd);
                            
                            timerGRSD.stopTimer();
                            std::cout << "[INFO] Time : " << timerGRSD.getTime() << std::endl;
                            
                            
                            if (scale_descriptor){
                                std::vector<float> data_tmp,value_descriptor_scaled;
                                for (size_t i = 0; i <pcl::GRSDSignature21::descriptorSize(); i++)
                                {
                                    data_tmp.push_back(grsd.points[0].histogram[i]);
                                }
                                value_descriptor_scaled = minMaxScaler(data_tmp);
                                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                    grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
                                }
                            }
                            bool containNANValues = false;
                            for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
                            {
                                if (pcl_isnan(grsd.points[0].histogram[i])){
                                    containNANValues = true;
                                    //std::cout << "NAAAAAN VALUE" << std::endl;
                                }
                            }
                            if (!containNANValues){
                                std::string output = descriptor_folder_grsd + name_descriptor;
                                pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(output, grsd);
                            }else {
                                continue;
                            }
                            
                        }
                        if (enable_gshot){
                            
                            Timer timerGSHOT;
                            timerGSHOT.startTimer();
                            
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing GSHOT" << std::endl;
                            
                            std::string descriptor_folder_gshot = descriptor_folder + "gshot/";
                            std::string output = descriptor_folder_gshot + name_descriptor;
                            if ( !boost::filesystem::exists(output) )
                            {
                                if (!boost::filesystem::exists (descriptor_folder_gshot)){
                                    boost::filesystem::create_directory(descriptor_folder_gshot);
                                }
                                
                                gshot_estimation.setInputCluster(*cloud);
                                gshot_estimation.estimate();
                                pcl::PointCloud<pcl::SHOT352> gshots;
                                gshot_estimation.getResultDescriptors(gshots);
                                
                                timerGSHOT.stopTimer();
                                std::cout << "[INFO] Time : " << timerGSHOT.getTime() << std::endl;
                                
                                if (scale_descriptor){
                                    std::vector<float> data_tmp,value_descriptor_scaled;
                                    for (size_t i = 0; i <pcl::SHOT352::descriptorSize(); i++)
                                    {
                                        data_tmp.push_back(gshots.points[0].descriptor[i]);
                                    }
                                    value_descriptor_scaled = minMaxScaler(data_tmp);
                                    for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                        gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
                                    }
                                }
                                //Check NANValue
                                bool containNANValues = false;
                                for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
                                {
                                    if (pcl_isnan(gshots.points[0].descriptor[i])){
                                        containNANValues = true;
                                        //std::cout << "NAAAAAN VALUE" << std::endl;
                                    }
                                }
                                if (!containNANValues){
                                    
                                    pcl::io::savePCDFileASCII<pcl::SHOT352>(output, gshots);
                                }else {
                                    continue;
                                }
                                
                            }
                        }
                        if (enable_gshot_pyramid){
                            
                            Timer timerGSHOTPyramid;
                            timerGSHOTPyramid.startTimer();
                            
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing GSHOT Pyramid" << std::endl;
                            std::string descriptor_folder_gshot_pyramid = descriptor_folder + "gshotPyramid/";
                            if (!boost::filesystem::exists (descriptor_folder_gshot_pyramid)){
                                boost::filesystem::create_directory(descriptor_folder_gshot_pyramid);
                            }
                            
                            gshot_pyramid_estimation.setInputCloud(*cloud);
                            gshot_pyramid_estimation.estimate();
                            pcl::PointCloud<pcl::SHOT352> gshots = gshot_pyramid_estimation.getResultDescriptors();
                            
                            timerGSHOTPyramid.stopTimer();
                            std::cout << "[INFO] Time : " << timerGSHOTPyramid.getTime() << std::endl;
                            //Check NANValue
                            bool containNANValues = false;
                            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
                            {
                                if (pcl_isnan(gshots.points[0].descriptor[i])){
                                    containNANValues = true;
                                    //std::cout << "NAAAAAN VALUE" << std::endl;
                                }
                            }
                            if (!containNANValues){
                                std::string output = descriptor_folder_gshot_pyramid + name_descriptor;
                                pcl::io::savePCDFileASCII<pcl::SHOT352>(output, gshots);
                            }else {
                                continue;
                            }
                            
                            
                        }
                        if (enable_good){
                            Timer timerGOOD;
                            timerGOOD.startTimer();
                            
                            std::string name_descriptor = "desc_" +filename + ".txt";
                            
                            std::cout << "Computing GOOD" << std::endl;
                            std::string descriptor_folder_good = descriptor_folder + "good/";
                            if (!boost::filesystem::exists (descriptor_folder_good)){
                                boost::filesystem::create_directory(descriptor_folder_good);
                            }
                            good_estimation.setNumberOfBins(5);
                            good_estimation.setThreshold(0.0015);
                            
                            // Provide the original point cloud
                            good_estimation.setInputCloud(cloud);
                            
                            // Compute GOOD discriptor for the given object
                            std::vector< float > object_description;
                            good_estimation.compute(object_description);
                            
                            timerGOOD.stopTimer();
                            std::cout << "[INFO] Time : " << timerGOOD.getTime() << std::endl;
                            
                            if (scale_descriptor){
                                std::vector<float> value_descriptor_scaled;
                                
                                value_descriptor_scaled = minMaxScaler(object_description);
                                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                    object_description.at(j) = value_descriptor_scaled.at(j);
                                }
                            }
                            std::string output = descriptor_folder_good + name_descriptor;
                            std::cout << "OUTPUT " << output << std::endl;
                            ofstream descfile;
                            descfile.open (output,ios::out );
                            copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
                            descfile.close();
                            
                            
                        }
                        if (enable_usc){
                            Timer timerUSC;
                            timerUSC.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing USC" << std::endl;
                            std::string descriptor_folder_usc = descriptor_folder + "usc/";
                            if (!boost::filesystem::exists (descriptor_folder_usc)){
                                boost::filesystem::create_directory(descriptor_folder_usc);
                            }
                            
                            usc_estimation.setInputCloud(*cloud);
                            usc_estimation.estimate();
                            pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
                            usc = usc_estimation.getResultDescriptors();
                            
                            
                            timerUSC.stopTimer();
                            std::cout << "[INFO] Time : " << timerUSC.getTime() << std::endl;
                            
                            std::string output = descriptor_folder_usc + name_descriptor;
                            pcl::io::savePCDFileASCII<pcl::UniqueShapeContext1960>(output, usc);
                            
                            
                        }
                        if (enable_sc3D){
                            Timer timersc3D;
                            timersc3D.startTimer();
                            std::string name_descriptor = "desc_" +filename + ".pcd";
                            
                            std::cout << "Computing sc3D" << std::endl;
                            std::string descriptor_folder_sc3D = descriptor_folder + "sc3D/";
                            if (!boost::filesystem::exists (descriptor_folder_sc3D)){
                                boost::filesystem::create_directory(descriptor_folder_sc3D);
                            }
                            sc3D_estimation.setInputCluster(*cloud);
                            sc3D_estimation.estimate();
                            pcl::PointCloud<pcl::ShapeContext1980> sc;
                            sc3D_estimation.getResultDescriptors(sc);
                            
                            timersc3D.stopTimer();
                            std::cout << "[INFO] Time : " << timersc3D.getTime() << std::endl;
                            
                            if (scale_descriptor){
                                std::vector<float> data_tmp,value_descriptor_scaled;
                                for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
                                {
                                    data_tmp.push_back(sc.points[0].descriptor[i]);
                                }
                                value_descriptor_scaled = minMaxScaler(data_tmp);
                                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                    sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
                                }
                            }
                            //Check NANValue
                            bool containNANValues = false;
                            for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
                            {
                                if (pcl_isnan(sc.points[0].descriptor[i])){
                                    containNANValues = true;
                                    //std::cout << "NAAAAAN VALUE" << std::endl;
                                }
                            }
                            if (!containNANValues){
                                
                                std::string output = descriptor_folder_sc3D + name_descriptor;
                                pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(output, sc);
                            }else {
                                continue;
                            }
                            
                        }
                        
                        timer_alldescriptor.stopTimer();
                        std::cout << "[INFO] Time processed for all descriptors : " << timer_alldescriptor.getTime() << std::endl;
                        
                    }
                    
                }
            }
        }
    }
    
    
    
    else {
        //Show help
        if (pcl::console::find_switch (argc, argv, "-h"))
        {
            showHelp (argv[0]);
            exit (0);
        }
        //If not enough parameters
        if (argc < 4)
        {
            std::cout << "[INFO] Not enough parameters " << std::endl;
            showHelp (argv[0]);
            return (-1);
        }
        
        std::string cloud_path;
        if(pcl::console::parse_argument(argc, argv, "-cloud", cloud_path) == -1)
        {
            std::cerr<<"Please specify the cloud point (pcd or ply)"<<std::endl;
            return -1;
        }
        
        std::string type_descriptor;
        if(pcl::console::parse_argument(argc, argv, "-descriptor", type_descriptor) == -1)
        {
            std::cerr<<"Please specify the name of a descriptor "<<std::endl;
            return -1;
        }
        
        if(pcl::console::parse_argument(argc, argv, "-output", output) == 1)
        {
            std::cout << "Output : " << output << std::endl;
        }
        
        std::string scale;
        if(pcl::console::parse_argument(argc, argv, "-scale", scale) ==  - 1)
        {
            std::cerr<<"Please specify if you want to scale or not  "<<std::endl;
            return -1;
        }
        
        std::cout << "Scale descriptor : " << scale_descriptor << std::endl;
        if (scale == "true"){
            scale_descriptor = true;
        }else {
            scale_descriptor = false;
        }
        std::cout << "Scale descriptor : " << scale_descriptor << std::endl;
        
        
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        
        if (readPointCloud( cloud_path,  cloud)==-1)
            return -1;
        
        if (!checkValidityDescriptor(type_descriptor)){
            return -1;
        }
        if (normalize_cloud){
            std::cout << "Normalize point cloud " << std::endl;
            normalizePointCloud(cloud);
            
        }
        /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = visualise_both_cloud(cloud,cloud);
         //--------------------
         // -----Main loop-----
         //--------------------
         while (!viewer->wasStopped ())
         {
         viewer->spinOnce (100);
         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
         }*/
        
        
        std::cout << "Computing cloud using " << type_descriptor << std::endl;
        if (type_descriptor == "esf"){
            
            esf_estimation.setInputCluster(*cloud);
            esf_estimation.estimate();
            pcl::PointCloud<pcl::ESFSignature640> esfs;
            esf_estimation.getResultDescriptors(esfs);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
                {
                    data_tmp.push_back(esfs.points[0].histogram[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                }
            }
            pcl::io::savePCDFileASCII<pcl::ESFSignature640>(output, esfs);
            
            
        }
        else if (type_descriptor == "vfh"){
            vfh_estimation.setInputCluster(*cloud);
            vfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> vfhs;
            vfh_estimation.getResultDescriptors(vfhs);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                {
                    data_tmp.push_back(vfhs.points[0].histogram[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                }
            }
            
            
            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, vfhs);
            
            
        }
        else if (type_descriptor == "cvfh"){
            cvfh_estimation.setInputCluster(*cloud);
            cvfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> cvfhs;
            cvfh_estimation.getResultDescriptors(cvfhs);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                {
                    data_tmp.push_back(cvfhs.points[0].histogram[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                }
            }
            
            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, cvfhs);
            
            
        }
        else if (type_descriptor == "ourcvfh"){
            //ourcvfh
            ourcvfh_estimation.setInputCluster(*cloud);
            ourcvfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
            ourcvfh_estimation.getResultDescriptors(ourcvfhs);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                {
                    data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                }
            }
            
            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, ourcvfhs);
            
        }
        else if (type_descriptor == "grsd"){
            grsd_estimation.setInputCluster(*cloud);
            grsd_estimation.estimate();
            pcl::PointCloud<pcl::GRSDSignature21> grsd;
            grsd_estimation.getResultDescriptors(grsd);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i <pcl::GRSDSignature21::descriptorSize(); i++)
                {
                    data_tmp.push_back(grsd.points[0].histogram[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
                }
            }
            
            pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(output, grsd);
            
        }
        else if (type_descriptor == "gshot"){
            gshot_estimation.setInputCluster(*cloud);
            gshot_estimation.estimate();
            pcl::PointCloud<pcl::SHOT352> gshots;
            gshot_estimation.getResultDescriptors(gshots);
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i <pcl::SHOT352::descriptorSize(); i++)
                {
                    data_tmp.push_back(gshots.points[0].descriptor[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
                }
            }
            
            pcl::io::savePCDFileBinary<pcl::SHOT352>(output, gshots);
            
            
        }
        else if (type_descriptor == "good"){
            good_estimation.setNumberOfBins(5);
            good_estimation.setThreshold(0.0015);
            
            // Provide the original point cloud
            good_estimation.setInputCloud(cloud);
            
            // Compute GOOD discriptor for the given object
            std::vector< float > object_description;
            good_estimation.compute(object_description);
            
            if (scale_descriptor){
                std::vector<float> value_descriptor_scaled;
                
                value_descriptor_scaled = minMaxScaler(object_description);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    object_description.at(j) = value_descriptor_scaled.at(j);
                }
            }
            ofstream descfile;
            descfile.open (output,ios::out );
            copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
            descfile.close();
            
            
        }
        else if (type_descriptor == "usc"){
            usc_estimation.setInputCloud(*cloud);
            usc_estimation.estimate();
            pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
            usc = usc_estimation.getResultDescriptors();
            
            pcl::io::savePCDFile(output, usc);
            
            
        }
        else if (type_descriptor == "sc3D"){
            sc3D_estimation.setInputCluster(*cloud);
            sc3D_estimation.estimate();
            pcl::PointCloud<pcl::ShapeContext1980> sc;
            sc3D_estimation.getResultDescriptors(sc);
            
            
            if (scale_descriptor){
                std::vector<float> data_tmp,value_descriptor_scaled;
                for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
                {
                    data_tmp.push_back(sc.points[0].descriptor[i]);
                }
                value_descriptor_scaled = minMaxScaler(data_tmp);
                for (int j = 0; j < value_descriptor_scaled.size(); j++){
                    sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
                }
            }
            
            pcl::io::savePCDFileBinary<pcl::ShapeContext1980>(output, sc);
            
            
        }
        
        std::cout << "\n Computing finished "<< std::endl;
        
    }
    
    
    
    
    return 0;
    
    
}




//
//int
//main (int argc, char** argv)
//{
//
//    VFHEstimation<PointT> vfh_estimation;
//    CVFHEstimation<PointT> cvfh_estimation;
//    OURCVFHEstimation<PointT> ourcvfh_estimation;
//    ESFEstimation<PointT> esf_estimation;
//    SPINEstimation<PointT> spin_estimation;
//    USCEstimation<PointT> usc_estimation;
//    SC3DEstimation<PointT> sc3D_estimation;
//    GRSDEstimation<PointT> grsd_estimation;
//    CRHEstimation<PointT> crh_estimation;
//    GSHOTEstimation<PointT> gshot_estimation;
//    GSHOTPyramidEstimation<PointT> gshot_pyramid_estimation;
//    GOODEstimation<PointT> good_estimation;
//    //Normally the program is for only computing a descriptor for a specific point cloud. For convenience, add an option to activate manually if we want to compute descriptors for a series of specific folder (var. subfolder) inside a dataset (var. dataset_folder)
//
//    if (compute_all){
//        std::cout << "COMPUTE ALL" << std::endl;
//        //Activate the descriptor you want
//        bool enable_esf = false;
//        bool enable_vfh = false;
//        bool enable_cvfh = false;
//        bool enable_ourcvfh = false;
//        bool enable_grsd = false;
//        bool enable_gshot = false;
//        bool enable_gshot_pyramid = false;
//        bool enable_good = false;
//        bool enable_usc = false;
//        bool enable_sc3D = false;
//
//        if (!boost::filesystem::exists (dataset_folder) && !boost::filesystem::is_directory (dataset_folder)){
//            std::cerr << "Error with the directory" << std::endl;
//            return -1;
//        }
//        //To uncomment for dataset with no view folder
//        if (!boost::filesystem::exists (output_dataset_folder) && !boost::filesystem::is_directory (output_dataset_folder)){
//            boost::filesystem::create_directory(output_dataset_folder);
//        }
//
//
//        for(boost::filesystem::recursive_directory_iterator it(dataset_folder); it!=boost::filesystem::recursive_directory_iterator(); ++it)
//        {
//
//            std::string path_current = it->path().c_str();
//            std::cout << path_current << std::endl;
//
//            //line to comment for dataset with no view folder
//            //if (path_current.find("/"+subfolder+"/")  != std::string::npos){
//            if (it->path().extension().string() == ".pcd" ){
//                //Path to the cloud
//                std::string cloud_path = it->path().string();
//                std::string parent_directory = it->path().parent_path().parent_path().string();
//
//                //To comment for dataset with no view folder
//                //std::string filename_parent_directory = it->path().parent_path().parent_path().stem().string();
//                //To uncomment for dataset with no view folder
//                std::string filename_parent_directory = it->path().parent_path().stem().string();
//
//                //Where we are going to save the descriptor
//                //To comment for dataset with no view folder
//                //std::string descriptor_folder = it->path().parent_path().parent_path().string() + "/descriptors/";
//
//
//                //To uncomment for dataset with no view folder
//                std::string descriptor_folder = output_dataset_folder + "/" + filename_parent_directory;
//                if (!boost::filesystem::exists (descriptor_folder) && !boost::filesystem::is_directory (descriptor_folder)){
//                    boost::filesystem::create_directory(descriptor_folder);
//                    //std::cerr << "Error with the descriptor folder. You must create it." << std::endl;
//                }
//                //To uncomment for dataset with no view folder
//                descriptor_folder = descriptor_folder + "/descriptors/";
//
//                //filename of the current cloud in order to extract its name
//                std::string filename = it->path().stem().string();
//                std::string pattern = "view_";
//                removeSubstrs(filename, pattern);
//                std::cout << "Name cloud : " << filename << std::endl;
//                std::cout << "Processing : " << cloud_path << std::endl;
//
//
//
//                if (!boost::filesystem::exists (descriptor_folder) && !boost::filesystem::is_directory (descriptor_folder)){
//                    boost::filesystem::create_directory(descriptor_folder);
//                    //std::cerr << "Error with the descriptor folder. You must create it." << std::endl;
//                }
//
//                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//                if (readPointCloud( cloud_path,  cloud)==-1)
//                    continue;
//
//                /*
//                 if (normalize_cloud){
//                 std::cout << "Normalize point cloud " << std::endl;
//                 normalizePointCloud(cloud);
//                 }*/
//
//                if (cloud->size() > 18){
//
//
//                    if (enable_filtering){
//                        //down sample the current view
//                        pcl::VoxelGrid<pcl::PointXYZ> down;
//                        down.setLeafSize (leaf_size, leaf_size, leaf_size);
//                        down.setInputCloud (cloud);
//                        down.filter (*cloud);
//
//                    }
//
//
//                    Timer timer_alldescriptor;
//                    timer_alldescriptor.startTimer();
//
//                    if (enable_esf){
//                        Timer timerESF;
//                        timerESF.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//                        std::cout << "Computing ESF" << std::endl;
//                        std::string descriptor_folder_esf = descriptor_folder + "esf/";
//
//                        if (!boost::filesystem::exists (descriptor_folder_esf)){
//                            boost::filesystem::create_directory(descriptor_folder_esf);
//                        }
//                        esf_estimation.setInputCluster(*cloud);
//                        esf_estimation.estimate();
//                        pcl::PointCloud<pcl::ESFSignature640> esfs;
//                        esf_estimation.getResultDescriptors(esfs);
//
//                        timerESF.stopTimer();
//                        std::cout << "[INFO] Time : " << timerESF.getTime() << std::endl;
//
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(esfs.points[0].histogram[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(esfs.points[0].histogram[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//
//
//                            std::string output = descriptor_folder_esf + name_descriptor;
//                            std::cout << "Save to " << output << std::endl;
//                            pcl::io::savePCDFileASCII<pcl::ESFSignature640>(output, esfs);
//
//                        }else {
//                            continue;
//                        }
//
//                    }
//                    if (enable_vfh){
//                        Timer timerVFH;
//                        timerVFH.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing VFH" << std::endl;
//                        std::string descriptor_folder_vfh = descriptor_folder + "vfh/";
//
//                        if (!boost::filesystem::exists (descriptor_folder_vfh)){
//                            boost::filesystem::create_directory(descriptor_folder_vfh);
//                            std::cout << "CREATE" << std::endl;
//                        }
//
//                        vfh_estimation.setInputCluster(*cloud);
//                        vfh_estimation.estimate();
//                        pcl::PointCloud<pcl::VFHSignature308> vfhs;
//                        vfh_estimation.getResultDescriptors(vfhs);
//
//                        timerVFH.stopTimer();
//                        std::cout << "[INFO] Time : " << timerVFH.getTime() << std::endl;
//
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(vfhs.points[0].histogram[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(vfhs.points[0].histogram[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//
//                            std::string output = descriptor_folder_vfh + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, vfhs);
//                        }else {
//                            continue;
//                        }
//
//                    }
//                    if (enable_ourcvfh){
//                        Timer timerOURCVFH;
//                        timerOURCVFH.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        //ourcvfh
//                        std::cout << "Computing OURCVFH" << std::endl;
//                        std::string descriptor_folder_ourcvfh = descriptor_folder + "ourcvfh/";
//                        if (!boost::filesystem::exists (descriptor_folder_ourcvfh)){
//                            boost::filesystem::create_directory(descriptor_folder_ourcvfh);
//                        }
//
//                        ourcvfh_estimation.setInputCluster(*cloud);
//                        ourcvfh_estimation.estimate();
//                        pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
//                        ourcvfh_estimation.getResultDescriptors(ourcvfhs);
//
//
//                        timerOURCVFH.stopTimer();
//                        std::cout << "[INFO] Time : " << timerOURCVFH.getTime() << std::endl;
//
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(ourcvfhs.points[0].histogram[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//                            std::string output = descriptor_folder_ourcvfh + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, ourcvfhs);
//                        }else {
//                            continue;
//                        }
//
//                    }
//
//                    if (enable_cvfh){
//                        Timer timerCVFH;
//                        timerCVFH.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing CVFH" << std::endl;
//                        std::string descriptor_folder_cvfh = descriptor_folder + "cvfh/";
//                        if (!boost::filesystem::exists (descriptor_folder_cvfh)){
//                            boost::filesystem::create_directory(descriptor_folder_cvfh);
//                        }
//                        std::string output = descriptor_folder_cvfh + name_descriptor;
//                        /* if (boost::filesystem::exists(output ) )
//                         {
//                         std::cout << "Already computed" << std::endl;
//                         continue;
//                         }*/
//                        cvfh_estimation.setInputCluster(*cloud);
//                        cvfh_estimation.estimate();
//                        pcl::PointCloud<pcl::VFHSignature308> cvfhs;
//                        cvfh_estimation.getResultDescriptors(cvfhs);
//
//                        timerCVFH.stopTimer();
//                        std::cout << "[INFO] Time : " << timerCVFH.getTime() << std::endl;
//
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(cvfhs.points[0].histogram[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(cvfhs.points[0].histogram[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//
//
//                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, cvfhs);
//                        }else {
//                            continue;
//                        }
//
//                    }
//                    if (enable_grsd){
//
//                        Timer timerGRSD;
//                        timerGRSD.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing GRSD" << std::endl;
//                        std::string descriptor_folder_grsd = descriptor_folder + "grsd/";
//                        if (!boost::filesystem::exists (descriptor_folder_grsd)){
//                            boost::filesystem::create_directory(descriptor_folder_grsd);
//                        }
//
//                        grsd_estimation.setInputCluster(*cloud);
//                        grsd_estimation.estimate();
//                        pcl::PointCloud<pcl::GRSDSignature21> grsd;
//                        grsd_estimation.getResultDescriptors(grsd);
//
//                        timerGRSD.stopTimer();
//                        std::cout << "[INFO] Time : " << timerGRSD.getTime() << std::endl;
//
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i <pcl::GRSDSignature21::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(grsd.points[0].histogram[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(grsd.points[0].histogram[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//                            std::string output = descriptor_folder_grsd + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(output, grsd);
//                        }else {
//                            continue;
//                        }
//
//                    }
//                    if (enable_gshot){
//
//                        Timer timerGSHOT;
//                        timerGSHOT.startTimer();
//
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing GSHOT" << std::endl;
//                        std::string descriptor_folder_gshot = descriptor_folder + "gshot/";
//                        if (!boost::filesystem::exists (descriptor_folder_gshot)){
//                            boost::filesystem::create_directory(descriptor_folder_gshot);
//                        }
//
//                        gshot_estimation.setInputCluster(*cloud);
//                        gshot_estimation.estimate();
//                        pcl::PointCloud<pcl::SHOT352> gshots;
//                        gshot_estimation.getResultDescriptors(gshots);
//
//                        timerGSHOT.stopTimer();
//                        std::cout << "[INFO] Time : " << timerGSHOT.getTime() << std::endl;
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i <pcl::SHOT352::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(gshots.points[0].descriptor[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        //Check NANValue
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(gshots.points[0].descriptor[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//                            std::string output = descriptor_folder_gshot + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::SHOT352>(output, gshots);
//                        }else {
//                            continue;
//                        }
//
//
//                    }
//                    if (enable_gshot_pyramid){
//
//                        Timer timerGSHOTPyramid;
//                        timerGSHOTPyramid.startTimer();
//
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing GSHOT Pyramid" << std::endl;
//                        std::string descriptor_folder_gshot_pyramid = descriptor_folder + "gshotPyramid/";
//                        if (!boost::filesystem::exists (descriptor_folder_gshot_pyramid)){
//                            boost::filesystem::create_directory(descriptor_folder_gshot_pyramid);
//                        }
//
//                        gshot_pyramid_estimation.setInputCloud(*cloud);
//                        gshot_pyramid_estimation.estimate();
//                        pcl::PointCloud<pcl::SHOT352> gshots = gshot_pyramid_estimation.getResultDescriptors();
//
//                        timerGSHOTPyramid.stopTimer();
//                        std::cout << "[INFO] Time : " << timerGSHOTPyramid.getTime() << std::endl;
//                        //Check NANValue
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(gshots.points[0].descriptor[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//                            std::string output = descriptor_folder_gshot_pyramid + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::SHOT352>(output, gshots);
//                        }else {
//                            continue;
//                        }
//
//
//                    }
//                    if (enable_good){
//                        Timer timerGOOD;
//                        timerGOOD.startTimer();
//
//                        std::string name_descriptor = "desc_" +filename + ".txt";
//
//                        std::cout << "Computing GOOD" << std::endl;
//                        std::string descriptor_folder_good = descriptor_folder + "good/";
//                        if (!boost::filesystem::exists (descriptor_folder_good)){
//                            boost::filesystem::create_directory(descriptor_folder_good);
//                        }
//                        good_estimation.setNumberOfBins(5);
//                        good_estimation.setThreshold(0.0015);
//
//                        // Provide the original point cloud
//                        good_estimation.setInputCloud(cloud);
//
//                        // Compute GOOD discriptor for the given object
//                        std::vector< float > object_description;
//                        good_estimation.compute(object_description);
//
//                        timerGOOD.stopTimer();
//                        std::cout << "[INFO] Time : " << timerGOOD.getTime() << std::endl;
//
//                        if (scale_descriptor){
//                            std::vector<float> value_descriptor_scaled;
//
//                            value_descriptor_scaled = minMaxScaler(object_description);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                object_description.at(j) = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        std::string output = descriptor_folder_good + name_descriptor;
//                        std::cout << "OUTPUT " << output << std::endl;
//                        ofstream descfile;
//                        descfile.open (output,ios::out );
//                        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
//                        descfile.close();
//
//
//                    }
//                    if (enable_usc){
//                        Timer timerUSC;
//                        timerUSC.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing USC" << std::endl;
//                        std::string descriptor_folder_usc = descriptor_folder + "usc/";
//                        if (!boost::filesystem::exists (descriptor_folder_usc)){
//                            boost::filesystem::create_directory(descriptor_folder_usc);
//                        }
//
//                        usc_estimation.setInputCloud(*cloud);
//                        usc_estimation.estimate();
//                        pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
//                        usc = usc_estimation.getResultDescriptors();
//
//
//                        timerUSC.stopTimer();
//                        std::cout << "[INFO] Time : " << timerUSC.getTime() << std::endl;
//
//                        std::string output = descriptor_folder_usc + name_descriptor;
//                        pcl::io::savePCDFileASCII<pcl::UniqueShapeContext1960>(output, usc);
//
//
//                    }
//                    if (enable_sc3D){
//                        Timer timersc3D;
//                        timersc3D.startTimer();
//                        std::string name_descriptor = "desc_" +filename + ".pcd";
//
//                        std::cout << "Computing sc3D" << std::endl;
//                        std::string descriptor_folder_sc3D = descriptor_folder + "sc3D/";
//                        if (!boost::filesystem::exists (descriptor_folder_sc3D)){
//                            boost::filesystem::create_directory(descriptor_folder_sc3D);
//                        }
//                        sc3D_estimation.setInputCluster(*cloud);
//                        sc3D_estimation.estimate();
//                        pcl::PointCloud<pcl::ShapeContext1980> sc;
//                        sc3D_estimation.getResultDescriptors(sc);
//
//                        timersc3D.stopTimer();
//                        std::cout << "[INFO] Time : " << timersc3D.getTime() << std::endl;
//
//                        if (scale_descriptor){
//                            std::vector<float> data_tmp,value_descriptor_scaled;
//                            for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
//                            {
//                                data_tmp.push_back(sc.points[0].descriptor[i]);
//                            }
//                            value_descriptor_scaled = minMaxScaler(data_tmp);
//                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                                sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
//                            }
//                        }
//                        //Check NANValue
//                        bool containNANValues = false;
//                        for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
//                        {
//                            if (pcl_isnan(sc.points[0].descriptor[i])){
//                                containNANValues = true;
//                                //std::cout << "NAAAAAN VALUE" << std::endl;
//                            }
//                        }
//                        if (!containNANValues){
//
//                            std::string output = descriptor_folder_sc3D + name_descriptor;
//                            pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(output, sc);
//                        }else {
//                            continue;
//                        }
//
//                    }
//
//                    timer_alldescriptor.stopTimer();
//                    std::cout << "[INFO] Time processed for all descriptors : " << timer_alldescriptor.getTime() << std::endl;
//
//                }
//            }
//        }
//    }
//    //To comment for dataset with no views folder
//    //}
//
//
//
//    else {
//        //Show help
//        if (pcl::console::find_switch (argc, argv, "-h"))
//        {
//            showHelp (argv[0]);
//            exit (0);
//        }
//        //If not enough parameters
//        if (argc < 4)
//        {
//            std::cout << "[INFO] Not enough parameters " << std::endl;
//            showHelp (argv[0]);
//            return (-1);
//        }
//
//        std::string cloud_path;
//        if(pcl::console::parse_argument(argc, argv, "-cloud", cloud_path) == -1)
//        {
//            std::cerr<<"Please specify the cloud point (pcd or ply)"<<std::endl;
//            return -1;
//        }
//
//        std::string type_descriptor;
//        if(pcl::console::parse_argument(argc, argv, "-descriptor", type_descriptor) == -1)
//        {
//            std::cerr<<"Please specify the name of a descriptor "<<std::endl;
//            return -1;
//        }
//
//        if(pcl::console::parse_argument(argc, argv, "-output", output) == 1)
//        {
//            std::cout << "Output : " << output << std::endl;
//        }
//
//        std::string scale;
//        if(pcl::console::parse_argument(argc, argv, "-scale", scale) ==  - 1)
//        {
//            std::cerr<<"Please specify if you want to scale or not  "<<std::endl;
//            return -1;
//        }
//
//        std::cout << "Scale descriptor : " << scale_descriptor << std::endl;
//        if (scale == "true"){
//            scale_descriptor = true;
//        }else {
//            scale_descriptor = false;
//        }
//        std::cout << "Scale descriptor : " << scale_descriptor << std::endl;
//
//
//        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//
//        if (readPointCloud( cloud_path,  cloud)==-1)
//            return -1;
//
//        if (!checkValidityDescriptor(type_descriptor)){
//            return -1;
//        }
//        if (normalize_cloud){
//            std::cout << "Normalize point cloud " << std::endl;
//            normalizePointCloud(cloud);
//
//        }
//        /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = visualise_both_cloud(cloud,cloud);
//         //--------------------
//         // -----Main loop-----
//         //--------------------
//         while (!viewer->wasStopped ())
//         {
//         viewer->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//         }*/
//
//
//        std::cout << "Computing cloud using " << type_descriptor << std::endl;
//        if (type_descriptor == "esf"){
//
//            esf_estimation.setInputCluster(*cloud);
//            esf_estimation.estimate();
//            pcl::PointCloud<pcl::ESFSignature640> esfs;
//            esf_estimation.getResultDescriptors(esfs);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(esfs.points[0].histogram[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                }
//            }
//            pcl::io::savePCDFileASCII<pcl::ESFSignature640>(output, esfs);
//
//
//        }
//        else if (type_descriptor == "vfh"){
//            vfh_estimation.setInputCluster(*cloud);
//            vfh_estimation.estimate();
//            pcl::PointCloud<pcl::VFHSignature308> vfhs;
//            vfh_estimation.getResultDescriptors(vfhs);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(vfhs.points[0].histogram[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//
//            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, vfhs);
//
//
//        }
//        else if (type_descriptor == "cvfh"){
//            cvfh_estimation.setInputCluster(*cloud);
//            cvfh_estimation.estimate();
//            pcl::PointCloud<pcl::VFHSignature308> cvfhs;
//            cvfh_estimation.getResultDescriptors(cvfhs);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(cvfhs.points[0].histogram[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, cvfhs);
//
//
//        }
//        else if (type_descriptor == "ourcvfh"){
//            //ourcvfh
//            ourcvfh_estimation.setInputCluster(*cloud);
//            ourcvfh_estimation.estimate();
//            pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
//            ourcvfh_estimation.getResultDescriptors(ourcvfhs);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(output, ourcvfhs);
//
//        }
//        else if (type_descriptor == "grsd"){
//            grsd_estimation.setInputCluster(*cloud);
//            grsd_estimation.estimate();
//            pcl::PointCloud<pcl::GRSDSignature21> grsd;
//            grsd_estimation.getResultDescriptors(grsd);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i <pcl::GRSDSignature21::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(grsd.points[0].histogram[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//            pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(output, grsd);
//
//        }
//        else if (type_descriptor == "gshot"){
//            gshot_estimation.setInputCluster(*cloud);
//            gshot_estimation.estimate();
//            pcl::PointCloud<pcl::SHOT352> gshots;
//            gshot_estimation.getResultDescriptors(gshots);
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i <pcl::SHOT352::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(gshots.points[0].descriptor[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//            pcl::io::savePCDFileBinary<pcl::SHOT352>(output, gshots);
//
//
//        }
//        else if (type_descriptor == "good"){
//            good_estimation.setNumberOfBins(5);
//            good_estimation.setThreshold(0.0015);
//
//            // Provide the original point cloud
//            good_estimation.setInputCloud(cloud);
//
//            // Compute GOOD discriptor for the given object
//            std::vector< float > object_description;
//            good_estimation.compute(object_description);
//
//            if (scale_descriptor){
//                std::vector<float> value_descriptor_scaled;
//
//                value_descriptor_scaled = minMaxScaler(object_description);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    object_description.at(j) = value_descriptor_scaled.at(j);
//                }
//            }
//            ofstream descfile;
//            descfile.open (output,ios::out );
//            copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
//            descfile.close();
//
//
//        }
//        else if (type_descriptor == "usc"){
//            usc_estimation.setInputCloud(*cloud);
//            usc_estimation.estimate();
//            pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
//            usc = usc_estimation.getResultDescriptors();
//
//            pcl::io::savePCDFile(output, usc);
//
//
//        }
//        else if (type_descriptor == "sc3D"){
//            sc3D_estimation.setInputCluster(*cloud);
//            sc3D_estimation.estimate();
//            pcl::PointCloud<pcl::ShapeContext1980> sc;
//            sc3D_estimation.getResultDescriptors(sc);
//
//
//            if (scale_descriptor){
//                std::vector<float> data_tmp,value_descriptor_scaled;
//                for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
//                {
//                    data_tmp.push_back(sc.points[0].descriptor[i]);
//                }
//                value_descriptor_scaled = minMaxScaler(data_tmp);
//                for (int j = 0; j < value_descriptor_scaled.size(); j++){
//                    sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
//                }
//            }
//
//            pcl::io::savePCDFileBinary<pcl::ShapeContext1980>(output, sc);
//
//
//        }
//
//        std::cout << "\n Computing finished "<< std::endl;
//
//    }
//
//
//
//
//    return 0;
//
//
//}




