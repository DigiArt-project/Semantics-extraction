// STL
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <algorithm>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//Configuration
#include "Configuration/configuration.hpp"

//VTK
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkVersion.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkProgrammableSource.h>
#include <vtkContourFilter.h>
#include <vtkReverseSense.h>
#include <vtkDelaunay2D.h>
#include <vtkDelaunay3D.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkCleanPolyData.h>

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
#include "descriptor_estimation/volume_area_estimation.hpp"
#include "descriptor_estimation/good_estimation.cpp"

/// This program generates views from 3D point cloud and compute the associated descriptor

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<90>,
                                  (float[90], histogram, histogram)
                                  )
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<153>,
                                  (float[153], histogram, histogram)
                                  )

//Read point cloud from a path
int readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud)
{
    
    if (!boost::filesystem::exists (object_path)){
        std::cerr << "Error with pcd file - Check the path" << std::endl;
        return -1;
    }
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

std::string remove_extension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot);
}


bool containsNaNValues(std::vector<float> data){
    bool ok = true;
    for (int i = 0; i< data.size(); i++){
        if (pcl_isnan(data.at(i))){
            ok = false;
        }
    }
    return ok;
}
void normalizePC(typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    
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
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *cloud, transform);
    //pcl::io::savePLYFile( newFilePath, *transformed_cloud);
}

/*
void normalizePointCloud(typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    //Compute Centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    //Demean Point cloud
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_demean (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::demeanPointCloud(*cloud,centroid,*cloud_demean);
    //Normalize by its root mean square distance to the origin
    int numberPoints = cloud_demean->size();
    float value_scale = 1;
    
    float accumX = 0.;
    float accumY = 0.;
    float accumZ = 0.;
    for(typename pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_demean->begin(); it!= cloud_demean->end(); it++){
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
}

void normalizePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud){
    //Compute Centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);
    //Demean Point cloud
 pcl::PointCloud<pcl::PointXYZ> cloud_demean;
    pcl::demeanPointCloud(cloud,centroid,cloud_demean);
    //Normalize by its root mean square distance to the origin
    int numberPoints = cloud_demean.size();
    float value_scale = 1;
    
    float accumX = 0.;
    float accumY = 0.;
    float accumZ = 0.;
    for(typename pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_demean.begin(); it!= cloud_demean.end(); it++){
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
        float x = cloud_demean.points[i].x;
        float y = cloud_demean.points[i].y;
        float z = cloud_demean.points[i].z;
        
        cloud_demean.points[i].x = x * res * scale ;
        cloud_demean.points[i].y = y * res  * scale ;
        cloud_demean.points[i].z = z * res  * scale ;
        
    }
    cloud = cloud_demean;
    
}
 */

double
compute_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud){
#ifdef DEBUG
    std::cout << "Computing cloud resolution...\n";
#endif
    
    double resolution = 0.0;
    int points = 0;
    int nres;
    
    std::vector<int> indices(2);
    std::vector<float> sqrDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;
        
        nres = kdtree.nearestKSearch(i, 2, indices, sqrDistances);
        
        if (nres == 2)
        {
            resolution += sqrt(sqrDistances[1]);
            ++points;
        }
    }
    
    if (points != 0)
        resolution /= points;
    
    return resolution;
}

void save_all_descriptors(std::string path,std::vector<std::pair<std::string, std::vector<float>> > all_descriptors_pairs, bool to_scale){
    ofstream file_descriptor;
    file_descriptor.open(path);
    int count = 0;
    for (int i = 0; i < all_descriptors_pairs.size(); i++){
        
        std::pair<std::string, std::vector<float> > descriptor_i = all_descriptors_pairs.at(i);
        std::string name_descriptor_i = descriptor_i.first;
        std::vector<float> value_descriptor_i = descriptor_i.second;
        
        //auto max = std::max_element(std::begin(value_descriptor_i), std::end(value_descriptor_i));
        //auto min = std::min_element(std::begin(value_descriptor_i), std::end(value_descriptor_i));
        if (count == 0) {
            file_descriptor << name_descriptor_i << "\n";
        }else {
            file_descriptor << "\n" << name_descriptor_i << "\n";
        }
        count ++;
        if (to_scale){
            std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                file_descriptor << value_descriptor_scaled.at(j) << " ";
                //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
            }
        }else {
            for (int j = 0; j < value_descriptor_i.size(); j++){
                file_descriptor << value_descriptor_i.at(j) << " ";
                //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
            }
        }
    }
}

void concatenateDescriptors(std::string path,std::vector<std::pair<std::string, std::vector<float>> > all_descriptors_pairs, int choice, bool to_scale ){
    ofstream file_descriptor;
    file_descriptor.open(path);
    int count = 0;
    for (int i = 0; i < all_descriptors_pairs.size(); i++){
        std::pair<std::string, std::vector<float> > descriptor_i = all_descriptors_pairs.at(i);
        std::string name_descriptor_i = descriptor_i.first;
        std::vector<float> value_descriptor_i = descriptor_i.second;
        
        switch(choice) {
                //Concatenate ESF and VFH
            case 0 :
                if (name_descriptor_i.compare("esf") == 0 || name_descriptor_i.compare("vfh") == 0 ){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                break;
                
                //Concatenate ESF and GSHOT
            case 1 :
                if (name_descriptor_i.compare("esf") == 0 || name_descriptor_i.compare("gshot") == 0 ){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                
                break;
                
                //Concatenate ESF and GRSD
            case 2 :
                if (name_descriptor_i.compare("esf") == 0 || name_descriptor_i.compare("grsd") == 0 ){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                
                break;
                
                //Concatenate ESF, GRSD and GSHOT
            case 3 :
                if (name_descriptor_i.compare("esf") == 0 || name_descriptor_i.compare("grsd") == 0 || name_descriptor_i.compare("gshot") == 0){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                break;
                
                //Concatenate ESF, GRSD, GSHOT, VFH
            case 4 :
                if (name_descriptor_i.compare("esf") == 0 || name_descriptor_i.compare("grsd") == 0 || name_descriptor_i.compare("gshot") == 0
                    || name_descriptor_i.compare("vfh") == 0){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                
                break;
                
                //Concatenate GRSD and GSHOT
            case 5 :
                if (name_descriptor_i.compare("gshot") == 0 || name_descriptor_i.compare("grsd") == 0 ){
                    if (to_scale){
                        std::vector<float> value_descriptor_scaled = minMaxScaler(value_descriptor_i);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            file_descriptor << value_descriptor_scaled.at(j) << " ";
                        }
                        
                    }else {
                        for (int j = 0; j < value_descriptor_i.size(); j++){
                            file_descriptor << value_descriptor_i.at(j) << " ";
                            //file_descriptor << (value_descriptor_i.at(j) - *min)/(*max - *min) << " ";
                        }
                    }
                }
                
                break;
                
        }
        
        count ++;
        
        
        
    }
}
/*
void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Compute views and descriptors                   *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "This program take a list of ply objects point cloud, normalize them and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed. Each view can be saved and every descriptors are saved.\n" << std::endl;
    std::cout << "\n Assumption 1 : the folder which contains categories txt file is the dataset folder " << std::endl;
    std::cout << "\n Assumption 2 : the structure of the dataset follows this structure : Dataset/categorie_i/object_j for i,j in [0...N] \n" << std::endl;
    std::cout << "Usage: " << filename << " config (config file cfg format) Dataset_folder" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

//../../config.cfg ../../../../Datasets/views_PSB_reconf_test
int
main (int argc, char** argv)
{
    //If not enough parameters
    if (argc < 3)
    {
        showHelp (argv[0]);
        return (-1);
    }
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    
    std::string configuration_file = argv[1]; // Configuration file to take in account the parameters
    std::string dataset_folder = argv[2]; // Dataset to normalize and to compute views
    
    //std::string save_dir = argv[2]; // folder where to save the results
    Config cfg;
    std::vector<std::string> categories; // will contain the different categorie of objects
    std::vector<std::string> sub_categories; // will contain the different categorie of objects
    
    
    
    //Check if the dataset directory exist
    if (!boost::filesystem::exists(dataset_folder)){
        std::cout << "[INFO] The dataset does not exist :"<< dataset_folder << std::endl;
        return (-1);
    }

    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0){
        std::cout << "[ERROR] Error get configuration " << std::endl;
        return (-1);
    }
    
    int count_data;
    std::string extension_cloud = cfg.m_computeViewsDescriptors.extension_cloud_file;
    float leaf = cfg.m_computeViewsDescriptors.leaf_resolution;
    std::string current_category,current_subcategory = "";
   
    boost::filesystem::path base_dir(dataset_folder);
    
    //Choose the option for cacatenation of descriptors
    int choice_concatenation = cfg.m_computeViewsDescriptors.concatenation_descriptors  ;
    bool scale_descriptor = true;
    
    for(boost::filesystem::recursive_directory_iterator it(base_dir); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {
        
        
        std::string path_current = it->path().c_str();
        if (boost::filesystem::is_directory (it->path())){

            std::size_t found = path_current.find_last_of("/\\");
            std::string name_category = path_current.substr(found+1);
            if (std::string::npos != name_category.find_first_of("0123456789"))
            {
                sub_categories.push_back(name_category);
                current_subcategory = name_category;
            }
            else {
                //std::cout << name_category << std::endl;
                categories.push_back(name_category);
                current_category = name_category;
                std::cout << "[INFO] Category : " << current_category << std::endl;
                count_data = 0;
            }
        }
        
        if (it->path().extension().string() == extension_cloud && it->path().string().find("/views/") == std::string::npos)
        {
            std::string cloud_path = it->path().c_str();
            std::cout << "[INFO] Processing : " << cloud_path << std::endl;
            boost::filesystem::path p (cloud_path);
            std::string filename = remove_extension( p.filename().string());
            std::cout << "Object : " << filename << std::endl;
            
            
            pcl::PointCloud<pcl::PointXYZ> completeModel;
            VFHEstimation<pcl::PointXYZ> vfh_estimation;
            CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
            OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
            ESFEstimation<pcl::PointXYZ> esf_estimation;
            SPINEstimation<pcl::PointXYZ> spin_estimation;
            USCEstimation<pcl::PointXYZ> usc_estimation;
            SC3DEstimation<pcl::PointXYZ> sc3D_estimation;
            GRSDEstimation<pcl::PointXYZ> grsd_estimation;
            CRHEstimation<pcl::PointXYZ> crh_estimation;
            GSHOTEstimation<pcl::PointXYZ> gshot_estimation;
            GOODEstimation<pcl::PointXYZ> good_estimation;
            
            std::string category, subcategory;
            std::string descriptor_file;
            std::string descriptor_path;
            boost::system::error_code error;
            
            std::string save_dir = dataset_folder;
            category = current_category;
            subcategory = current_subcategory;
            
            std::cout << "CATEGORY  : " << category << std::endl;
            std::cout << "SUBCATEGORY  : " << subcategory << std::endl;
            std::string category_dir;
            if (!current_subcategory.empty()){
                category_dir = save_dir + "/" + category;
                boost::filesystem::create_directory(category_dir);
                category_dir = category_dir + "/" + subcategory;
                boost::filesystem::create_directory(category_dir);
            }else {
                category_dir = save_dir + "/" + category;
                std::cout << category_dir << std::endl;
                boost::filesystem::create_directory(category_dir);
            }
            
            std::string view_dir = category_dir + "/views";
            boost::filesystem::create_directory(view_dir);
            std::string desc_dir = category_dir + "/descriptors";
            boost::filesystem::create_directory(desc_dir);
            
            std::cout << "VIEW DIR " << view_dir << std::endl;
            std::cout << "Descriptors DIR " << desc_dir << std::endl;
            
            
            descriptor_file = boost::filesystem::path (cloud_path.c_str ()).stem ().string ();
            descriptor_path = descriptor_file + "/" + category;

            if (cfg.m_computeViewsDescriptors.enable_compute_views){
                
                //read in the ply model
                boost::shared_ptr<pcl::visualization::PCLVisualizer> renderer (new pcl::visualization::PCLVisualizer ("render"));
                //pcl::visualization::PCLVisualizer renderer("render");
                renderer->setBackgroundColor( 1, 1, 1 );
                
                vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
                std::string path_to_ply = cloud_path;
                std::cout << "Path to Ply : " << cloud_path << std::endl;

                //Normalize point cloud
                typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                if (readPointCloud( cloud_path,  cloud)==-1)
                    return -1;
                
                //normalizePointCloud(cloud);
                normalizePC(cloud);
                
                vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New ();
                
                //Load PLY model and scale it
                const float model_scale = cfg.m_computeViewsDescriptors.model_scale;
                
                readerQuery->SetFileName (path_to_ply.c_str());
                vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
                trans->Scale (model_scale, model_scale, model_scale);
                trans->Modified ();
                trans->Update ();
                
                vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
                filter_scale->SetTransform (trans);
                filter_scale->SetInputConnection (readerQuery->GetOutputPort ());
                //filter_scale->SetInputData(data);
                filter_scale->Update ();
                
                vtkSmartPointer<vtkPolyData> mapper = filter_scale->GetOutput ();
                
                //vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
                readerQuery->Update();
                
                //render the ply model from different view
                //add the model to PCLVisualizer
                renderer->addModelFromPolyData(mapper, "model", 0);
                //input parameter
                const int xres = cfg.m_computeViewsDescriptors.xres;
                const int yres = cfg.m_computeViewsDescriptors.yres;
                const int view_angle = cfg.m_computeViewsDescriptors.view_angle;
                const int radius_sphere = cfg.m_computeViewsDescriptors.radius_sphere;
                const float tesselation_level = cfg.m_computeViewsDescriptors.tesselation_level;
                
                //output parameter
                pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views;
                std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
                std::vector<float> enthropies;
                //do the rendering
                renderer->renderViewTesselatedSphere(xres, yres, views, poses, enthropies, tesselation_level, view_angle,radius_sphere);
                renderer->removeCorrespondences("model");
                renderer->close();
                
                
                //iterate through all the views
                std::cout << "[INFO] Number of views : " << views.size() << std::endl;
                if (views.size() == 0){
                    std::cerr << "There is no views - Aborting" << std::endl;
                    continue;
                }
               
                for(int i = 0; i < views.size(); ++i)
                    {
                    std::vector<std::pair<std::string, std::vector<float>> > all_descriptors_pairs;
                    std::pair<std::string, std::vector<float>> descriptor_pair;
                    //add the partial view to the complete model
                    Eigen::Matrix4f pose_inverse = Eigen::Matrix4f::Identity();
                    pose_inverse.block(0,0,3,3) = poses[i].block(0,0,3,3).transpose();
                    pose_inverse.block(0,3,3,1) = - poses[i].block(0,0,3,3).transpose() * poses[i].block(0,3,3,1);
                    pcl::PointCloud<pcl::PointXYZ> transformed_view;
                    pcl::transformPointCloud<pcl::PointXYZ>(views[i], transformed_view, pose_inverse);
                    completeModel += transformed_view;
                    
                    pcl::PointCloud<pcl::PointXYZ>::Ptr view_i_ptr(new pcl::PointCloud<pcl::PointXYZ>(views[i]));
                    normalizePC(view_i_ptr);

                    //Downsample cloud to given resolution
                    if (cfg.m_computeViewsDescriptors.enable_resolution){
                        std::cout << "INFO : using leaf resolution  : " << leaf << std::endl;
                        int size_original = view_i_ptr->size();
                        float resolution_original = compute_resolution(view_i_ptr);
                        //down sample the current view
                        pcl::VoxelGrid<pcl::PointXYZ> down;
                        down.setLeafSize (leaf, leaf, leaf);
                        down.setInputCloud (view_i_ptr);
                        down.filter (*view_i_ptr);
                        int size_after = view_i_ptr->size();
                        float resolution_after = compute_resolution(view_i_ptr);
    
                    }
                    if (cfg.m_computeViewsDescriptors.enable_vfh){
                        std::string vfh_dir = desc_dir + "/vfh";
                        boost::filesystem::create_directory(vfh_dir);
                        std::cout << "[INFO] Compute VFH " <<  std::endl;
                        //vfh
                        vfh_estimation.setInputCluster(*view_i_ptr);
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
                        
                        //std::string descriptor_name = vfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = vfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, vfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, vfhs);
                            
                        }
                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = vfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "vfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                        
                    }
                    if (cfg.m_computeViewsDescriptors.enable_cvfh){
                        std::string cvfh_dir = desc_dir + "/cvfh";
                        boost::filesystem::create_directory(cvfh_dir);
                        //std::cout << "[INFO] Compute CVFH " <<  std::endl;
                        
                        
                        //cvfh
                        cvfh_estimation.setInputCluster(*view_i_ptr);
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
                        
                        //std::string descriptor_name = cvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = cvfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }
                        
                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = cvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "cvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_ourcvfh){
                        std::string ourcvfh_dir = desc_dir + "/ourcvfh";
                        boost::filesystem::create_directory(ourcvfh_dir);
                        
                        std::cout << "[INFO] Compute OURCVFH " <<  std::endl;
                        
                        
                        //ourcvfh
                        ourcvfh_estimation.setInputCluster(*view_i_ptr);
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
                        
                        std::string descriptor_name = ourcvfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }
                        
                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = ourcvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "ourcvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                        
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_esf){
                        std::string esf_dir = desc_dir + "/esf";
                        boost::filesystem::create_directory(esf_dir);
                        std::cout << "[INFO] Compute ESF " <<  std::endl;
                        
                        //esf
                        esf_estimation.setInputCluster(*view_i_ptr);
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

                        std::string descriptor_name = esf_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name, esfs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name, esfs);
                        }
                        
                        descriptor_pair.second.resize (640);
                        for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = esfs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "esf";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_usc){
                        std::string usc_dir = desc_dir + "/usc";
                        boost::filesystem::create_directory(usc_dir);
                        
                        std::cout << "[INFO] Compute USC " <<  std::endl;
                        //usc
                        usc_estimation.setInputCloud(*view_i_ptr);
                        usc_estimation.estimate();
                        pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
                        usc = usc_estimation.getResultDescriptors();

                        std::string descriptor_name = usc_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        pcl::io::savePCDFile(descriptor_name, usc);
                        
                    }
                    if (cfg.m_computeViewsDescriptors.enable_sc){
                        std::string sc_dir = desc_dir + "/sc3D";
                        boost::filesystem::create_directory(sc_dir);
                        
                        std::cout << "[INFO] Compute SC " <<  std::endl;
                        //sc
                        sc3D_estimation.setInputCluster(*view_i_ptr);
                        sc3D_estimation.estimate();
                        pcl::PointCloud<pcl::ShapeContext1980> sc;
                        sc3D_estimation.getResultDescriptors(sc);
                        
                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
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
                        for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
                        {
                            if (pcl_isnan( sc.points[0].descriptor[i])){
                                containNANValues = true;
                            }
                        }
                        if (!containNANValues){
                            
                            //std::string descriptor_name = usc_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                            std::string descriptor_name = sc_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                            pcl::io::savePCDFile(descriptor_name, sc);
                            
                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::ShapeContext1980>(descriptor_name, sc);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(descriptor_name, sc);
                            }
                            
                            descriptor_pair.second.resize (1980);
                            
                            for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = sc.points[0].descriptor[i];
                            }
                            descriptor_pair.first = "sc";
                            all_descriptors_pairs.push_back(descriptor_pair);
                        }
                        
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_spin){
                        std::string spin_dir = desc_dir + "/spin";
                        boost::filesystem::create_directory(spin_dir);
                        
                        std::cout << "[INFO] Compute SPIN " <<  std::endl;
                        //spin
                        spin_estimation.setInputCloud(*view_i_ptr);
                        spin_estimation.estimate();
                        
                        pcl::PointCloud< pcl::Histogram<153> > spinImage;
                        spinImage = spin_estimation.getResultDescriptors();
                        
                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i < pcl::Histogram<153>::descriptorSize(); i++)
                            {
                                data_tmp.push_back(spinImage.points[0].histogram[i]);
                            }
                            value_descriptor_scaled = minMaxScaler(data_tmp);
                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                spinImage.points[0].histogram[j] = value_descriptor_scaled.at(j);
                            }
                        }
                        
                        
                        //Check NANValue
                        bool containNANValues = false;
                        for (size_t i = 0; i <pcl::Histogram<153>::descriptorSize(); i++)
                        {
                            if (pcl_isnan( spinImage.points[0].histogram[i])){
                                containNANValues = true;
                                //std::cout << "NAAAAAN VALUE" << std::endl;
                            }
                        }
                        
                        if (!containNANValues){
                            //std::string descriptor_name = spin_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                            std::string descriptor_name = spin_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::Histogram<153>>(descriptor_name, spinImage);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::Histogram<153>>(descriptor_name, spinImage);
                            }
                            descriptor_pair.second.resize (153);
                            for (size_t i = 0; i < pcl::Histogram<153>::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = spinImage.points[0].histogram[i];
                            }
                            descriptor_pair.first = "spin";
                            //all_descriptors_pairs.push_back(descriptor_pair);
                        }
                        
                        
                    }
                    if (cfg.m_computeViewsDescriptors.enable_grsd){
                        std::string grsd_dir = desc_dir + "/grsd";
                        boost::filesystem::create_directory(grsd_dir);
                        std::cout << "[INFO] Compute GRSD " <<  std::endl;
                        //grsd
                        grsd_estimation.setInputCluster(*view_i_ptr);
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
                        

                        std::string descriptor_name = grsd_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::GRSDSignature21>(descriptor_name, grsd);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(descriptor_name, grsd);
                        }
                        
                        descriptor_pair.second.resize (21);
                        for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = grsd.points[0].histogram[i];
                        }
                        descriptor_pair.first = "grsd";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_crh){
                        std::string crh_dir = desc_dir + "/crh";
                        boost::filesystem::create_directory(crh_dir);
                        
                        std::cout << "[INFO] Compute CRH " <<  std::endl;
                        //crh
                        crh_estimation.setInputCluster(*view_i_ptr);
                        crh_estimation.estimate();
                        pcl::PointCloud<pcl::Histogram<90>> crhs;
                        crh_estimation.getResultDescriptors(crhs);
                        
                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i <pcl::Histogram<90>::descriptorSize(); i++)
                            {
                                data_tmp.push_back(crhs.points[0].histogram[i]);
                            }
                            value_descriptor_scaled = minMaxScaler(data_tmp);
                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                crhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                            }
                        }

                        std::string descriptor_name = crh_dir + "/desc_" + filename + "_" + std::to_string(i) +".pcd";
                        
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::Histogram<90>>(descriptor_name, crhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::Histogram<90>>(descriptor_name, crhs);
                        }
                        
                        descriptor_pair.second.resize (90);
                        for (size_t i = 0; i < pcl::Histogram<90>::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = crhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "crh";
                        //all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    if (cfg.m_computeViewsDescriptors.enable_good){
                        std::string good_dir = desc_dir + "/good";
                        boost::filesystem::create_directory(good_dir);
                        
                        std::cout << "[INFO] Compute GOOD " <<  std::endl;
                        
                        good_estimation.setNumberOfBins(5);
                        good_estimation.setThreshold(0.0015);
                        
                        // Provide the original point cloud
                        
                        good_estimation.setInputCloud(view_i_ptr);
                        
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
                        
                        std::string descriptor_name = good_dir + "/desc_" + filename + ".txt";
                        ofstream descfile;
                        descfile.open (descriptor_name,ios::out );
                        
                        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
                        
                        descfile.close();
                    }
                    
                    if (cfg.m_computeViewsDescriptors.enable_gshot){
                        std::string gshot_dir = desc_dir + "/gshot";
                        boost::filesystem::create_directory(gshot_dir);
                        
                        std::cout << "[INFO] Compute GSHOT " <<  std::endl;
                        //gshot
                        gshot_estimation.setInputCluster(*view_i_ptr);
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

                            std::string descriptor_name = gshot_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                            
                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::SHOT352>(descriptor_name, gshots);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name, gshots);
                            }
                            
                            descriptor_pair.second.resize (352);
                            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = gshots.points[0].descriptor[i];
                            }
                            descriptor_pair.first = "gshot";
                            all_descriptors_pairs.push_back(descriptor_pair);
                        }
                        
                    }
                    
                    if (cfg.m_computeViewsDescriptors.save_view) {
                        std::cout << "Save views" << std::endl;
                        std::string view_name = view_dir + "/view_" + filename + "_" + std::to_string(i) + ".pcd";
                        std::string view_name_ply = view_dir + "/view_" + filename + "_" + std::to_string(i) + ".ply";
                        
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::PointXYZ>(view_name, *view_i_ptr);
                            pcl::io::savePLYFileBinary<pcl::PointXYZ>(view_name_ply, *view_i_ptr);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::PointXYZ>(view_name, *view_i_ptr);
                            pcl::io::savePLYFileASCII<pcl::PointXYZ>(view_name_ply, *view_i_ptr);
                        }
                        
                    }
                        
             
      
                }
       
                //save the complete model to a pcd file
                pcl::VoxelGrid<pcl::PointXYZ> down;
                
                down.setLeafSize(leaf, leaf, leaf);
                down.setInputCloud(completeModel.makeShared());
                down.filter(completeModel);
                //std::string complete_model_name = category_dir + "/complete_model_" + std::to_string(count_data)+ ".pcd";
                std::string complete_model_name = category_dir + "/" + filename + ".pcd";
                std::string complete_model_name_ply = category_dir + "/" + filename + ".ply";
                pcl::io::savePCDFileBinary<pcl::PointXYZ>(complete_model_name, completeModel);
                pcl::io::savePLYFileBinary<pcl::PointXYZ>(complete_model_name_ply, completeModel);
                
                VolumeEstimation<pcl::PointXYZ> volume_estimation;
                
                if (cfg.m_computeViewsDescriptors.enable_volume){
                    std::cout << "[INFO] Compute Volume " <<  std::endl;
                    //Compute octomap of the target cloud
                    
                    std::string volume_dir = desc_dir + "/volume";
                    boost::filesystem::create_directory(volume_dir);
                    std::string descriptor_name = volume_dir + "/descTree_" + filename + ".bt";

                    std::string descriptor_name_file = volume_dir + "/complete_model_vol_" + filename + ".txt";
                    volume_estimation.createOctoMap(completeModel, 0.01);
                    volume_estimation.saveTree(descriptor_name);
                    ofstream descfile;
                    descfile.open (descriptor_name_file);
                    descfile <<  volume_estimation.getVolumeBox() << "\n";
                    descfile <<  volume_estimation.getVolumeObject() <<  "\n";
                    descfile.close();

                }

                count_data++;
                
            }
            
        }
    }
}
*/

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Compute views and descriptors                   *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "This program take a list of ply objects point cloud, normalize them and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed. Each view can be saved and every descriptors are saved.\n" << std::endl;
    std::cout << "\n Assumption 1 : the folder which contains categories txt file is the dataset folder " << std::endl;
    std::cout << "\n Assumption 2 : the structure of the dataset follows this structure : Dataset/categorie_i/object_j for i,j in [0...N] \n" << std::endl;
    std::cout << "Usage: " << filename << " config (config file cfg format) folder_result (folder where to save the descriptors)" << std::endl << std::endl;

    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;

}


int
main (int argc, char** argv)
{
    //If not enough parameters
    if (argc < 3)
    {
        showHelp (argv[0]);
        return (-1);
    }
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }

    std::string configuration_file = argv[1]; // Configuration file to take in account the parameters
    std::string save_dir = argv[2]; // folder where to save the results
    Config cfg;
    std::vector<std::string> categories; // will contain the different categorie of objects
    std::vector<std::string> sub_categories; // will contain the different categorie of objects

    //Check if the save directory exist
    if (!boost::filesystem::exists(save_dir)){
        std::cout << "[INFO] The result directory does not exist - Going to create it "<< std::endl;
        boost::filesystem::create_directory(save_dir);
    }

    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0){
        std::cout << "[ERROR] Error get configuration " << std::endl;
        return (-1);
    }

    //Check if the dataset exists
    std::string path_dataset = cfg.m_computeViewsDescriptors.path_to_dataset;
    if (!boost::filesystem::exists(path_dataset)){
        std::cout << "[INFO] The dataset does not exist :"<< path_dataset << std::endl;
        return (-1);
    }
    int count_data;
    std::string extension_cloud = cfg.m_computeViewsDescriptors.extension_cloud_file;
    std::string current_category,current_subcategory = "";
    boost::filesystem::path base_dir(path_dataset);

    //Choose the option for cacatenation of descriptors
    int choice_concatenation = cfg.m_computeViewsDescriptors.concatenation_descriptors  ;
    bool scale_descriptor = true;
    std::cout << "Save view : " << cfg.m_computeViewsDescriptors.save_view << std::endl;
    std::cout << "Compute good : " << cfg.m_computeViewsDescriptors.enable_good << std::endl;
    std::cout << "Compute esf : " << cfg.m_computeViewsDescriptors.enable_esf << std::endl;

    for(boost::filesystem::recursive_directory_iterator it(base_dir); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {


        std::string path_current = it->path().c_str();
        if (boost::filesystem::is_directory (it->path())){
            //std::cout << "Directory : " << path_current << std::endl;

            std::size_t found = path_current.find_last_of("/\\");
            std::string name_category = path_current.substr(found+1);

            if (std::string::npos != name_category.find_first_of("0123456789"))
            {
                //std::cout << "digit(s)found!" << std::endl;

                sub_categories.push_back(name_category);
                current_subcategory = name_category;


            }
            else {
                //std::cout << name_category << std::endl;
                categories.push_back(name_category);
                current_category = name_category;
                std::cout << "Categorie : " << current_category << std::endl;
                count_data = 0;
            }
        }

        if (it->path().extension().string() == extension_cloud)
        {
            std::string cloud_path = it->path().c_str();
            std::cout << "[INFO] Processing : " << cloud_path << std::endl;
            boost::filesystem::path p (cloud_path);
            std::string filename = remove_extension( p.filename().string());
            std::cout << "Object : " << filename << std::endl;


            pcl::PointCloud<pcl::PointXYZ> completeModel;
            VFHEstimation<pcl::PointXYZ> vfh_estimation;
            CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
            OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
            ESFEstimation<pcl::PointXYZ> esf_estimation;
            SPINEstimation<pcl::PointXYZ> spin_estimation;
            USCEstimation<pcl::PointXYZ> usc_estimation;
            SC3DEstimation<pcl::PointXYZ> sc3D_estimation;
            GRSDEstimation<pcl::PointXYZ> grsd_estimation;
            CRHEstimation<pcl::PointXYZ> crh_estimation;
            GSHOTEstimation<pcl::PointXYZ> gshot_estimation;
            GOODEstimation<pcl::PointXYZ> good_estimation;


            float leaf = cfg.m_computeViewsDescriptors.leaf_resolution;

            std::string category, subcategory;
            std::string descriptor_file;
            std::string descriptor_path;
            boost::system::error_code error;

            category = current_category;
            subcategory = current_subcategory;
            std::string category_dir;
            if (!current_subcategory.empty()){
                category_dir = save_dir + "/" + category;
                boost::filesystem::create_directory(category_dir);
                category_dir = category_dir + "/" + subcategory;
                boost::filesystem::create_directory(category_dir);
            }else {
                category_dir = save_dir + "/" + category;
                std::cout << category_dir << std::endl;
                boost::filesystem::create_directory(category_dir);
            }

            std::string view_dir = category_dir + "/views";
            boost::filesystem::create_directory(view_dir);
            std::string desc_dir = category_dir + "/descriptors";
            boost::filesystem::create_directory(desc_dir);


            descriptor_file = boost::filesystem::path (cloud_path.c_str ()).stem ().string ();
            descriptor_path = descriptor_file + "/" + category;


            if (cfg.m_computeViewsDescriptors.enable_compute_views){

                //read in the ply model
                boost::shared_ptr<pcl::visualization::PCLVisualizer> renderer (new pcl::visualization::PCLVisualizer ("render"));
                //pcl::visualization::PCLVisualizer renderer("render");
                renderer->setBackgroundColor( 1, 1, 1 );

                vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
                std::string path_to_ply = cloud_path;
                std::cout << "Path to Ply : " << cloud_path << std::endl;

                //Normalize point cloud
                typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                if (readPointCloud( cloud_path,  cloud)==-1)
                    return -1;

                normalizePC(cloud);

                vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New ();

                //Load PLY model and scale it
                const float model_scale = cfg.m_computeViewsDescriptors.model_scale;

                readerQuery->SetFileName (path_to_ply.c_str());
                vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
                trans->Scale (model_scale, model_scale, model_scale);
                trans->Modified ();
                trans->Update ();

                vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
                filter_scale->SetTransform (trans);
                filter_scale->SetInputConnection (readerQuery->GetOutputPort ());
                //filter_scale->SetInputData(data);
                filter_scale->Update ();

                vtkSmartPointer<vtkPolyData> mapper = filter_scale->GetOutput ();

                //vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
                readerQuery->Update();

                //TODO Sauvegarder sous format PLY puis reouvrir PLY

                //render the ply model from different view
                //add the model to PCLVisualizer
                renderer->addModelFromPolyData(mapper, "model", 0);
                //input parameter
                const int xres = cfg.m_computeViewsDescriptors.xres;
                const int yres = cfg.m_computeViewsDescriptors.yres;
                const int view_angle = cfg.m_computeViewsDescriptors.view_angle;
                const int radius_sphere = cfg.m_computeViewsDescriptors.radius_sphere;
                const float tesselation_level = cfg.m_computeViewsDescriptors.tesselation_level;

                //output parameter
                pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views;
                std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
                std::vector<float> enthropies;
                //do the rendering
                renderer->renderViewTesselatedSphere(xres, yres, views, poses, enthropies, tesselation_level, view_angle,radius_sphere);
                renderer->removeCorrespondences("model");
                renderer->close();


                //iterate through all the views
                std::cout << "[INFO] Number of views : " << views.size() << std::endl;
                if (views.size() == 0){
                    std::cerr << "There is no views - Aborting" << std::endl;
                    continue;
                }


#if(_OPENMP)
#pragma omp parallel for
#endif
                for(int i = 0; i < views.size(); ++i)
                {
                    std::vector<std::pair<std::string, std::vector<float>> > all_descriptors_pairs;
                    std::pair<std::string, std::vector<float>> descriptor_pair;
                    //add the partial view to the complete model
                    Eigen::Matrix4f pose_inverse = Eigen::Matrix4f::Identity();
                    pose_inverse.block(0,0,3,3) = poses[i].block(0,0,3,3).transpose();
                    pose_inverse.block(0,3,3,1) = - poses[i].block(0,0,3,3).transpose() * poses[i].block(0,3,3,1);
                    pcl::PointCloud<pcl::PointXYZ> transformed_view;
                    pcl::transformPointCloud<pcl::PointXYZ>(views[i], transformed_view, pose_inverse);
                    completeModel += transformed_view;
 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr view_i_ptr(new pcl::PointCloud<pcl::PointXYZ>(views[i]));
                    normalizePC(view_i_ptr);

                    //normalizePointCloud(views[i]);

                    //save the view, pose, enthropy, and descriptor to the disk
                    if (cfg.m_computeViewsDescriptors.enable_resolution){
                        std::cout << "INFO : using leaf resolution  : " << leaf << std::endl;
                        int size_original = views[i].size();
                        float resolution_original = compute_resolution(views[i].makeShared());
                        //down sample the current view
                        pcl::VoxelGrid<pcl::PointXYZ> down;
                        down.setLeafSize (leaf, leaf, leaf);
                        down.setInputCloud (views[i].makeShared());
                        down.filter (views[i]);
                        int size_after = views[i].size();
                        float resolution_after = compute_resolution(views[i].makeShared());
                        std::cout << "Resolution before : " << resolution_original << "| Resolution after : " << resolution_after << std::endl;
                        std::cout << "Size before : " << size_original << "| Size after : " << size_after << std::endl;

                    }

                    //save the descriptor of each view
                    if (cfg.m_computeViewsDescriptors.enable_vfh){
                        std::string vfh_dir = desc_dir + "/vfh";
                        boost::filesystem::create_directory(vfh_dir);

                        std::cout << "[INFO] Compute VFH " <<  std::endl;


                        //vfh
                        vfh_estimation.setInputCluster(views[i]);
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

                        //std::string descriptor_name = vfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = vfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, vfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, vfhs);

                        }
                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = vfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "vfh";
                        all_descriptors_pairs.push_back(descriptor_pair);

                    }
                    if (cfg.m_computeViewsDescriptors.enable_cvfh){
                        std::string cvfh_dir = desc_dir + "/cvfh";
                        boost::filesystem::create_directory(cvfh_dir);
                        //std::cout << "[INFO] Compute CVFH " <<  std::endl;


                        //cvfh
                        cvfh_estimation.setInputCluster(views[i]);
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

                        //std::string descriptor_name = cvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = cvfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }

                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = cvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "cvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }

                    if (cfg.m_computeViewsDescriptors.enable_ourcvfh){
                        std::string ourcvfh_dir = desc_dir + "/ourcvfh";
                        boost::filesystem::create_directory(ourcvfh_dir);

                        std::cout << "[INFO] Compute OURCVFH " <<  std::endl;


                        //ourcvfh
                        ourcvfh_estimation.setInputCluster(views[i]);
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


                        //std::string descriptor_name = ourcvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = ourcvfh_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }

                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = ourcvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "ourcvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);

                    }

                    if (cfg.m_computeViewsDescriptors.enable_esf){
                        std::string esf_dir = desc_dir + "/esf";
                        boost::filesystem::create_directory(esf_dir);
                        std::cout << "[INFO] Compute ESF " <<  std::endl;

                        //esf
                        esf_estimation.setInputCluster(views[i]);
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


                        //std::string descriptor_name = esf_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = esf_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name, esfs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name, esfs);
                        }

                        descriptor_pair.second.resize (640);
                        for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = esfs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "esf";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }

                    if (cfg.m_computeViewsDescriptors.enable_usc){
                        std::string usc_dir = desc_dir + "/usc";
                        boost::filesystem::create_directory(usc_dir);

                        std::cout << "[INFO] Compute USC " <<  std::endl;
                        //usc
                        usc_estimation.setInputCloud(views[i]);
                        usc_estimation.estimate();
                        pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
                        usc = usc_estimation.getResultDescriptors();

                        //std::string descriptor_name = usc_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = usc_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        pcl::io::savePCDFile(descriptor_name, usc);

                    }
                    if (cfg.m_computeViewsDescriptors.enable_sc){
                        std::string sc_dir = desc_dir + "/sc3D";
                        boost::filesystem::create_directory(sc_dir);

                        std::cout << "[INFO] Compute SC " <<  std::endl;
                        //sc
                        sc3D_estimation.setInputCluster(views[i]);
                        sc3D_estimation.estimate();
                        pcl::PointCloud<pcl::ShapeContext1980> sc;
                        sc3D_estimation.getResultDescriptors(sc);

                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
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
                        for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
                        {
                            if (pcl_isnan( sc.points[0].descriptor[i])){
                                containNANValues = true;
                            }
                        }
                        if (!containNANValues){

                            //std::string descriptor_name = usc_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                            std::string descriptor_name = sc_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                            pcl::io::savePCDFile(descriptor_name, sc);

                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::ShapeContext1980>(descriptor_name, sc);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(descriptor_name, sc);
                            }

                            descriptor_pair.second.resize (1980);

                            for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = sc.points[0].descriptor[i];
                            }
                            descriptor_pair.first = "sc";
                            all_descriptors_pairs.push_back(descriptor_pair);
                        }

                    }

                    if (cfg.m_computeViewsDescriptors.enable_spin){
                        std::string spin_dir = desc_dir + "/spin";
                        boost::filesystem::create_directory(spin_dir);

                        std::cout << "[INFO] Compute SPIN " <<  std::endl;
                        //spin
                        spin_estimation.setInputCloud(views[i]);
                        spin_estimation.estimate();

                        pcl::PointCloud< pcl::Histogram<153> > spinImage;
                        spinImage = spin_estimation.getResultDescriptors();

                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i < pcl::Histogram<153>::descriptorSize(); i++)
                            {
                                data_tmp.push_back(spinImage.points[0].histogram[i]);
                            }
                            value_descriptor_scaled = minMaxScaler(data_tmp);
                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                spinImage.points[0].histogram[j] = value_descriptor_scaled.at(j);
                            }
                        }


                        //Check NANValue
                        bool containNANValues = false;
                        for (size_t i = 0; i <pcl::Histogram<153>::descriptorSize(); i++)
                        {
                            if (pcl_isnan( spinImage.points[0].histogram[i])){
                                containNANValues = true;
                                //std::cout << "NAAAAAN VALUE" << std::endl;
                            }
                        }

                        if (!containNANValues){
                            //std::string descriptor_name = spin_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                            std::string descriptor_name = spin_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::Histogram<153>>(descriptor_name, spinImage);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::Histogram<153>>(descriptor_name, spinImage);
                            }
                            descriptor_pair.second.resize (153);
                            for (size_t i = 0; i < pcl::Histogram<153>::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = spinImage.points[0].histogram[i];
                            }
                            descriptor_pair.first = "spin";
                            //all_descriptors_pairs.push_back(descriptor_pair);
                        }


                    }
                    if (cfg.m_computeViewsDescriptors.enable_grsd){
                        std::string grsd_dir = desc_dir + "/grsd";
                        boost::filesystem::create_directory(grsd_dir);
                        std::cout << "[INFO] Compute GRSD " <<  std::endl;
                        //grsd
                        grsd_estimation.setInputCluster(views[i]);
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

                        //std::string descriptor_name = grsd_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = grsd_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";
                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::GRSDSignature21>(descriptor_name, grsd);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(descriptor_name, grsd);
                        }

                        descriptor_pair.second.resize (21);
                        for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = grsd.points[0].histogram[i];
                        }
                        descriptor_pair.first = "grsd";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }

                    if (cfg.m_computeViewsDescriptors.enable_crh){
                        std::string crh_dir = desc_dir + "/crh";
                        boost::filesystem::create_directory(crh_dir);

                        std::cout << "[INFO] Compute CRH " <<  std::endl;
                        //crh
                        crh_estimation.setInputCluster(views[i]);
                        crh_estimation.estimate();
                        pcl::PointCloud<pcl::Histogram<90>> crhs;
                        crh_estimation.getResultDescriptors(crhs);

                        if (scale_descriptor){
                            std::vector<float> data_tmp,value_descriptor_scaled;
                            for (size_t i = 0; i <pcl::Histogram<90>::descriptorSize(); i++)
                            {
                                data_tmp.push_back(crhs.points[0].histogram[i]);
                            }
                            value_descriptor_scaled = minMaxScaler(data_tmp);
                            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                                crhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                            }
                        }


                        //std::string descriptor_name = crh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        std::string descriptor_name = crh_dir + "/desc_" + filename + "_" + std::to_string(i) +".pcd";

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::Histogram<90>>(descriptor_name, crhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::Histogram<90>>(descriptor_name, crhs);
                        }

                        descriptor_pair.second.resize (90);
                        for (size_t i = 0; i < pcl::Histogram<90>::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = crhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "crh";
                        //all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    if (cfg.m_computeViewsDescriptors.enable_good){
                        std::string good_dir = desc_dir + "/good";
                        boost::filesystem::create_directory(good_dir);

                        std::cout << "[INFO] Compute GOOD " <<  std::endl;

                        good_estimation.setNumberOfBins(5);
                        good_estimation.setThreshold(0.0015);

                        // Provide the original point cloud

                        good_estimation.setInputCloud(views[i].makeShared());

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

                        std::string descriptor_name = good_dir + "/desc_" + filename + ".txt";
                        ofstream descfile;
                        descfile.open (descriptor_name,ios::out );

                        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));

                        descfile.close();
                    }

                    if (cfg.m_computeViewsDescriptors.enable_gshot){
                        std::string gshot_dir = desc_dir + "/gshot";
                        boost::filesystem::create_directory(gshot_dir);

                        std::cout << "[INFO] Compute GSHOT " <<  std::endl;
                        //gshot
                        gshot_estimation.setInputCluster(views[i]);
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
                            //std::string descriptor_name = gshot_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                            std::string descriptor_name = gshot_dir + "/desc_" + filename + "_" + std::to_string(i) + ".pcd";

                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::SHOT352>(descriptor_name, gshots);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name, gshots);
                            }

                            descriptor_pair.second.resize (352);
                            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = gshots.points[0].descriptor[i];
                            }
                            descriptor_pair.first = "gshot";
                            all_descriptors_pairs.push_back(descriptor_pair);
                        }

                    }

                    if (cfg.m_computeViewsDescriptors.save_view) {
                        std::cout << "Save views" << std::endl;
                        //save the view to a pcd file
                        //std::string view_name = view_dir + "/view_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                        //std::string view_name_ply = view_dir + "/view_" +  std::to_string(count_data) + std::to_string(i) + ".ply";
                        std::string view_name = view_dir + "/view_" + filename + "_" + std::to_string(i) + ".pcd";
                        std::string view_name_ply = view_dir + "/view_" + filename + "_" + std::to_string(i) + ".ply";

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::PointXYZ>(view_name, views[i]);
                            pcl::io::savePLYFileBinary<pcl::PointXYZ>(view_name_ply, views[i]);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::PointXYZ>(view_name, views[i]);
                            pcl::io::savePLYFileASCII<pcl::PointXYZ>(view_name_ply, views[i]);
                        }

                    }



                }


                //save the complete model to a pcd file
                pcl::VoxelGrid<pcl::PointXYZ> down;

                down.setLeafSize(leaf, leaf, leaf);
                down.setInputCloud(completeModel.makeShared());
                down.filter(completeModel);
                //std::string complete_model_name = category_dir + "/complete_model_" + std::to_string(count_data)+ ".pcd";
                std::string complete_model_name = category_dir + "/" + filename + ".pcd";
                std::string complete_model_name_ply = category_dir + "/" + filename + ".ply";
                pcl::io::savePCDFileBinary<pcl::PointXYZ>(complete_model_name, completeModel);
                pcl::io::savePLYFileBinary<pcl::PointXYZ>(complete_model_name_ply, completeModel);

                VolumeEstimation<pcl::PointXYZ> volume_estimation;

                if (cfg.m_computeViewsDescriptors.enable_volume){
                    std::cout << "[INFO] Compute Volume " <<  std::endl;
                    //Compute octomap of the target cloud

                    std::string volume_dir = desc_dir + "/volume";
                    boost::filesystem::create_directory(volume_dir);
                    //std::string descriptor_name = volume_dir + "/descTree_" + std::to_string(count_data)+ ".bt";
                    std::string descriptor_name = volume_dir + "/descTree_" + filename + ".bt";
                    //std::string descriptor_name_file = volume_dir + "/complete_model_vol_" + std::to_string(count_data)+ ".txt";
                    std::string descriptor_name_file = volume_dir + "/complete_model_vol_" + filename + ".txt";
                    volume_estimation.createOctoMap(completeModel, 0.01);
                    volume_estimation.saveTree(descriptor_name);
                    ofstream descfile;
                    descfile.open (descriptor_name_file);
                    descfile <<  volume_estimation.getVolumeBox() << "\n";
                    descfile <<  volume_estimation.getVolumeObject() <<  "\n";
                    descfile.close();


                    std::cout << "--> Volume box Target: " << volume_estimation.getVolumeBox() << std::endl;
                    std::cout << "--> Volume object Target: " << volume_estimation.getVolumeObject() << std::endl;

                }


                count_data++;


            }
            else {
                std::cout << "[INFO] Compute global descriptor" << std::endl;
                std::vector<std::pair<std::string, std::vector<float>> > all_descriptors_pairs;
                std::pair<std::string, std::vector<float>> descriptor_pair;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

                if (pcl::io::loadPLYFile<pcl::PointXYZ> (cloud_path, *cloud) == -1) //* load the file
                {
                    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                    return (-1);
                }

                float leaf = cfg.m_computeViewsDescriptors.leaf_resolution;
                //save the view, pose, enthropy, and descriptor to the disk
                if (cfg.m_computeViewsDescriptors.enable_resolution){
                    //down sample the current view
                    pcl::VoxelGrid<pcl::PointXYZ> down;
                    down.setLeafSize (leaf, leaf, leaf);
                    down.setInputCloud (cloud);
                    down.filter (*cloud);

                }
                if (cfg.m_computeViewsDescriptors.enable_grsd){
                    std::string grsd_dir = desc_dir + "/grsdfull";
                    boost::filesystem::create_directory(grsd_dir);
                    std::string descriptor_name = grsd_dir + "/desc_complete_" + filename + ".pcd";
                    if (!boost::filesystem::exists (descriptor_name)){

                    //std::cout << "[INFO] Compute GRSD " <<  std::endl;
                    //grsd
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


                    if (cfg.m_computeViewsDescriptors.save_binary){
                        pcl::io::savePCDFileBinary<pcl::GRSDSignature21>(descriptor_name, grsd);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(descriptor_name, grsd);
                    }
                    }
                }
                if (cfg.m_computeViewsDescriptors.enable_esf){
                    std::string esf_dir = desc_dir + "/esffull";
                    boost::filesystem::create_directory(esf_dir);
                    std::string descriptor_name = esf_dir + "/desc_complete_" + filename + ".pcd";

                    if (!boost::filesystem::exists (descriptor_name)){
                    //std::cout << "[INFO] Compute ESF " <<  std::endl;

                    //esf
                    esf_estimation.setInputCluster(*cloud);
                    esf_estimation.estimate();
                    pcl::PointCloud<pcl::ESFSignature640> esfs;
                    esf_estimation.getResultDescriptors(esfs);

                    if (scale_descriptor){
                        std::vector<float> data_tmp,value_descriptor_scaled;
                        for (size_t i = 0; i <pcl::ESFSignature640::descriptorSize(); i++)
                        {
                            data_tmp.push_back(esfs.points[0].histogram[i]);
                        }
                        value_descriptor_scaled = minMaxScaler(data_tmp);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                        }
                    }

                    if (cfg.m_computeViewsDescriptors.save_binary){
                        pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name, esfs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name, esfs);
                    }
                    }

                }

                if (cfg.m_computeViewsDescriptors.enable_vfh){
                    std::string vfh_dir = desc_dir + "/vfhfull";
                    boost::filesystem::create_directory(vfh_dir);
                     std::string descriptor_name = vfh_dir + "/desc_complete_" + filename + ".pcd";
                     if (!boost::filesystem::exists (descriptor_name)){

                    //std::cout << "[INFO] Compute GSHOT " <<  std::endl;
                    //gshot
                    vfh_estimation.setInputCluster(*cloud);
                    vfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> vfhs;
                    vfh_estimation.getResultDescriptors(vfhs);

                    if (scale_descriptor){
                        std::vector<float> data_tmp,value_descriptor_scaled;
                        for (size_t i = 0; i <pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            data_tmp.push_back(vfhs.points[0].histogram[i]);
                        }
                        value_descriptor_scaled = minMaxScaler(data_tmp);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                        }
                    }
                    //Check NANValue
                    bool containNANValues = false;
                    for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                    {
                        if (pcl_isnan(vfhs.points[0].histogram[i])){
                            containNANValues = true;
                            //std::cout << "NAAAAAN VALUE" << std::endl;
                        }
                    }
                    if (!containNANValues){
                        //std::string descriptor_name = gshot_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";


                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, vfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, vfhs);
                        }

                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = vfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "vfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                     }
                }
                if (cfg.m_computeViewsDescriptors.enable_cvfh){
                    std::string cvfh_dir = desc_dir + "/cvfhfull";
                    boost::filesystem::create_directory(cvfh_dir);
                    std::string descriptor_name = cvfh_dir + "/desc_complete_" + filename + ".pcd";

                     if (!boost::filesystem::exists (descriptor_name)){
                    cvfh_estimation.setInputCluster(*cloud);
                    cvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> cvfhs;
                    cvfh_estimation.getResultDescriptors(cvfhs);

                    if (scale_descriptor){
                        std::vector<float> data_tmp,value_descriptor_scaled;
                        for (size_t i = 0; i <pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            data_tmp.push_back(cvfhs.points[0].histogram[i]);
                        }
                        value_descriptor_scaled = minMaxScaler(data_tmp);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                        }
                    }
                    //Check NANValue
                    bool containNANValues = false;
                    for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                    {
                        if (pcl_isnan(cvfhs.points[0].histogram[i])){
                            containNANValues = true;
                        }
                    }
                    if (!containNANValues){

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, cvfhs);
                        }

                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = cvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "cvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                     }
                }
                if (cfg.m_computeViewsDescriptors.enable_ourcvfh){
                    std::string ourcvfh_dir = desc_dir + "/ourcvfhfull";
                    boost::filesystem::create_directory(ourcvfh_dir);
                    std::string descriptor_name = ourcvfh_dir + "/desc_complete_" + filename + ".pcd";
                     if (!boost::filesystem::exists (descriptor_name)){
                    ourcvfh_estimation.setInputCluster(*cloud);
                    ourcvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
                    ourcvfh_estimation.getResultDescriptors(ourcvfhs);

                    if (scale_descriptor){
                        std::vector<float> data_tmp,value_descriptor_scaled;
                        for (size_t i = 0; i <pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
                        }
                        value_descriptor_scaled = minMaxScaler(data_tmp);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                        }
                    }
                    //Check NANValue
                    bool containNANValues = false;
                    for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                    {
                        if (pcl_isnan(ourcvfhs.points[0].histogram[i])){
                            containNANValues = true;
                        }
                    }
                    if (!containNANValues){


                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                        }

                        descriptor_pair.second.resize (308);
                        for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = ourcvfhs.points[0].histogram[i];
                        }
                        descriptor_pair.first = "ourcvfh";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                }
                }


                if (cfg.m_computeViewsDescriptors.enable_good){
                    std::string good_dir = desc_dir + "/goodfull";
                    boost::filesystem::create_directory(good_dir);
                    std::string descriptor_name = good_dir + "/desc_complete" + filename + ".txt";
                    if (!boost::filesystem::exists (descriptor_name)){
                    //std::cout << "[INFO] Compute GOOD " <<  std::endl;

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



                    std::string descriptor_name = good_dir + "/desc_complete" + filename + ".txt";
                    ofstream descfile;
                    descfile.open (descriptor_name,ios::out );
                    copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
                    descfile.close();

                }
                }

                if (cfg.m_computeViewsDescriptors.enable_sc){
                    std::string sc_dir = desc_dir + "/sc3Dfull";
                    boost::filesystem::create_directory(sc_dir);
                    std::string descriptor_name = sc_dir + "/desc_complete_" + filename + ".pcd";
                     if (!boost::filesystem::exists (descriptor_name)){
                    //std::cout << "[INFO] Compute SC " <<  std::endl;
                    //sc
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

                    //Check NANValue
                    bool containNANValues = false;
                    for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
                    {
                        if (pcl_isnan( sc.points[0].descriptor[i])){
                            containNANValues = true;
                        }
                    }
                    if (!containNANValues){
                        //std::string descriptor_name = usc_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";

                        pcl::io::savePCDFile(descriptor_name, sc);

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::ShapeContext1980>(descriptor_name, sc);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(descriptor_name, sc);
                        }

                        descriptor_pair.second.resize (1980);

                        for (size_t i = 0; i < pcl::ShapeContext1980::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = sc.points[0].descriptor[i];
                        }
                        descriptor_pair.first = "sc";
                        all_descriptors_pairs.push_back(descriptor_pair);
                    }
                }
                }
                if (cfg.m_computeViewsDescriptors.enable_gshot){
                    std::string gshot_dir = desc_dir + "/gshotfull";
                    boost::filesystem::create_directory(gshot_dir);
                    std::string descriptor_name = gshot_dir + "/desc_complete_" + filename + ".pcd";
                    if (!boost::filesystem::exists (descriptor_name)){
                        std::cout << "[INFO] Compute GSHOT Full " <<  std::endl;
                        //sc
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

                        //Check NANValue
                        bool containNANValues = false;
                        for (size_t i = 0; i <pcl::SHOT352::descriptorSize(); i++)
                        {
                            if (pcl_isnan( gshots.points[0].descriptor[i])){
                                containNANValues = true;
                            }
                        }
                        if (!containNANValues){
                            //std::string descriptor_name = usc_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";

                            pcl::io::savePCDFile(descriptor_name, gshots);

                            if (cfg.m_computeViewsDescriptors.save_binary){
                                pcl::io::savePCDFileBinary<pcl::SHOT352>(descriptor_name, gshots);
                            }else {
                                pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name, gshots);
                            }

                            descriptor_pair.second.resize (352);

                            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
                            {
                                descriptor_pair.second[i] = gshots.points[0].descriptor[i];
                            }
                            descriptor_pair.first = "gshot";
                            all_descriptors_pairs.push_back(descriptor_pair);
                        }
                    }
                }

                if (cfg.m_computeViewsDescriptors.enable_spin){
                    std::string spin_dir = desc_dir + "/spinfull";
                    boost::filesystem::create_directory(spin_dir);
                    std::string descriptor_name = spin_dir + "/desc_complete_" + filename + ".pcd";
                    if (!boost::filesystem::exists (descriptor_name)){
                    //std::cout << "[INFO] Compute SPIN " <<  std::endl;
                    //spin
                    spin_estimation.setInputCloud(*cloud);
                    spin_estimation.estimate();

                    pcl::PointCloud< pcl::Histogram<153> > spinImage;
                    spinImage = spin_estimation.getResultDescriptors();

                    if (scale_descriptor){
                        std::vector<float> data_tmp,value_descriptor_scaled;
                        for (size_t i = 0; i <pcl::Histogram<153>::descriptorSize(); i++)
                        {
                            data_tmp.push_back(spinImage.points[0].histogram[i]);
                        }
                        value_descriptor_scaled = minMaxScaler(data_tmp);
                        for (int j = 0; j < value_descriptor_scaled.size(); j++){
                            spinImage.points[0].histogram[j] = value_descriptor_scaled.at(j);
                        }
                    }
                    //Check NANValue
                    bool containNANValues = false;
                    for (size_t i = 0; i <pcl::Histogram<153>::descriptorSize(); i++)
                    {
                        if (pcl_isnan( spinImage.points[0].histogram[i])){
                            containNANValues = true;
                            //std::cout << "NAAAAAN VALUE" << std::endl;
                        }
                    }
                    if (!containNANValues){
                        //std::string descriptor_name = spin_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";

                        if (cfg.m_computeViewsDescriptors.save_binary){
                            pcl::io::savePCDFileBinary<pcl::Histogram<153>>(descriptor_name, spinImage);
                        }else {
                            pcl::io::savePCDFileASCII<pcl::Histogram<153>>(descriptor_name, spinImage);
                        }
                        descriptor_pair.second.resize (153);
                        for (size_t i = 0; i < pcl::Histogram<153>::descriptorSize(); i++)
                        {
                            descriptor_pair.second[i] = spinImage.points[0].histogram[i];
                        }
                        descriptor_pair.first = "spin";
                        //all_descriptors_pairs.push_back(descriptor_pair);
                    }
                    }
                }
                count_data++;
            }

        }
    }
}


