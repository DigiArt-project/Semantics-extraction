#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include "others/utils.hpp"
#include "others/Timer.hpp"
#include "configuration/configuration.hpp"
#include "recognition_database/hypothesis.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "descriptor_estimation/esf_estimation.h"
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

/**
 
 Ce programme permet d'entraîner une database d'objet 3D. L'ensemble des objets 3D doivent être entier (par exemple la base RGBD dataset non). Ici, le programme fonctionne pour la base de donnée du structure sensor. La database a la structure suivante :
 Dataset/category_i/object_j pour i de 1 à N avec N le nombre de catégories et j de 1 à N avec m le nombre d'objet de cette catégorie
 ce programme va itérer dans le dataset et pour chaque objet de chaque catégorie, une caméra virtuelle va être créer pour créer plusieurs views de l'objet. Ensuite pour chaque view de l'objet crée, un descriptor global va être calculé (ESF,VFH,CVFH ou OURCVFH). A la fin, un index FLANN est créer pour accelerer le nearest neighbor
 
 **/

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Compute descriptors                              *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "This program take a list of ply object point cloud and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed (VFH, CVFH, ESF or/and OURCVFH). Each view can be saved and every descriptors are saved." << std::endl;
    std::cout << "\n Assumption 1 : the folder which contains categories txt file is the dataset folder " << std::endl;
    std::cout << "\n Assumption 2 : the structure of the dataset follows this structure : Dataset/categorie_i/object_j for i,j in [0...N] \n" << std::endl;
    std::cout << "Usage: " << filename << " config (config file cfg format) category (category file which contains the path to the dataset. One line per data) folder_result (folder where to save the descriptors)" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

int main(int argc, char** argv)
{
    //If not enough parameters
    if (argc < 4)
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
    std::string category_file = argv[2]; // txt file which contains the path to the data
    std::string save_dir = argv[3]; // folder where to save the results
    std::vector<std::string> categories; // will contain the different categorie of objects
    Config cfg;
    
    //Check if the save directory exist
    if (!boost::filesystem::exists(save_dir)){
        std::cout << "[INFO] The result directory does not exist - Going to create it "<< std::endl;
        boost::filesystem::create_directory(save_dir);
    }
    
    //check if the category file is a file
    if (!boost::filesystem::is_regular_file(category_file)){
        std::cout << "[ERROR] Category txt file is not regular file : " << category_file <<  std::endl;
        return (-1);
    }
    
    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0){
         std::cout << "[ERROR] Error get configuration " << std::endl;
        return (-1);
    }
    
    //Get the category
    if (Utils::getCategories (category_file, categories) < 0){
        std::cout << "[ERROR] Error get categories " << std::endl;
        return (-1);
    }
    if ( categories.size () == 0){
        std::cout << "[ERROR] Zero categories available " << std::endl;
        return (-1);
    }
    
    //Get the parent name of the category file for the path of the dataset
    boost::filesystem::path p(category_file);
    boost::filesystem::path parent_dir = p.parent_path();
    std::string dataset_dir = parent_dir.c_str();
    char lastChar = dataset_dir.at( dataset_dir.length() - 1 );
    if (lastChar != '/'){
        dataset_dir = dataset_dir + "/";
    }
    std::vector<std::string> point_clouds;
    for (size_t i = 0; i < categories.size (); ++i)
    {
        std::cout << "Categorie : " << categories.at(i) << std::endl;
        point_clouds.clear();
        if (Utils::getDataFilename (category_file,categories.at(i), point_clouds) < 0)
            continue;
        int count_data = 0;
        for (size_t ii = 0; ii < point_clouds.size (); ++ii)
        {
            
            std::cout << "Data : " << point_clouds.at(ii) << std::endl;
            std::string point_cloud = point_clouds.at(ii);
        
            std::cout << "Processing: " << point_cloud.c_str () << std::endl;
            std::string path_to_cloud = dataset_dir  + point_cloud;

            //read in the ply model
            boost::shared_ptr<pcl::visualization::PCLVisualizer> renderer (new pcl::visualization::PCLVisualizer ("render"));
            //pcl::visualization::PCLVisualizer renderer("render");
            renderer->setBackgroundColor( 1, 1, 1 );
            
            vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
            std::string path_to_ply = dataset_dir +point_cloud;
            
            std::cout << "[INFO] Path to PLY cloud data : " << path_to_ply << std::endl;
            boost::filesystem::path model_path(path_to_ply);
            readerQuery->SetFileName (model_path.c_str());
            vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
            readerQuery->Update();
           
            
            //render the ply model from different view
            //add the model to PCLVisualizer
            renderer->addModelFromPolyData(polydata, "model", 0);
            std::cout << "[INFO] OK " << std::endl;
            //input parameter
            const int xres = cfg.similaritySearch.rendering.xres;
            const int yres = cfg.similaritySearch.rendering.yres;
            const int view_angle = cfg.similaritySearch.rendering.view_angle;
            const float tesselation_level = cfg.similaritySearch.rendering.tesselation_level;
            
            //output parameter
            pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views;
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
            std::vector<float> enthropies;
            //do the rendering
            renderer->renderViewTesselatedSphere(xres, yres, views, poses, enthropies, tesselation_level, view_angle);
            renderer->removeCorrespondences("model");
            renderer->close();
            
            //renderer->reset();
            
            
            pcl::PointCloud<pcl::PointXYZ> completeModel;
            VFHEstimation<pcl::PointXYZ> vfh_estimation;
            CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
            OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
            ESFEstimation<pcl::PointXYZ> esf_estimation;
            
           
            float leaf = cfg.filtering.leaf_resolution;
            
            std::string category;
            std::string descriptor_file;
            std::string descriptor_path;
            boost::system::error_code error;
            
            category = categories.at(i);
            //Create necessary directories
            std::string category_dir = save_dir + "/" + category;
            boost::filesystem::create_directory(category_dir);
            std::string view_dir = category_dir + "/views";
            boost::filesystem::create_directory(view_dir);
            std::string desc_dir = category_dir + "/descriptors";
            boost::filesystem::create_directory(desc_dir);
            std::string esf_dir = desc_dir + "/esf";
            boost::filesystem::create_directory(esf_dir);
            std::string vfh_dir = desc_dir + "/vfh";
            boost::filesystem::create_directory(vfh_dir);
            std::string cvfh_dir = desc_dir + "/cvfh";
            boost::filesystem::create_directory(cvfh_dir);
            std::string ourcvfh_dir = desc_dir + "/ourcvfh";
            boost::filesystem::create_directory(ourcvfh_dir);
            
            
            descriptor_file = boost::filesystem::path (point_cloud.c_str ()).stem ().string ();
            descriptor_path = descriptor_file + "/" + category;
            
            std::cout << "[INFO] Category path : " << category << std::endl;
            std::cout << "[INFO] Descriptor File path: " << descriptor_file << std::endl;
            std::cout << "[INFO] descriptor path : " << descriptor_path << std::endl;
            
           
            
            //iterate through all the views
            std::cout << "[INFO] Number of views : " << views.size() << std::endl;
            for(int i = 0; i < views.size(); ++i)
            {
                //add the partial view to the complete model
                Eigen::Matrix4f pose_inverse = Eigen::Matrix4f::Identity();
                pose_inverse.block(0,0,3,3) = poses[i].block(0,0,3,3).transpose();
                pose_inverse.block(0,3,3,1) = - poses[i].block(0,0,3,3).transpose() * poses[i].block(0,3,3,1);
                pcl::PointCloud<pcl::PointXYZ> transformed_view;
                pcl::transformPointCloud<pcl::PointXYZ>(views[i], transformed_view, pose_inverse);
                completeModel += transformed_view;
                
                
                /*save the view, pose, enthropy, and descriptor to the disk*/
                if (cfg.similaritySearch.enable_resolution){
                    //down sample the current view
                    pcl::VoxelGrid<pcl::PointXYZ> down;
                    down.setLeafSize (leaf, leaf, leaf);
                    down.setInputCloud (views[i].makeShared());
                    down.filter (views[i]);

                }
                
                
                if (cfg.similaritySearch.rendering.save_view) {
                    //save the view to a pcd file
                    std::string view_name = view_dir + "/view_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                    std::string view_name_ply = view_dir + "/view_" + std::to_string(count_data) + std::to_string(i) + ".ply";
                    
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::PointXYZ>(view_name, views[i]);
                        pcl::io::savePLYFileBinary<pcl::PointXYZ>(view_name_ply, views[i]);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::PointXYZ>(view_name, views[i]);
                        pcl::io::savePLYFileASCII<pcl::PointXYZ>(view_name_ply, views[i]);
                    }
                    
                    std::cout << "Save view to : " << view_name << std::endl;
                }
                
                
                //save the descriptor of each view
                if (cfg.descriptor.enable_vfh){
                    //vfh
                    vfh_estimation.setInputCluster(views[i]);
                    vfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> vfhs;
                    vfh_estimation.getResultDescriptors(vfhs);
                    
                    std::string descriptor_name = vfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, vfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, vfhs);
                    }
                }
                if (cfg.descriptor.enable_cvfh){
                    //cvfh
                    cvfh_estimation.setInputCluster(views[i]);
                    cvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> cvfhs;
                    cvfh_estimation.getResultDescriptors(cvfhs);
                    
                    
                    std::string descriptor_name = cvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, cvfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, cvfhs);
                    }
                }
                
                if (cfg.descriptor.enable_ourcvfh){
                    //ourcvfh
                    ourcvfh_estimation.setInputCluster(views[i]);
                    ourcvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
                    ourcvfh_estimation.getResultDescriptors(ourcvfhs);
                    
                    std::string descriptor_name = ourcvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                   
                    if (cfg.rendering.save_binary){
                         pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                    }
                    
                }
                
                if (cfg.descriptor.enable_esf){
                    //esf
                    esf_estimation.setInputCluster(views[i]);
                    esf_estimation.estimate();
                    pcl::PointCloud<pcl::ESFSignature640> esfs;
                    esf_estimation.getResultDescriptors(esfs);
                    
                    std::string descriptor_name = esf_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                    if (cfg.rendering.save_binary){
                         pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name, esfs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name, esfs);
                    }
                }
            }
            
            //save the complete model to a pcd file
            pcl::VoxelGrid<pcl::PointXYZ> down;
            down.setLeafSize(leaf, leaf, leaf);
            down.setInputCloud(completeModel.makeShared());
            down.filter(completeModel);
            std::string complete_model_name = category_dir + "/complete_model_" + std::to_string(count_data)+ ".pcd";
            pcl::io::savePCDFileBinary<pcl::PointXYZ>(complete_model_name, completeModel);
            
            count_data++;
            
        }
    }
    return 0;
}
