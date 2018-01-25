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

#include <omp.h>

/**
 
 Ce programme calcule les descriptors d'une base de données d'objet 3D. En l'occurence, ici c'est le RGBD Dataset.
 Cependant, si la database suit la même structure, il est possible d'utiliser ce programme sur cette dernière.
 Voir le fichier config.cfg pour changer les parametres.
 Choisir un descripteur à la fois dans le config. Il ne faut pas enabler tous les descripteurs (enable_vfh = true). Un à la fois.
 
 **/


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Compute descriptor for RGBD Dataset             *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "This program take a list of pcd object point cloud and global descriptors are computed (VFH, CVFH, ESF or/and OURCVFH). Each descriptors is saved." << std::endl;
    std::cout << "\n Assumption 1 : the folder which contains the Categories txt file is the dataset " << std::endl;
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
        std::cerr << "[ERROR] The result directory does not exist - Please create one "<< std::endl;
        exit(0);
        
    }
    
    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0)
        return (-1);
    //Get the category
    if (Utils::getCategories (category_file, categories) < 0)
        return (-1);
    
    //Get the parent name of the category file for the path of the dataset
    boost::filesystem::path p(category_file);
    boost::filesystem::path parent_dir = p.parent_path();
    std::string dataset_dir = parent_dir.c_str();
    char lastChar = dataset_dir.at( dataset_dir.length() - 1 );
    if (lastChar != '/'){
        dataset_dir = dataset_dir + "/";
    }
    

    
    bool resolution_activated = cfg.similaritySearch.enable_resolution;

    

   /* #if(_OPENMP)
    omp_set_nested (1);
    #pragma omp parallel for
    #endif*/
    for (size_t i = 0; i < categories.size (); ++i)
    {
        std::cout << "Categorie : " << categories.at(i) << std::endl;
        std::vector<std::string> point_clouds;
        if (Utils::getDataFilename (category_file,categories.at(i), point_clouds) < 0)
            continue;
        int count_data = 0;
       /* #if(_OPENMP)
        #pragma omp parallel for
        #endif*/
        for (size_t ii = 0; ii < point_clouds.size (); ++ii)
        {
            std::cout << "n-° : " << ii << "/ " <<  point_clouds.size () << std::endl;
            std::cout << "Data : " << point_clouds.at(ii) << std::endl;
            std::string point_cloud = point_clouds.at(ii);
            
            
            std::cout << "Processing: " << point_cloud.c_str () << std::endl;
            std::string path_to_cloud = dataset_dir + point_cloud;
            
            pcl::PointCloud<pcl::PointXYZ> completeModel;
            VFHEstimation<pcl::PointXYZ> vfh_estimation;
            CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
            OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
            ESFEstimation<pcl::PointXYZ> esf_estimation;
            

            float leaf_resolution = cfg.filtering.leaf_resolution;
            
            std::string category;
            std::string descriptor_file;
            std::string descriptor_path;
            boost::system::error_code error;
            
            category = categories.at(i);
            //Create necessary directories
            std::string category_dir = save_dir + "/" + category;
            boost::filesystem::create_directory(category_dir);
            std::string esf_dir = category_dir + "/esf";
            boost::filesystem::create_directory(esf_dir);
            std::string vfh_dir = category_dir + "/vfh";
            boost::filesystem::create_directory(vfh_dir);
            std::string cvfh_dir = category_dir + "/cvfh";
            boost::filesystem::create_directory(cvfh_dir);
            std::string ourcvfh_dir = category_dir + "/ourcvfh";
            boost::filesystem::create_directory(ourcvfh_dir);
            
            
            descriptor_file = boost::filesystem::path (point_cloud.c_str ()).stem ().string ();
            descriptor_path = descriptor_file + "/" + category;
            
            std::cout << "[INFO] Category path : " << category << std::endl;
            //std::cout << "[INFO] Descriptor File path: " << descriptor_file << std::endl;
            //std::cout << "[INFO] descriptor path : " << descriptor_path << std::endl;
            
            //Load the pcd point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_cloud.c_str (), *cloud) < 0)
            {
                printf ("Could not open file '%s'! Error : %s\n",  path_to_cloud.c_str (), strerror (errno));
                continue;
            }
            
            //Downsampling for the resolution (for training and testing)
            
            if (resolution_activated){
                 //down sample the current view
                pcl::VoxelGrid<pcl::PointXYZ> down;
                down.setLeafSize (leaf_resolution, leaf_resolution, leaf_resolution);
                down.setInputCloud (cloud);
                down.filter (*cloud);

            }
           
            
            if (cloud->size() < 5){
                std::cout << "[INFO] Cloud size <2 - pass it" << std::endl;
                continue;
            }
            
            //save the descriptor of each view
            if (cfg.descriptor.enable_vfh){
                std::string descriptor_name = vfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                
                if (boost::filesystem::exists(descriptor_name)){
                    std::cerr << "[INFO] VFH exists already "<< std::endl;
                    
                }else {
                    
                    //vfh
                    vfh_estimation.setInputCluster(*cloud);
                    vfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> vfhs;
                    vfh_estimation.getResultDescriptors(vfhs);
                    
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, vfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, vfhs);
                    }
                    
                }
            }
            if (cfg.descriptor.enable_cvfh){
                std::string descriptor_name = cvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                if (boost::filesystem::exists(descriptor_name)){
                    std::cerr << "[INFO] CVFH exists already "<< std::endl;
                    
                }else {
                    //cvfh
                    cvfh_estimation.setInputCluster(*cloud);
                    cvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> cvfhs;
                    cvfh_estimation.getResultDescriptors(cvfhs);
                    
                    
                    
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, cvfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, cvfhs);
                    }
                }
            }
            
            if (cfg.descriptor.enable_ourcvfh){
                std::string descriptor_name = ourcvfh_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                
                
                if (boost::filesystem::exists(descriptor_name)){
                    std::cerr << "[INFO] OURCVFH exists already "<< std::endl;
                    
                }else {
                    std::cout << "[INFO] Computing OURCVFH... "<< std::flush;
                    //ourcvfh
                    ourcvfh_estimation.setInputCluster(*cloud);
                    ourcvfh_estimation.estimate();
                    pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
                    ourcvfh_estimation.getResultDescriptors(ourcvfhs);
                    std::cout << "Completed"<< std::endl;

                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name, ourcvfhs);
                    }
                }
                
            }
            
            if (cfg.descriptor.enable_esf){
                std::string descriptor_name = esf_dir + "/desc_" + std::to_string(count_data) + std::to_string(i) + ".pcd";
                
                if (boost::filesystem::exists(descriptor_name)){
                    std::cerr << "[INFO] ESF exists already "<< std::endl;
                    
                }else {
                    
                    
                    //esf
                    std::cout << "[INFO] Computing ESF... "<< std::flush;
                    esf_estimation.setInputCluster(*cloud);
                    esf_estimation.estimate();
                    pcl::PointCloud<pcl::ESFSignature640> esfs;
                    esf_estimation.getResultDescriptors(esfs);
                    std::cout << "Completed"<< std::endl;
                    if (cfg.rendering.save_binary){
                        pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name, esfs);
                    }else {
                        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name, esfs);
                    }
                    
                }
            }
            
            
            std::cerr << "[INFO] Changing "<< std::endl;
            count_data++;
        }
    }
    
}
