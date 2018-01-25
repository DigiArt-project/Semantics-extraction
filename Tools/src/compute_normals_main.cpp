// STL
#include <iostream>
#include <vector>

//NANOFLANN
//#include <nanoflann.hpp>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/features/normal_3d.h>
#include <others/typedefs.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <stdio.h>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ PointT;

//Normal radius for normal computation
float normal_radius = 0.5f;

//If instead of computing one point cloud, you want to compute inside folder

bool compute_all = true;
std::string dataset_folder = "/Users/lironesamoun/digiArt/Datasets/ModelNet10";
std::string subfolder = "views";
//If not views folder


//Read point cloud from a path
int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud)
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

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Compute normals of point cloud                        *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << "-cloud pointCloud" << std::endl << std::endl;

    std::cout << "Options:" << std::endl;
    std::cout << "     -nr:                     normal radius" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}


int
main (int argc, char** argv)
{

    if (compute_all){

        if (!boost::filesystem::exists (dataset_folder) && !boost::filesystem::is_directory (dataset_folder)){
            std::cerr << "Error with the directory" << std::endl;
            return -1;
        }
        for(boost::filesystem::recursive_directory_iterator it(dataset_folder); it!=boost::filesystem::recursive_directory_iterator(); ++it)
        {
            
            std::string path_current = it->path().c_str();
            //if (path_current.find("/"+subfolder+"/")  != std::string::npos && it->path().extension().string() == ".pcd"){
            if (it->path().extension().string() == ".pcd"){
                //Path to the cloud pcd
                std::string cloud_path = it->path().string();
                std::cout << "Processing : " << cloud_path << std::endl;
                
                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
                if (readPointCloud( cloud_path,  cloud)==-1)
                    return -1;
                

                // Create the normal estimation class, and pass the input dataset to it
                pcl::NormalEstimation<PointT, pcl::Normal> ne;
                ne.setInputCloud (cloud);
                
                // Create an empty kdtree representation, and pass it to the normal estimation object.
                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
                ne.setSearchMethod (tree);
                
                // Output datasets
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
                
                // Use all neighbors in a sphere of a specific radius
                ne.setRadiusSearch (normal_radius);
                
                // Compute the features
                ne.compute (*cloud_normals);

                
                pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields (*cloud,*cloud_normals, *point_cloud_normals);
                
                //remove NaNs
                std::vector<int> indexes;
                pcl::removeNaNNormalsFromPointCloud(*point_cloud_normals,*point_cloud_normals,indexes);
                //std::cout << "Point cloud normal size : " <<point_cloud_normals->size() << std::endl;
                /*if (boost::filesystem::exists (cloud_path)){
                    remove(cloud_path.c_str());
                }*/
                if (point_cloud_normals->size() > 0){
                   
                    pcl::io::savePCDFileASCII (cloud_path,*point_cloud_normals);
                    std::cout << "Save to : " <<cloud_path << std::endl;
                }
                
            }
        }
        std::cout << "Finished" << std::endl;
        
    }
    else {
        
        //Show help
        if (pcl::console::find_switch (argc, argv, "-h"))
        {
            showHelp (argv[0]);
            exit (0);
        }
        //If not enough parameters
        if (argc < 2)
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
        if(pcl::console::parse_argument(argc, argv, "-nr", normal_radius) == 1)
        {
        }
        
        std::cout << "INFO : computing normals for point cloud..." << std::flush;
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        if (readPointCloud( cloud_path,  cloud)==-1)
            return -1;
        
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud (cloud);
        
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        ne.setSearchMethod (tree);
        
        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        
        // Use all neighbors in a sphere of a specific radius
        ne.setRadiusSearch (normal_radius);
        
        // Compute the features
        ne.compute (*cloud_normals);
        
        pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud,*cloud_normals, *point_cloud_normals);
        
        //remove NaNs
        std::vector<int> indexes;
        pcl::removeNaNNormalsFromPointCloud(*point_cloud_normals,*point_cloud_normals,indexes);
        
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *point_cloud_normals);
        std::cout << "Finished" << std::endl;

    }


}
