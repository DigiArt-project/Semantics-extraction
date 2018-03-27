// STL
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <algorithm>

//NANOFLANN
//#include <nanoflann.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>


void normalizePC(std::string folderPath, std::string newFilePath){
    std::cout<<"Open old file ..."<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>());
    if( pcl::io::loadPLYFile( folderPath, *cloud) < 0){
        std::cerr << "Error loading point cloud " << folderPath << std::endl << std::endl;
        return;
    }
    
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);

    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud( cloud, NULL);
    kdtree.nearestKSearch( centroid, cloud->size(), indices, distances);//Ks max
    //    std::cout<<"scale factor :"<<distances.back()<<std::endl;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << -centroid.x/sqrt(distances.back()), -centroid.y/sqrt(distances.back()), -centroid.z/sqrt(distances.back());
    transform.scale(1/sqrt(distances.back()));
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    pcl::io::savePLYFile( newFilePath, *transformed_cloud);
    std::cout << newFilePath << " has been saved" << std::endl;
}

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Normalize Point cloud inside a folder           *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "This program take a list of ply objects point cloud and normalize them" << std::endl;
    std::cout << "Usage: " << filename << " Dataset_folder Save_folder" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}


int main(int argc, char *argv[] ){
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
    
    std::string dataset_folder = argv[1];
    std::string save_folder = argv[2];
    
    normalizePC( dataset_folder,save_folder);
    
    /*
    if (!save_folder.empty() && *save_folder.rbegin() != '/')
        save_folder += '/';
    
    for(boost::filesystem::recursive_directory_iterator it(dataset_folder); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {
        
        std::string path_current = it->path().c_str();
       
        if (it->path().extension().string() == ".ply")
        {
            std::string final_save = "";
             std::cout << path_current << std::endl;
            std::string parent_directory = it->path().parent_path().string();
            std::cout << parent_directory << std::endl;
            std::string filename = it->path().filename().c_str();
            std::cout << filename << std::endl;
            final_save = save_folder + filename;
            std::cout << "FINAL : " << save_folder << std::endl;
            normalizePC( path_current,final_save);
        }
        
    }*/

}
