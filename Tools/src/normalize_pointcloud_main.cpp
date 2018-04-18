// STL
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <algorithm>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
/**
 
 Given a point cloud, normalize it.
 
 **/

//Read point cloud from a path
int readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > point_cloud)
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
        std::cout << "\n file extension is not correct." << std::endl;
        return -1;
    }
    return 1;
}

void normalizePC(std::string pointCloudPath, std::string newFilePath){
    std::cout<<"Open old file ..."<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>());
    if (readPointCloud( pointCloudPath,  cloud)==-1)
         std::cerr << "Error loading point cloud " << pointCloudPath << std::endl << std::endl;
        return -1;
    
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
    std::cout << "This program take a point cloud object and normalize it" << std::endl;
    std::cout << "Usage: " << filename << " pointCloudPath output" << std::endl << std::endl;
    
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
    
    std::string pointCloudPath = argv[1];
    std::string output = argv[2];
    
    normalizePC( pointCloudPath,output);

}
