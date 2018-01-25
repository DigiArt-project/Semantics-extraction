#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#define _USE_MATH_DEFINES
#include <math.h>
//Boost
#include <boost/filesystem.hpp>

/**
 
 Ce programme transforme nuage de point
 
 **/

bool isOBJ = false;
bool isPCD = false;
bool isPLY = false;

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         transform point cloud                           *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << "-query cloud_file (pcd/ply/obj) -output pcd_file tx ty tz (in meters) rx ry rz (degree)" << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}


int main(int argc, char** argv)
{
    
    pcl::PolygonMesh mesh;
    //If not enough parameters
    if (argc < 10)
    {
        std::cerr << "Not enough arguments" <<std::endl;
        showHelp (argv[0]);
        return (-1);
    }
    
    
    std::string query;
    if(pcl::console::parse_argument(argc, argv, "-query", query) == -1)
    {
        std::cerr<<"[Transform point cloud Error] Please specify the query point cloud"<<std::endl;
        return -1;
    }
    std::string output;
    if(pcl::console::parse_argument(argc, argv, "-output", output) == -1)
    {
        std::cerr<<"[Transform point cloud Error] Please specify the output"<<std::endl;
        return -1;
    }
    
    double tx = std::stod(argv[5]);
    double ty = std::stod(argv[6]);
    double tz = std::stod(argv[7]);
    float rx = std::stof(argv[8]) * (M_PI/180) ;
    float ry = std::stof(argv[9]) * (M_PI/180) ;
    float rz = std::stof(argv[10])* (M_PI/180) ;
    
    boost::filesystem::path path_cloud(query);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    if(path_cloud.filename().extension().string() == ".ply"){
        std::cout << "PlY FILE" << std::endl;
        isPLY = true;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(query, *cloud) != 0)
        {
            return -1;
        }
        
    }else if (path_cloud.filename().extension().string() == ".obj"){
        isOBJ = true;
        std::cout << "OBJ FILE" << std::endl;
        if (pcl::io::loadOBJFile (query, mesh) < 0){
            std::cout << "[ERROR] Error loading point cloud " << query << std::endl << std::endl;
            return -1;

        }
        
    }else if (path_cloud.filename().extension().string() == ".pcd"){
        std::cout << "PCD FILE" << std::endl;
        isPCD = true;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(query, *cloud) != 0)
        {
            return -1;
        }
    }else {
         std::cout << "ERROR extension" << std::endl;
    }
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << tx, ty, tz;
    
    transform.rotate (Eigen::AngleAxisf (rx, Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (rz, Eigen::Vector3f::UnitZ()));
    
    if (isOBJ){
        
        //Important part starts here
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        pcl::transformPointCloud(cloud, cloud, transform);
        pcl::toPCLPointCloud2(cloud, mesh.cloud);
    }
    
    
    // Print the transformation
    std::cout << transform.matrix() << std::endl;
    
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    
    if (isPCD){
         pcl::io::savePCDFileBinary(output, *transformed_cloud);
    }else if (isOBJ){
        pcl::io::saveOBJFile(output,mesh);
    }else {
        pcl::io::savePLYFileBinary(output, *transformed_cloud);

    }
   
    
    std::cout << "New point cloud saved" << std::endl;
    
    
    return 0;
}
