// STL
#include <iostream>
#include <vector>

//NANOFLANN
//#include <nanoflann.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

std::vector<std::string> objects_filenames_;

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
        std::cout << "\n file extension is not correct. Syntax is: compute_descriptor_cloud_main <path/file_name.pcd> [--nogui] or compute_descriptor_cloud_main <path/file_name.ply> [--nogui]" << std::endl;
        return -1;
    }
    return 1;
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

void computeBoundingBoxDimensions (pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointXYZ& dimensions)
{
    pcl::PointXYZ minimum_pt;
    pcl::PointXYZ maximum_pt;
    pcl::getMinMax3D(*pc, minimum_pt, maximum_pt); // min max for bounding box
    dimensions.x = (maximum_pt.x - minimum_pt.x);
    dimensions.y = (maximum_pt.y - minimum_pt.y);
    dimensions.z = (maximum_pt.z - minimum_pt.z);
}

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Add multiple points cloud together                    *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << "cloud1 cloud 2..." << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

int
main (int argc, char** argv)
{
    
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    
    // Objects filenames
    std::vector<int> filenames;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    if (filenames.size () < 1)
    {
        std::cout << "Filenames missing.\n";
        showHelp (argv[0]);
        exit (-1);
    }
    for(int i=0;i<filenames.size();i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>());
        if (readPointCloud(argv[filenames[i]],  object)==-1)
            continue;
        normalizePC(object);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // Define a translation of 2.5 meters on the x axis.
        float x = i * 1.5;
        transform.translation() << x, 0.0, 0.0;
        pcl::transformPointCloud (*object, *object, transform);
        pointclouds.push_back(object);
        
    }
    
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Resolution Viewer"));
    viewer->initCameraParameters ();
    
    viewer->setBackgroundColor (0, 0, 0);
    
    for (int j = 0; j < pointclouds.size();j++){
        std::string name_cloud = "cloud"+std::to_string(j);;
        viewer->addPointCloud<pcl::PointXYZ> (pointclouds.at(j),name_cloud);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,name_cloud);
    }
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while(!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    
    
    
    
}
