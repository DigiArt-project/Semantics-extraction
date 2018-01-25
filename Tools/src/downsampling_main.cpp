// STL
#include <iostream>
#include <vector>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointTnormals;

float leaf_size = 1.0f;

bool visualise_result = false;
bool compute_all = true;
bool save = true;
std::string dataset_folder = "/Users/lironesamoun/digiArt/Datasets/ModelNet10";


boost::shared_ptr<pcl::visualization::PCLVisualizer> visualise_both_cloud (pcl::PointCloud<PointTnormals>::ConstPtr cloud1,pcl::PointCloud<PointTnormals>::ConstPtr cloud2)
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
    viewer->addText(number_points_cloud1, 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointTnormals> single_color1(cloud1, 0, 255, 0);
    viewer->addPointCloud<PointTnormals> (cloud1, single_color1, "sample cloud1", v1);
    
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    std::string number_points_cloud2 = " Number of points : " + std::to_string(cloud2->size());
    pcl::visualization::PointCloudColorHandlerCustom<PointTnormals> single_color2(cloud2, 0, 255, 0);
    viewer->addText(number_points_cloud2, 10, 10, "v2 text", v2);
    viewer->addPointCloud<PointTnormals> (cloud2, single_color2, "sample cloud2", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    
    
    return (viewer);
}



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

//Read point cloud from a path
int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointTnormals> > point_cloud)
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
    std::cout << "*                   Downsample points cloud                               *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << "-cloud pointCloud" << std::endl << std::endl;

    std::cout << "Options:" << std::endl;
    std::cout << "     -lf:                     leaf size (defaut : 0.01f)" << std::endl;
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
                std::cout << "INFO : Downsampling point cloud using leaf size "<< leaf_size << "..." << std::endl;
                std::cout << "Processing : " << cloud_path << std::endl;
                
                pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
                if (readPointCloud( cloud_path,  cloud)==-1)
                    return -1;
                
                // Filter object.
                pcl::VoxelGrid<pcl::PointNormal> filter;
                filter.setInputCloud(cloud);

                filter.setLeafSize(leaf_size, leaf_size, leaf_size);
                std::cout << "Before size : " << cloud->size() << std::endl;
                filter.filter(*cloud);
                std::cout << "After size : " << cloud->size() << std::endl;
                
                
                if (save && cloud->size() > 0){
                    
                    pcl::io::savePCDFileASCII (cloud_path,*cloud);
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
        if(pcl::console::parse_argument(argc, argv, "-lf", leaf_size) == 1)
        {
            
        }
        
        std::cout << "INFO : Downsampling point cloud using leaf size "<< leaf_size << "..." << std::endl;
        //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointNormal>);
        if (readPointCloud( cloud_path,  cloud)==-1)
            return -1;
        
        pcl::copyPointCloud(*cloud, *cloud_tmp);
        //Keep original
        
        // Filter object.
        pcl::VoxelGrid<pcl::PointNormal> filter;
        filter.setInputCloud(cloud);
        std::cout << "Before size : " << cloud->size() << std::endl;
        filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        filter.filter(*cloud);
        std::cout << "After size : " << cloud->size() << std::endl;
        
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
        std::cout << "Finished" << std::endl;

        
        if (visualise_result){
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = visualise_both_cloud(cloud_tmp, cloud);
            
            //--------------------
            // -----Main loop-----
            //--------------------
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            }
        }

        

        
     
    }


}
