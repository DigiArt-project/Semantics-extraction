#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <string>
/**
 
Given a point cloud, add more points to it and rise his resolution by applying an upsampling method
 
 **/

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

typedef pcl::PointXYZ PointType;

//Read point cloud from a path
int readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointType> > point_cloud)
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

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualise_both_cloud (pcl::PointCloud<PointType>::ConstPtr cloud1,pcl::PointCloud<PointType>::ConstPtr cloud2)
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
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color1, "sample cloud1", v1);
    
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    std::string number_points_cloud2 = " Number of points : " + std::to_string(cloud2->size());
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
    viewer->addText(number_points_cloud2, 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    
    
    return (viewer);
}


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Upsampling point cloud                  *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " -query pcd_point_cloud [option]" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -output :                 output for saving the new point cloud" << std::endl;
    std::cout << "     -rs :                     Radius Search (0.03 per defaut)" << std::endl;
    std::cout << "     -ur :                     Upsampling radius (0.03 per defaut)" << std::endl;
    std::cout << "     -uss :                    upsampling step size  (0.03 per defaut)" << std::endl;
    std::cout << "     -vis 1/0:                 Visualise results (0.03 per defaut)" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}



int main(int argc, char** argv)
{
    bool visualise_result = false;
    
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
    
    
    std::string query;
    if(pcl::console::parse_argument(argc, argv, "-query", query) == -1)
    {
        std::cerr<<"Please specify the query point cloud -query"<<std::endl;
        return -1;
    }
    
    float radius_search = 0.03f;
    pcl::console::parse_argument (argc, argv, "-rs", radius_search);
    float upsampling_radius = 0.03f;
    pcl::console::parse_argument(argc, argv, "-ur", upsampling_radius);
    float upsampling_step_size = 0.03f;
    pcl::console::parse_argument(argc, argv, "-uss", upsampling_step_size);
    std::string output;
    pcl::console::parse_argument(argc, argv, "-output", output);
    
    std::string visualise = "0";
    
    if(pcl::console::parse_argument(argc, argv, "-vis", visualise) == -1)
    {
        visualise = "0";
    }
    if (std::stoi(visualise) == 1){
        visualise_result = true;
    }else {
        visualise_result = false;
    }
    
    std::cout << "Process : " << query << std::endl;
    
    // Read a PCD file from disk.
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_tmp (new pcl::PointCloud<PointType>);
    
    
    if (readPointCloud( query,  cloud)==-1)
        return -1;
    
    
    pcl::copyPointCloud(*cloud, *cloud_tmp);
    //Keep original
    double val_resolution_original = compute_resolution(cloud);
    
    
    pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
    // Filtering object.
    pcl::MovingLeastSquares<PointType, PointType> filter;
    filter.setInputCloud(cloud);
    // Object for searching.
    pcl::search::KdTree<PointType>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    // Use all neighbors in a radius of 3cm.
    float radius_search_manual = 3;
    
    filter.setSearchRadius(radius_search_manual);
    // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointType>::SAMPLE_LOCAL_PLANE );
    // Radius around each point, where the local plane will be sampled.
    //filter.setPointDensity(200);
    filter.setUpsamplingRadius(upsampling_radius);
    //Sampling step size. Bigger values will yield less (if any) new points.
    filter.setUpsamplingStepSize(upsampling_step_size);
    
    filter.process(*filteredCloud);
    double val_resolution = compute_resolution(filteredCloud);
    double size_cloud = filteredCloud->size();
    std::cout << "New Resolution of point cloud : " << val_resolution << " | Number of points : " << size_cloud << std::endl;
    
    if (!output.empty()){
        
        //SAVING
        std::string extension = boost::filesystem::extension(query);
        if (extension == ".pcd" || extension == ".PCD")
        {
            if (pcl::io::savePCDFile(output, *filteredCloud) == -1)
            {
                std::cout << "\n Cloud saving failed." << std::endl;
                return (-1);
            }
        }
        else if (extension == ".ply" || extension == ".PLY")
        {
            if (pcl::io::savePLYFile(output, *filteredCloud) == -1)
            {
                std::cout << "\n Cloud saving failed." << std::endl;
                return (-1);
            }
        }
        
        
        
    }
    
    if (visualise_result){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = visualise_both_cloud(cloud, filteredCloud);
        
        //--------------------
        // -----Main loop-----
        //--------------------
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    
    
    return 0;
}
