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
#include <pcl/visualization/pcl_visualizer.h>
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

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualise_both_cloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
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


boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_cloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

void computeBoundingBoxDimensions (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc, pcl::PointXYZ& dimensions)
{
    pcl::PointXYZ minimum_pt;
    pcl::PointXYZ maximum_pt;
    pcl::getMinMax3D(*pc, minimum_pt, maximum_pt); // min max for bounding box
    dimensions.x = (maximum_pt.x - minimum_pt.x);
    dimensions.y = (maximum_pt.y - minimum_pt.y);
    dimensions.z = (maximum_pt.z - minimum_pt.z);
}

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
    if (argc < 1)
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
    
    /*
    std::string pointCloudPath = argv[1];
    std::string output = argv[2];
    
    normalizePC( pointCloudPath,output);
     */
    //Other normalization
    std::string pointCloudPath = argv[1];
    std::string output = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>());
    if (readPointCloud( pointCloudPath,  cloud)== - 1)
        std::cerr << "Error loading point cloud " << pointCloudPath << std::endl << std::endl;
    else {
        
        double resolution_wanted = 0.18;
        int length_x = 4;
        int length_y = 6;
        int length_z = 5;
        std::cout << "Cloud size : " << cloud->size() << std::endl;
        std::cout << "Resolution of the cloud : " << compute_resolution(cloud) << std::endl;
        pcl::PointXYZ minimum_pt;
        pcl::PointXYZ maximum_pt;
        pcl::getMinMax3D(*cloud, minimum_pt, maximum_pt); // min max for bounding box
        pcl::PointCloud<pcl::PointXYZ>::Ptr normalized_pc (new pcl::PointCloud<pcl::PointXYZ>);
        
        double ratio_x = ((length_x - 1) * resolution_wanted)/(maximum_pt.x - minimum_pt.x);
        double ratio_y = ((length_y - 1) * resolution_wanted)/(maximum_pt.y - minimum_pt.y);
        double ratio_z = ((length_z - 1) * resolution_wanted)/(maximum_pt.z - minimum_pt.z);
        std::cout << "Ratio X : " << ratio_x << std::endl;
        std::cout << "Ratio Y : " << ratio_y << std::endl;
        std::cout << "Ratio Z : " << ratio_z << std::endl;
        for(typename pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it!= cloud->end(); ++it )
        {
            pcl::PointXYZ pt(it->x,it->y,it->z);
            double new_x = pt.x * ratio_x;
            double new_y = pt.y * ratio_y;
            double new_z = pt.z * ratio_z;
            
            pcl::PointXYZ new_pt(new_x,new_y,new_z);
            normalized_pc->points.push_back(new_pt);

            //std::cout << pt.x << "," << pt.y << "," << pt.z << std::endl;
            
        }
        pcl::getMinMax3D(*normalized_pc, minimum_pt, maximum_pt); // min max for bounding box
        std::cout << "Normalized cloud size : " << normalized_pc->size() << std::endl;
        std::cout << "Resolution Normalized PC : " << compute_resolution(normalized_pc) << std::endl;
        
        
        if (pcl::io::savePLYFile(output, *normalized_pc) == -1)
        {
            std::cout << "\n Cloud saving failed." << std::endl;
            return (-1);
        }
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = visualise_both_cloud(cloud, normalized_pc);
        
        
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    
    }

}
