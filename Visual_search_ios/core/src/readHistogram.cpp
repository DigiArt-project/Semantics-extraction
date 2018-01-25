#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <typedefs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include "recognition_database/hypothesis.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "descriptor_estimation/esf_estimation.h"

#include "pre_processing/pre_processing.h"

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){}

bool computeResolution = true;


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Read histogram                                  *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << " -query query.pcd -descriptor type_descriptor [Options]" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}
//performs the spatial resolution computation for a given point cloud averaging the distance between each cloud point and its nearest neighbor.thus achieving some sort of resolution invariance that might be useful when using this with different point clouds
double
computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud (cloud);
    
    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

int main(int argc, char** argv)
{
    
    showHelp (argv[0]);
    
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    
    
    std::string cloud_path;
    if(pcl::console::parse_argument(argc, argv, "-query", cloud_path) == -1)
    {
        std::cerr<<"Please specify the query cloud"<<std::endl;
        return -1;
    }
    
    //If we want to see the histograms directly from a object point cloud
    if (argc == 3){
        // Read a PCD file from disk.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, *cloud) != 0)
        {
            return -1;
        }
        
        if (computeResolution){
            double resolution = computeCloudResolution (cloud);
            std::cout << "Resolution of current point cloud : " << computeCloudResolution (cloud) << std::endl;
            pcl::VoxelGrid<pcl::PointXYZ> down;
            down.setLeafSize (0.01, 0.01, 0.01);
            down.setInputCloud (cloud);
            down.filter (*cloud);

            
            std::cout << "Resolution of current point cloud after downsize: " << computeCloudResolution (cloud) << std::endl;
        }
        
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
        OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
        ESFEstimation<pcl::PointXYZ> esf_estimation;
        
        
        //vfh
        vfh_estimation.setInputCluster(*cloud);
        vfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> vfhs;
        vfh_estimation.getResultDescriptors(vfhs);
        //cvfh
        cvfh_estimation.setInputCluster(*cloud);
        cvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> cvfhs;
        cvfh_estimation.getResultDescriptors(cvfhs);
        //ourcvfh
        ourcvfh_estimation.setInputCluster(*cloud);
        ourcvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
        ourcvfh_estimation.getResultDescriptors(ourcvfhs);
        //esf
        esf_estimation.setInputCluster(*cloud);
        esf_estimation.estimate();
        pcl::PointCloud<pcl::ESFSignature640> esfs;
        esf_estimation.getResultDescriptors(esfs);
        
        pcl::visualization::PCLPlotter plotter,plotter_esf;
        plotter.setShowLegend(true);
        plotter_esf.setShowLegend(true);
        
        plotter.addFeatureHistogram(vfhs, 308, "VFH");
        plotter.addFeatureHistogram(cvfhs, 308, "CVFH");
        plotter.addFeatureHistogram(ourcvfhs, 308, "OURCVFH");
        
        plotter_esf.addFeatureHistogram(esfs, 640, "ESF");
        
        
        plotter.plot();
        plotter_esf.plot();
    
    }
    //else we want to see the histogram of a given pcd descriptor file
    else {
        std::string descriptor_type;
        if(pcl::console::parse_argument(argc, argv, "-descriptor", descriptor_type) == -1)
        {
            std::cerr<<"Please specify the descriptor type (esf, vfh, cvfh, ourcvfh)"<<std::endl;
            return -1;
        }
        
        pcl::visualization::PCLPlotter plotter;
        plotter.setShowLegend(true);
        if (descriptor_type.compare("vfh") == 0){
            
            // Read a PCD file from disk.
            pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud (new pcl::PointCloud<pcl::VFHSignature308>);
            
            if (pcl::io::loadPCDFile<pcl::VFHSignature308>(cloud_path, *cloud) != 0)
            {
                return -1;
            }
            
            plotter.addFeatureHistogram(*cloud, 308, "VFH");
            
        }else if (descriptor_type.compare("cvfh") == 0){
            // Read a PCD file from disk.
            pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud (new pcl::PointCloud<pcl::VFHSignature308>);
            
            if (pcl::io::loadPCDFile<pcl::VFHSignature308>(cloud_path, *cloud) != 0)
            {
                return -1;
            }
            
            plotter.addFeatureHistogram(*cloud, 308, "CVFH");
            
        }else if (descriptor_type.compare("ourcvfh") == 0){
            // Read a PCD file from disk.
            pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud (new pcl::PointCloud<pcl::VFHSignature308>);
            
            if (pcl::io::loadPCDFile<pcl::VFHSignature308>(cloud_path, *cloud) != 0)
            {
                return -1;
            }
            
            plotter.addFeatureHistogram(*cloud, 308, "OURCVFH");
            
        }else if (descriptor_type.compare("esf") == 0){
            // Read a PCD file from disk.
            pcl::PointCloud<pcl::ESFSignature640>::Ptr cloud (new pcl::PointCloud<pcl::ESFSignature640>);
            
            if (pcl::io::loadPCDFile<pcl::ESFSignature640>(cloud_path, *cloud) != 0)
            {
                return -1;
            }
            
            
            plotter.addFeatureHistogram(*cloud, 640, "ESF");
            
        }else {
            std::cerr<<"Unknwon descriptor type"<<std::endl;
            return -1;
        }
        
        
        plotter.plot();
        
    }
    return 0;
    
}
