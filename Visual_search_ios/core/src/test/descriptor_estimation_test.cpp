#include "descriptor_estimation/esf_estimation.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "pre_processing/pre_processing.h"
//#include "openni_frame_source/openni_frame_source.h"

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/**
 
 Ce programme calcul descripteurs globaux d'un nuage de point
 
 **/

int main(int argc, char** argv)
{
    
    std::string roi;
    if(pcl::console::parse_argument(argc, argv, "-roi", roi) == -1)
    {
        std::cerr<<"Unable to read roi.txt"<<std::endl;
        return -1;
    }
    
    std::string query;
    if(pcl::console::parse_argument(argc, argv, "-query", query) == -1)
    {
        std::cerr<<"Please specify the query point cloud"<<std::endl;
        return -1;
    }
    
    // Read a PCD file from disk.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr copied_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(query, *copied_cloud) != 0)
    {
        return -1;
    }
    
    
    PreProcessing<pcl::PointXYZRGB> pp;
    pp.setROIBoundary(roi);
    
    
    pp.setInputScene(copied_cloud);
    pp.process();
    pcl::PointCloud<pcl::PointXYZRGB>::CloudVectorType clusters;
    pp.getResultClusters(clusters);
    
    ESFEstimation<pcl::PointXYZRGB> esf_estimation;
    VFHEstimation<pcl::PointXYZRGB> vfh_estimation;
    CVFHEstimation<pcl::PointXYZRGB> cvfh_estimation;
    OURCVFHEstimation<pcl::PointXYZRGB> ourcvfh_estimation;
    
    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
        //ESF
        esf_estimation.setInputCluster(clusters[i]);
        esf_estimation.estimate();
        pcl::PointCloud<pcl::ESFSignature640> esfs;
        esf_estimation.getResultDescriptors(esfs);
        
        std::stringstream ss;
        ss<<"esfs_"<<i<<".pcd";
        pcl::io::savePCDFileBinary<pcl::ESFSignature640>(ss.str(), esfs);
        
        //VFH
        vfh_estimation.setInputCluster(clusters[i]);
        vfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> vfhs;
        vfh_estimation.getResultDescriptors(vfhs);
        
        ss.str("");
        ss<<"vfhs_"<<i<<".pcd";
        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(ss.str(), vfhs);
        
        //CVFH
        cvfh_estimation.setInputCluster(clusters[i]);
        cvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> cvfhs;
        cvfh_estimation.getResultDescriptors(cvfhs);
        
        ss.str("");
        ss<<"cvfhs_"<<i<<".pcd";
        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(ss.str(), cvfhs);
        
        //OUR-CVFH
        ourcvfh_estimation.setInputCluster(clusters[i]);
        ourcvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
        ourcvfh_estimation.getResultDescriptors(ourcvfhs);
        
        ss.str("");
        ss<<"ourcvfh_"<<i<<".pcd";
        pcl::io::savePCDFileBinary<pcl::VFHSignature308>(ss.str(), ourcvfhs);
    }
    
    return 0;
}
