// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <others/typedefs.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

//FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Multifonction for database                            *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << " -option chosenoption [Options]" << std::endl << std::endl;
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
    //If not enough parameters
    if (argc < 3)
    {
        std::cout << "[INFO] Not enough parameters " << std::endl;
        showHelp (argv[0]);
        return (-1);
    }
    
    std::string option;
    if(pcl::console::parse_argument(argc, argv, "-option", option) == -1)
    {
        std::cerr<<"Please specify the option"<<std::endl;
        return -1;
    }
   
    if (option == "buildtree"){
 
    }else if (option == "computeviews"){
        
    }


}
