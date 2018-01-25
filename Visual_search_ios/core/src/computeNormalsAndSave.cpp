
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include "others/Utils.hpp"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.obj output.pcd\n", argv[0]);

}

template <typename T> void
saveCloud (const std::string &filename, const pcl::PointCloud<T> &cloud)
{
    TicToc tt;
    tt.tic ();
    
    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
    
    PCDWriter w;
    w.writeBinaryCompressed (filename, cloud);
    
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
    print_info ("Convert a OBJ file to PCD format. For more information, use: %s -h\n", argv[0]);
    
    if (argc < 3)
    {
        printHelp (argc, argv);
        return (-1);
    }
    
    std::string input_query = argv[1];
    std::string output = argv[2];


    
    // Parse the command line arguments for .pcd and .obj files
    std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
    if (pcd_file_indices.size () != 1 || obj_file_indices.size () != 1)
    {
        print_error ("Need one input OBJ file and one output PCD file.\n");
        return (-1);
    }
    
    // Load the OBJ file
    TicToc tt;
    print_highlight ("Loading "); print_value ("%s ", argv[obj_file_indices[0]]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadOBJFile (input_query, *cloud) < 0){
        std::cout << "[ERROR] Error loading point cloud " << input_query << std::endl;
        return -1;
        
    }
    //Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = Utils::compute_normals(cloud);
    
    //Point cloud with normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    
    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    
    print_highlight ("Saving "); print_value ("%s ", input_query.c_str ());
    pcl::io::savePCDFileASCII (output, *cloud_with_normals);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_with_normals->width * cloud_with_normals->height); print_info (" points]\n");
    
    return (0);
}

