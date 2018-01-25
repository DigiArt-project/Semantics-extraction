/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:
 *
 */

///**
// Programme qui permet de convertir des fichiers .obj en .pcd
// **/
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <vtkSmartPointer.h>
//#include <vtkPLYReader.h>
//#include <vtkPolyData.h>
//#include <pcl/common/transforms.h>
//
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizeCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//{
//    // --------------------------------------------
//    // -----Open 3D viewer and add point cloud-----
//    // --------------------------------------------
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D cloud Viewer"));
//    viewer->setBackgroundColor (1, 1, 1);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer->setSize(2000, 2000);
//    viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb, "cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    
//    
//    
//    while(!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }
//    
//    return (viewer);
//}
//
//void
//printHelp (int, char **argv)
//{
//    print_error ("Syntax is: %s input.obj output.pcd [options]\n", argv[0]);
//    print_info ("where options are: \n");
//    print_info ("     -copy_normals 0/1 : set to true (1) or false (0) if the output PointCloud should contain normals or not.\n");
//}
//
///* ---[ */
//int
//main (int argc, char** argv)
//{
//    print_info ("Convert a OBJ file to PCD format. For more information, use: %s -h\n", argv[0]);
//    
//    if (argc < 3)
//    {
//        printHelp (argc, argv);
//        return (-1);
//    }
//    
//    // Parse the command line arguments for .pcd and .obj files
//    std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
//    std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
//    if (pcd_file_indices.size () != 1 || obj_file_indices.size () != 1)
//    {
//        print_error ("Need one input OBJ file and one output PCD file.\n");
//        return (-1);
//    }
//    
//    // Load the OBJ file
//    TicToc tt;
//    print_highlight ("Loading "); print_value ("%s ", argv[obj_file_indices[0]]);
//    
//    std::string input = argv[1];
//    std::string output = argv[2];
//
//    pcl::PolygonMesh mesh;
//    pcl::PolygonMesh mesh_rgb;
//    
//    // Read a obj file from disk.
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    
//    if (pcl::io::loadOBJFile<pcl::PointXYZRGB>(input, *cloud) != 0)
//    {
//        return -1;
//    }
//    if (pcl::io::loadOBJFile (input, mesh) < 0)  {
//        std::cout << "[ERROR] Error loading point cloud " << input << std::endl;
//        return -1;
//    }
//    
//  
//    
//    pcl::PointCloud<pcl::PointXYZRGB> vertices_rgb;
//    pcl::fromPCLPointCloud2 (mesh.cloud, vertices_rgb);
//    mesh_rgb.polygons = mesh.polygons;
//    for (size_t i = 0; i < vertices_rgb.size (); ++i)
//    {
//        pcl::PointXYZRGB &pt_rgb = vertices_rgb.at (i);
//       // pt_rgb.r = static_cast<uint8_t> (255);
//       // pt_rgb.g = static_cast<uint8_t> (0);
//        //pt_rgb.b = static_cast<uint8_t> (0);
//    }
//    for (int i = 0; i < cloud->size();i++){
//        pcl::PointXYZRGB &pt_rgb = vertices_rgb.at (i);
//        uint32_t rgb = cloud->points[i].rgba;
//        uint8_t r = (rgb >> 16) & 0x0000ff;
//        uint8_t g = (rgb >> 8)  & 0x0000ff;
//        uint8_t b = (rgb)     & 0x0000ff;
//        pt_rgb.r = r;
//        pt_rgb.g = g;
//        pt_rgb.b = b;
//        
//        
//    }
//    
//    
//    pcl::toPCLPointCloud2 (vertices_rgb, mesh_rgb.cloud);
//    
//    pcl::io::savePLYFileBinary("Model_test.ply", mesh_rgb);
//    
//    
//    std::cout << "OBJ has been read" << std::endl;
//    
//    //remove NAN points from the cloud
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
//    
//    pcl::io::savePCDFile<pcl::PointXYZRGB>(output, *cloud);
//    
//    //Important part starts here
//    //pcl::PointCloud<pcl::PointXYZRGB> cloud1;
//    //pcl::fromPCLPointCloud2(mesh.cloud, cloud1);
//    //pcl::toPCLPointCloud2(cloud1, mesh.cloud);
//    
//    
//    
//
//    
//      std::cout << "PCD has been saved" << std::endl;
//    std::string output_ply = "Model.ply";
//    pcl::io::savePLYFileASCII<pcl::PointXYZRGB>(output_ply, *cloud);
//    std::cout << "PLY has been saved" << std::endl;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcd (new pcl::PointCloud<pcl::PointXYZRGB>);
//    
//    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("test.pcd", *cloud_pcd) != 0)
//    {
//        return -1;
//    }
//    std::cout << "PCD has been read" << std::endl;
//     std::cout << "PCD saved" << std::endl;
//    visualizeCloud(cloud_pcd);
//    
//    return (0);
//}
//

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.obj output.pcd [options]\n", argv[0]);
    print_info ("where options are: \n");
    print_info ("     -copy_normals 0/1 : set to true (1) or false (0) if the output PointCloud should contain normals or not.\n");
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
    
    // Load the input file
    vtkSmartPointer<vtkPolyData> polydata;
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New ();
    reader->SetFileName (argv[obj_file_indices[0]]);
    reader->Update ();
    polydata = reader->GetOutput ();
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", polydata->GetNumberOfPoints ()); print_info (" points]\n");
    
    // Setup the colors array
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtkDataArray* insideArray  = polydata->GetPointData()->GetScalars();
    
    bool copy_normals = false;
    parse_argument (argc, argv, "-copy_normals", copy_normals);
    PCL_INFO ("Copy normals: %s.\n", copy_normals ? "true" : "false");
    
    if (copy_normals)
    {
        vtkSmartPointer<vtkPolyDataNormals> ng = vtkSmartPointer<vtkPolyDataNormals>::New ();
#if VTK_MAJOR_VERSION < 6
        ng->SetInput (polydata);
#else
        ng->SetInputData (polydata);
#endif
        ng->ComputePointNormalsOn ();
        ng->ComputeCellNormalsOff ();
        ng->Update ();
        polydata = ng->GetOutput ();
        
        pcl::PointCloud<pcl::PointNormal> cloud;
        vtkPolyDataToPointCloud (polydata, cloud);
        // Convert to pcd and save
        saveCloud (argv[pcd_file_indices[0]], cloud);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        vtkPolyDataToPointCloud (polydata, cloud);
        // Convert to pcd and save
        saveCloud (argv[pcd_file_indices[0]], cloud);
    }
    
    return (0);
}


