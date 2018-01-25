/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

// STL
#include <iostream>
#include <vector>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointXYZ PointT;
//Normal radius for normal computation
float normal_radius = 0.03f;
int default_number_samples = 50000;
float default_leaf_size = 0.01f;
bool compute_all = true;
bool vis_result = false;
bool compute_normals = true;
std::string dataset_folder = "/Users/lironesamoun/digiArt/Datasets/Dataset_structuresensor";


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

inline double
uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = sqrtf (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
    float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);
    
    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());
    
    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    polydata->GetCellPoints (el, npts, ptIds);
    polydata->GetPoint (ptIds[0], A);
    polydata->GetPoint (ptIds[1], B);
    polydata->GetPoint (ptIds[2], C);
    randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                         float (B[0]), float (B[1]), float (B[2]),
                         float (C[0]), float (C[1]), float (C[2]), p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
    polydata->BuildCells ();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
    
    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
    {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }
    
    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;
    
    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        randPSurface (polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}



void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
    print_info ("  where options are:\n");
    print_info ("                     -n_samples X      = number of samples (default: ");
    print_value ("%d", default_number_samples);
    print_info (")\n");
    print_info (
                "                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
    print_value ("%f", default_leaf_size);
    print_info (" m)\n");
    print_info (
                "                     -no_vis_result = flag to stop visualizing the generated pcd\n");
}

/* ---[ */
int
main (int argc, char **argv)
{
    
    print_info ("Convert a CAD model to a point cloud using uniform sampling. For more information, use: %s -h\n",
                argv[0]);
    
    
      if (compute_all){
        if (!boost::filesystem::exists (dataset_folder) && !boost::filesystem::is_directory (dataset_folder)){
            std::cerr << "Error with the directory" << std::endl;
            return -1;
        }
        for(boost::filesystem::recursive_directory_iterator it(dataset_folder); it!=boost::filesystem::recursive_directory_iterator(); ++it)
        {
            
            std::string path_current = it->path().c_str();
            if (it->path().extension().string() == ".ply"){
                //Path to the cloud pcd
                std::string cloud_path = it->path().string();
                std::cout << "Processing : " << cloud_path << std::endl;
                
                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
                
                int SAMPLE_POINTS_ = default_number_samples;
                float leaf_size = default_leaf_size;
                
                std::string parent_directory = it->path().parent_path().string();
                std::string filename = it->path().stem().string();
                std::string output = parent_directory + "/" + filename + ".pcd";
                
                
                
                vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
                pcl::PolygonMesh mesh;
                pcl::io::loadPolygonFilePLY (cloud_path, mesh);
                pcl::io::mesh2vtk (mesh, polydata1);
                
                //make sure that the polygons are triangles!
                vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
                triangleFilter->SetInput (polydata1);
#else
                triangleFilter->SetInputData (polydata1);
#endif
                triangleFilter->Update ();
                
                vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
                triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
                triangleMapper->Update();
                polydata1 = triangleMapper->GetInput();
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
                uniform_sampling (polydata1, SAMPLE_POINTS_, *cloud_1);
                
                // Voxelgrid
                  std::cout << "Before Voxel Grid : " << cloud_1->size() << std::endl;
                VoxelGrid<PointXYZ> grid_;
                grid_.setInputCloud (cloud_1);
                grid_.setLeafSize (leaf_size, leaf_size, leaf_size);
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
                grid_.filter (*res);
                
                std::cout << "Size point cloud sampling : " << res->size() << std::endl;
                
                double val_resolution = compute_resolution(res);
                double size_cloud = res->size();
                std::cout << " Resolution of point cloud : " << val_resolution << " | Number of points : " << size_cloud << std::endl;
                
                bool has_been_saved = false;
                if (compute_normals){
                    std::cout << "Computation Normals, " << std::flush;
                    // Create the normal estimation class, and pass the input dataset to it
                    pcl::NormalEstimation<PointT, pcl::Normal> ne;
                    ne.setInputCloud (res);
                    
                    // Create an empty kdtree representation, and pass it to the normal estimation object.
                    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
                    ne.setSearchMethod (tree);
                    
                    // Output datasets
                    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
                    
                    // Use all neighbors in a sphere of a specific radius
                    ne.setRadiusSearch (normal_radius);
                    
                    // Compute the features
                    ne.compute (*cloud_normals);
                    
                    
                    pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
                    pcl::concatenateFields (*res,*cloud_normals, *point_cloud_normals);
                    
                    //remove NaNs
                    std::vector<int> indexes;
                    pcl::removeNaNNormalsFromPointCloud(*point_cloud_normals,*point_cloud_normals,indexes);
                    std::cout << "Point cloud normal size : " <<point_cloud_normals->size() << std::endl;
                    /*if (boost::filesystem::exists (cloud_path)){
                     remove(cloud_path.c_str());
                     }*/
                    
                    if (!has_been_saved){
                        pcl::io::savePCDFileASCII (output, *res);
                    }

                    if (point_cloud_normals->size() > 0){
                        
                        pcl::io::savePCDFileASCII (output,*point_cloud_normals);
                        has_been_saved = true;
                    }
                    std::cout << "Finished." << std::endl;
                }
              


            }
        }
        
    }else {
        if (argc < 3)
        {
            printHelp (argc, argv);
            return (-1);
        }

        // Parse command line arguments
        int SAMPLE_POINTS_ = default_number_samples;
        parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
        float leaf_size = default_leaf_size;
        parse_argument (argc, argv, "-leaf_size", leaf_size);
        bool vis_result = ! find_switch (argc, argv, "-no_vis_result");
        
        // Parse the command line arguments for .ply and PCD files
        std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
        if (pcd_file_indices.size () != 1)
        {
            print_error ("Need a single output PCD file to continue.\n");
            return (-1);
        }
        std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
        std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
        if (ply_file_indices.size () != 1 && obj_file_indices.size () != 1)
        {
            print_error ("Need a single input PLY/OBJ file to continue.\n");
            return (-1);
        }
        
        vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();;
        if (ply_file_indices.size () == 1)
        {
            pcl::PolygonMesh mesh;
            pcl::io::loadPolygonFilePLY (argv[ply_file_indices[0]], mesh);
            pcl::io::mesh2vtk (mesh, polydata1);
        }
        else if (obj_file_indices.size () == 1)
        {
            vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
            readerQuery->SetFileName (argv[obj_file_indices[0]]);
            readerQuery->Update ();
            polydata1 = readerQuery->GetOutput ();
        }
        
        //make sure that the polygons are triangles!
        vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
        triangleFilter->SetInput (polydata1);
#else
        triangleFilter->SetInputData (polydata1);
#endif
        triangleFilter->Update ();
        
        vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
        triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
        triangleMapper->Update();
        polydata1 = triangleMapper->GetInput();
        
        bool INTER_VIS = false;
        
        if (INTER_VIS)
        {
            visualization::PCLVisualizer vis;
            vis.addModelFromPolyData (polydata1, "mesh1", 0);
            vis.setRepresentationToSurfaceForAllActors ();
            vis.spin();
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
        uniform_sampling (polydata1, SAMPLE_POINTS_, *cloud_1);
        
        if (INTER_VIS)
        {
            visualization::PCLVisualizer vis_sampled;
            vis_sampled.addPointCloud (cloud_1);
            vis_sampled.spin ();
        }
         std::cout << "Before Voxel Grid : " << cloud_1->size() << std::endl;
        // Voxelgrid
        VoxelGrid<PointXYZ> grid_;
        grid_.setInputCloud (cloud_1);
        grid_.setLeafSize (leaf_size, leaf_size, leaf_size);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
        grid_.filter (*res);
        
        std::cout << "Size point cloud sampling : " << res->size() << std::endl;
        
        double val_resolution = compute_resolution(res);
        double size_cloud = res->size();
        std::cout << " Resolution of point cloud : " << val_resolution << " | Number of points : " << size_cloud << std::endl;
        
        bool has_been_saved = false;
        if (compute_normals){
            std::cout << "Computation Normals : " << std::flush;
            // Create the normal estimation class, and pass the input dataset to it
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            ne.setInputCloud (res);
            
            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            ne.setSearchMethod (tree);
            
            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            
            // Use all neighbors in a sphere of a specific radius
            ne.setRadiusSearch (normal_radius);
            
            // Compute the features
            ne.compute (*cloud_normals);
            
            
            pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields (*res,*cloud_normals, *point_cloud_normals);
            
            //remove NaNs
            std::vector<int> indexes;
            pcl::removeNaNNormalsFromPointCloud(*point_cloud_normals,*point_cloud_normals,indexes);
            std::cout << "Point cloud normal size : " <<point_cloud_normals->size() << std::endl;
            /*if (boost::filesystem::exists (cloud_path)){
             remove(cloud_path.c_str());
             }*/
            if (point_cloud_normals->size() > 0){
                
                pcl::io::savePCDFileASCII (argv[pcd_file_indices[0]],*point_cloud_normals);
                has_been_saved = true;
            }
            std::cout << "Finished." << std::endl;
        }
        if (!has_been_saved){
            savePCDFileASCII (argv[pcd_file_indices[0]], *res);
        }
        
        if (vis_result)
        {
            visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
            vis3.addPointCloud (res);
            vis3.spin ();
        }
        
        
    }
}
