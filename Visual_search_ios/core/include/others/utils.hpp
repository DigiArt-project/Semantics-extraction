
#ifndef Utils_hpp
#define Utils_hpp


#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cerrno>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include "typedefs.hpp"
// Boost headers
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>
//PCL
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/transforms.h>

class Utils
{
    
public:
    /** replace a string by another one
     * @param s string to analyse
     * @param ext portion of the string we want to replace
     */
    static void replaceExt (std::string &s, const std::string &ext);
    /** Assume you have a file with the following structure:
     category_1
     category_1/object_1/Model.ply
     category_1/object_2/Model.ply
     category_1/object_3/Model.ply
     ccategory_2
     category_2/object_1/Model.ply
     ...
     this function fill in a vector which contains only the names of the category present in the txt file. For exemple, the result there will be : category1,category2
     * @param file_name path to the filename which contains the path to the data
     * @param categories vector which will contains all the categories of the data
     */
    static int getCategories (const std::string &file_name, std::vector<std::string> &categories);
    /** Assume you have a file with the following structure:
     category_1
     category_1/object_1/Model.ply
     category_1/object_2/Model.ply
     category_1/object_3/Model.ply
     ccategory_2
     category_2/object_1/Model.ply
     ...
     this function fill in a vector which contains the path to only the cloud present in txt file corresponding to the choosen category. For exemple, if we want to take the data from the category 1, the result there will be :  category_1/object_1/Model.ply, category_1/object_2/Model.ply...
     * @param file_name path to the filename which contains the path to the data
     * @param category name of the category we want to take the data
     * @param data vector which contains all the cloud's path
     */
    static int getDataFilename (const std::string &file_name, const std::string &category, std::vector<std::string> &data);
    /** from a directory path, list all the .pcd (point cloud) file and put the path to the vector of filenames
     * @param directoryPath path of the dataset
     * @param filename vector which will contain the path to the cloud
     */
    static void get_clouds_filenames(const std::string & directoryPath, std::vector<std::string> & filenames);
    /** Compute the resolution of a point cloud
     * @param cloud
     return resolution of the point cloud
     */
    static double compute_resolution(const pcl::PointCloud<PointType>::ConstPtr & cloud);
    
    /** Compute the normals of a point cloud
     * @param cloud
     return normals of the point cloud
     */
    static pcl::PointCloud<pcl::Normal>::Ptr compute_normals(const pcl::PointCloud<PointType>::Ptr & cloud);
    
    static void split(const std::string &s, char delim, std::vector<std::string> &elems);
    /** Given a string and an char, split the string and put the items in a vector
     * @param s string
     * @param delim the delimiter
     return a vector of string which the cut string
     */

    static std::vector<std::string> split(const std::string &s, char delim);
    
    static char* readline(FILE *input);
    
};

#endif  // Utils_hpp
