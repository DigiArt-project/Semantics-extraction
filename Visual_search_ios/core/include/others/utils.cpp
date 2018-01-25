

#include "utils.hpp"


/** replace a string by another one
 * @param s string to analyse
 * @param ext portion of the string we want to replace
 */
void
Utils::replaceExt (std::string &s, const std::string &ext)
{
    std::string::size_type i = s.rfind ('.', s.length ());
    
    if (i != std::string::npos)
    {
        s.replace (i + 1, ext.length (), ext);
    }
}

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
int
Utils::getCategories (const std::string &file_name, std::vector<std::string> &categories)
{
    std::string line;
    std::ifstream fs;
    
    fs.open (file_name.c_str (), std::ios::in);
    if (!fs.is_open () || fs.fail ())
    {
        std::cerr << "Could not open file : " << file_name.c_str () << "Error : " << strerror (errno) << std::endl;
        fs.close ();
        return (-1);
    }
    
    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.find("/") != std::string::npos) {
            
        }else {
            if (!line.empty()){
                categories.push_back (line);
            }
        }
    }
    
    fs.close ();
    
    return (0);
}
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
int
Utils::getDataFilename (const std::string &file_name, const std::string &category, std::vector<std::string> &data)
{
    std::string line;
    std::ifstream fs;
    
    fs.open (file_name.c_str (), std::ios::in);
    if (!fs.is_open () || fs.fail ())
    {
        std::cerr << "Could not open file : " << file_name.c_str () << "Error : " << strerror (errno) << std::endl;
        fs.close ();
        return (-1);
    }
    
    while (!fs.eof ())
    {
        getline (fs, line);
        
             if (line.find(category) != std::string::npos && line.find("/") != std::string::npos) {
                 data.push_back(line);
             }
        
    }
    
    fs.close ();
    return 0;

}



/** from a directory path, list all the .pcd (point cloud) file and put the path to the vector of filenames
 * @param directoryPath path of the dataset
 * @param filename vector which will contain the path to the cloud
 */
void
Utils::get_clouds_filenames(const std::string & directoryPath, std::vector<std::string> & filenames)
{
    boost::filesystem::path directory(directoryPath);
    filenames.clear();
    
    try
    {
        if (boost::filesystem::exists(directory))
        {
            if (boost::filesystem::is_directory(directory))
            {
                std::vector<boost::filesystem::path> paths;
                std::copy(boost::filesystem::directory_iterator(directory), boost::filesystem::directory_iterator(), std::back_inserter(paths));
                std::sort(paths.begin(), paths.end());
                
                for (std::vector<boost::filesystem::path>::const_iterator it = paths.begin(); it != paths.end(); ++it)
                {
                    if (it->extension().string() == ".pcd")
                    {
                        std::cout << *it << "\n";
                        filenames.push_back(it->relative_path().string());
                    }
                }
            }
        }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        std::cout << ex.what() << '\n';
    }
}

/** Compute the resolution of a point cloud
 * @param cloud
 return resolution of the point cloud
 */
double
Utils::compute_resolution(const pcl::PointCloud<PointType>::ConstPtr & cloud){
#ifdef DEBUG
    std::cout << "Computing cloud resolution...\n";
#endif
    
    double resolution = 0.0;
    int points = 0;
    int nres;
    
    std::vector<int> indices(2);
    std::vector<float> sqrDistances(2);
    pcl::search::KdTree<PointType> kdtree;
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
    
    
#ifdef DEBUG
    std::cout << "Cloud resolution is " << resolution << "\n";
#endif
    
    return resolution;
}

pcl::PointCloud<pcl::Normal>::Ptr
Utils::compute_normals(const pcl::PointCloud<PointType>::Ptr & cloud){
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 2cm
    ne.setRadiusSearch (0.02);
    
    // Compute the features
    ne.compute (*cloud_normals);
    
    return cloud_normals;
}

void Utils::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

/** Given a string and an char, split the string and put the items in a vector
 * @param s string
 * @param delim the delimiter
return a vector of string which the cut string
 */

std::vector<std::string> Utils::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}



