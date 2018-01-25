//
//  Utils.cpp
//  RECONNAISSANCE
//
//  Created by Lirone Samoun on 04/05/2016.
//
//

#include "Utils.hpp"

void Utils::get_clouds_filenames(const std::string & directoryPath, std::vector<std::string> & filenames)
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

double Utils::compute_resolution(const pcl::PointCloud<PointType>::ConstPtr & cloud){
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
