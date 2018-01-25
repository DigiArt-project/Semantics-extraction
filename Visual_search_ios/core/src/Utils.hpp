//
//  Utils.hpp
//  RECONNAISSANCE
//
//  Created by Lirone Samoun on 04/05/2016.
//
//

#ifndef Utils_hpp
#define Utils_hpp

#include <vector>
#include <map>
#include <iostream>
#include "typedefs.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <string>
// Boost headers
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>

using namespace pcl;

class Utils
{
public:
    
    static void get_clouds_filenames(const std::string & directoryPath, std::vector<std::string> & filenames);
    
    static double compute_resolution(const pcl::PointCloud<PointType>::ConstPtr & cloud);

};

#endif /* Utils_hpp */
