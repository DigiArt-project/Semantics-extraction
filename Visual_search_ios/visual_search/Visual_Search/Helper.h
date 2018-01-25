//
//  Helper.h
//  Visual_Search
//
//  Created by Lirone Samoun on 06/10/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#ifndef Helper_h
#define Helper_h

#include <string>
#include <pcl/io/pcd_io.h>
#include "typedefs.h"


class Helper
{
public:
    /** \brief load point cloud data. Use a mix of C++ and Objective C. in the folder, need a point cloud with the format .pcd
     For example if nameCloud is cup --> the function going to search for cup.pcd
     * \param nameCloud name of the point cloud
     * \param outputCloud the point cloud loaded
     */
    static bool loadPointCloud(std::string nameCloud, pcl::PointCloud<PointType>::Ptr& outputCloud);
    
    
};


#endif /* Helper_h */
