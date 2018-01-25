//
//  Helper.m
//  Visual_Search
//
//  Created by Lirone Samoun on 06/10/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#import "Helper.h"

/*
bool Helper::loadPointCloud(std::string nameCloud, pcl::PointCloud<PointType>::Ptr& outputCloud){
    NSString *documentsDirectoryPath = [NSSearchPathForDirectoriesInDomains( NSDocumentDirectory, NSUserDomainMask, YES ) objectAtIndex:0];
    
    NSString *nameCloudNsString = [NSString stringWithCString:nameCloud.c_str()
                                                encoding:[NSString defaultCStringEncoding]];
    NSString *cloud_pcd = [nameCloudNsString stringByAppendingString:@".pcd"];
    
    NSString *cloud_pcd_path = [documentsDirectoryPath stringByAppendingPathComponent:cloud_pcd];
    //NSLog(@"Point cloud loaded path: %@ ", cloud_pcd_path);
    
    const char *cpath = [cloud_pcd_path fileSystemRepresentation];
    std::string cloud_pcd_String(cpath);
    
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(cloud_pcd_String.c_str(), *cloud) == -1) //* load the file
    {
        NSLog(@"[INFO] error");
        return false;
        
        
    }else {
        std::cout << "PCD Point cloud loaded with succees " << std::endl;
        NSLog(@"[INFO] Point cloud loaed with success");
        NSLog(@"[INFO] Size cloud point %zu : ", cloud->size());
        outputCloud = cloud;
        return true;
        
        
    }
    

}
*/
