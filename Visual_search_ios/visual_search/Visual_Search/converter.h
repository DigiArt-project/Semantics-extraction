//
//  converter.hpp
//  Visual_search_app
//
//  Created by Lirone Samoun on 27/09/2016.
//  Copyright © 2016 Occipital. All rights reserved.
//

#ifndef converter_hpp
#define converter_hpp

#include <string>

class Converter
{
public:
    /** \brief convert a pcd cloud point to obj cloud point
     * \param inputFilename the path to the pcd file cloud point
     * \param outputFilename path to the output of the obj point cloud
     */
    static bool pcd2obj(const std::string& inputFilename, const std::string& outputFilename = "output.obj");
    
    /** \brief convert a obj cloud point to pcd cloud point
     * \param inputFilename the path to the obj file cloud point
     * \param outputFilename path to the output of the pcd point cloud
     */
    static bool obj2pcd(const std::string& inputFilename, const std::string& outputFilename = "output.pcd");
};

#endif /* converter_hpp */
