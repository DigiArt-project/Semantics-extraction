

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <libconfig.h++>
#include <cerrno>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include "others/typedefs.hpp"


//Filtering
typedef struct {
    float leaf_resolution;
}Filtering;

// Scaling
typedef struct {
    bool scale_data;
    double lower_limit;
    double upper_limit;
    std::string path_to_rangeFile;
    std::string path_to_scale_program;
} Scaling;


//Path txt
typedef struct {
    std::string path_to_dataset_categories;
}Path_dataset;


//Rendering
typedef struct {
    bool enable_rendering;
    int xres;
    int yres;
    int view_angle;
    float tesselation_level;
    bool save_view;
    bool save_binary;
} Rendering;


//similarity search
typedef struct {
    bool build_tree;
    std::string descriptor;
    bool enable_resolution;
    Rendering rendering;
} SimilaritySearch;



// Feature descriptors
typedef struct {
    bool enable_esf;
    bool enable_vfh;
    bool enable_cvfh;
    bool enable_ourcvfh;
} Descriptor;


// SVM
typedef struct {
    float gamma;
    float C;
    float eps;
    std::string path_to_svm;
} Svm;



// Classification parameters
typedef struct{
    bool enable_svm;
    Svm svm;
    Scaling scaling;
} Classification;


// CORE configuration data
typedef struct {
    Descriptor descriptor;
    Rendering rendering;
    Filtering filtering;
    SimilaritySearch similaritySearch;
    Path_dataset path_dataset;
    Classification classification;
} Config;


int getConfiguration (const std::string &file_name, Config &config_cfg);



#endif  // CONFIGURATION_H
