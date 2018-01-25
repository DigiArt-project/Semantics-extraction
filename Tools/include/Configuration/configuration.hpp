

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <libconfig.h++>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>


// Feature descriptors
typedef struct {
    bool enable_esf;
    bool enable_vfh;
    bool enable_cvfh;
    bool enable_ourcvfh;
    bool enable_spin;  // SPIN feature descriptor
    bool enable_usc; //Unique Shape context
    bool enable_sc; //Unique Shape context
    bool enable_grsd; //GRSD descriptor
    bool enable_crh; //CRH descriptor
    bool enable_volume;
    bool enable_gshot; //GShot descriptor
    bool enable_good; //Good descriptor
    int xres;
    int yres;
    int view_angle;
    float tesselation_level;
    float radius_sphere;
    float model_scale;
    bool save_view;
    bool save_binary;
    bool enable_resolution;
    float leaf_resolution;
    std::string path_to_dataset;
    std::string extension_cloud_file;
    bool enable_compute_views;
    int concatenation_descriptors;

} ComputeViewsDescriptors;

// Feature descriptors
typedef struct {
    bool enable_resolution;
    float leaf_resolution;

    
} RetrievalSimilaritySearch;


// CORE configuration data
typedef struct {
    ComputeViewsDescriptors m_computeViewsDescriptors;
    RetrievalSimilaritySearch m_retrievalSimilaritySearch;
} Config;



int getConfiguration (const std::string &file_name, Config &config_cfg);





#endif  // CONFIGURATION_H
