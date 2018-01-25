
#include "configuration.hpp"
#include <errno.h>



/**  Given a config file, parse all the parameters needed. All the parameters are gathered in the same place. Easier to manage.
 * @param file_name path to the config file
 * @param config_cfg Config object which contains all the parameters of the files
 */
int
getConfiguration (const std::string &file_name, Config &config_cfg)
{
    libconfig::Config cfg;
    
    try
    {
        cfg.readFile (file_name.c_str ());
    }
    catch (const libconfig::FileIOException &fioex)
    {
        std::cerr << "I/O error while reading configuration file" << std::endl;
        return (-1);
    }
    catch (const libconfig::ParseException &pex)
    {
        printf ("Parse error in %s:  line %d - %s\n", pex.getFile (), pex.getLine (), pex.getError ());
        return (-1);
    }
   
    // Get the store name.
    try
    {
        //Paths dataset
        std::string path_dataset_categories = cfg.lookup ("config.path_dataset.path_to_dataset_categories");
        config_cfg.path_dataset.path_to_dataset_categories = path_dataset_categories;
       
        
        //Filtering
        config_cfg.filtering.leaf_resolution             = cfg.lookup ("config.filter.leaf_resolution");
        
        //Similarity search
        config_cfg.similaritySearch.build_tree               = cfg.lookup ("config.similaritySearch.build_tree");
        config_cfg.similaritySearch.enable_resolution               = cfg.lookup ("config.similaritySearch.enable_resolution");
        std::string descriptor_type = cfg.lookup ("config.similaritySearch.descriptor");
        config_cfg.similaritySearch.descriptor  = descriptor_type;
        // Rendering
         config_cfg.similaritySearch.rendering.enable_rendering               = cfg.lookup ("config.similaritySearch.rendering.enable_rendering");
        config_cfg.similaritySearch.rendering.xres               = cfg.lookup ("config.similaritySearch.rendering.xres");
        config_cfg.similaritySearch.rendering.yres               = cfg.lookup ("config.similaritySearch.rendering.yres");
        config_cfg.similaritySearch.rendering.view_angle         = cfg.lookup ("config.similaritySearch.rendering.view_angle");
        config_cfg.similaritySearch.rendering.tesselation_level  = cfg.lookup ("config.similaritySearch.rendering.tesselation_level");
        config_cfg.similaritySearch.rendering.save_view  = cfg.lookup ("config.similaritySearch.rendering.save_view");
        config_cfg.similaritySearch.rendering.save_binary  = cfg.lookup ("config.similaritySearch.rendering.save_binary");
        
        // Feature descriptors
        //ESF
        config_cfg.descriptor.enable_esf              = cfg.lookup ("config.descriptor.enable_esf");
        //CVFH
        config_cfg.descriptor.enable_cvfh              = cfg.lookup ("config.descriptor.enable_cvfh");
        //OUCVFH
        config_cfg.descriptor.enable_ourcvfh          = cfg.lookup ("config.descriptor.enable_ourcvfh");
        //VFH
        config_cfg.descriptor.enable_vfh              = cfg.lookup ("config.descriptor.enable_vfh");
        
        //Scaling
        config_cfg.classification.scaling.scale_data = cfg.lookup ("config.classification.scaling.scale_data");
        config_cfg.classification.scaling.lower_limit = cfg.lookup ("config.classification.scaling.lower_limit");
        config_cfg.classification.scaling.upper_limit = cfg.lookup ("config.classification.scaling.upper_limit");
        std::string pathRange = cfg.lookup ("config.classification.scaling.path_to_rangeFile");
        config_cfg.classification.scaling.path_to_rangeFile = pathRange;
        std::string pathScaleProgram = cfg.lookup ("config.classification.scaling.path_to_scale_program");
        config_cfg.classification.scaling.path_to_scale_program = pathScaleProgram;

        
  
        // Classification parameters
        config_cfg.classification.enable_svm = cfg.lookup ("config.classification.enable_svm");
        
        std::string path_svm = cfg.lookup ("config.classification.svm.path_to_svm");
        config_cfg.classification.svm.path_to_svm  = path_svm;
        
        config_cfg.classification.svm.gamma  = cfg.lookup ("config.classification.svm.gamma");
        config_cfg.classification.svm.C  = cfg.lookup ("config.classification.svm.c");
        config_cfg.classification.svm.eps  = cfg.lookup ("config.classification.svm.eps");
        
        
        
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
         printf ("No 'name' setting in configuration file. - %s \n", nfex.getPath());

    }

    
    return (0);
}
