
#include "Configuration/configuration.hpp"
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
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        printf ("No 'name' setting in configuration file. - %s \n", nfex.getPath());
        
    }
    
    // ##### Compute view descriptors
    std::string path_dataset = cfg.lookup ("config.compute_views_descriptors.path_to_dataset");
    config_cfg.m_computeViewsDescriptors.path_to_dataset = path_dataset;
    
    std::string extension_filename = cfg.lookup ("config.compute_views_descriptors.extension_cloud_filename");
    config_cfg.m_computeViewsDescriptors.extension_cloud_file = extension_filename;
    // Rendering
    config_cfg.m_computeViewsDescriptors.xres               = cfg.lookup ("config.compute_views_descriptors.xres");
    config_cfg.m_computeViewsDescriptors.yres               = cfg.lookup ("config.compute_views_descriptors.yres");
    config_cfg.m_computeViewsDescriptors.view_angle         = cfg.lookup ("config.compute_views_descriptors.view_angle");
    config_cfg.m_computeViewsDescriptors.tesselation_level  = cfg.lookup ("config.compute_views_descriptors.tesselation_level");
    config_cfg.m_computeViewsDescriptors.radius_sphere  = cfg.lookup ("config.compute_views_descriptors.radius_sphere");
    config_cfg.m_computeViewsDescriptors.model_scale  = cfg.lookup ("config.compute_views_descriptors.model_scale");
    config_cfg.m_computeViewsDescriptors.save_view  = cfg.lookup ("config.compute_views_descriptors.save_view");
    config_cfg.m_computeViewsDescriptors.save_binary  = cfg.lookup ("config.compute_views_descriptors.save_binary");
    
    config_cfg.m_computeViewsDescriptors.enable_resolution               = cfg.lookup ("config.compute_views_descriptors.enable_resolution");
    config_cfg.m_computeViewsDescriptors.leaf_resolution               = cfg.lookup ("config.compute_views_descriptors.leaf_resolution");
    // Feature descriptors
    //ESF
    config_cfg.m_computeViewsDescriptors.enable_esf              = cfg.lookup ("config.compute_views_descriptors.enable_esf");
    //CVFH
    config_cfg.m_computeViewsDescriptors.enable_cvfh              = cfg.lookup ("config.compute_views_descriptors.enable_cvfh");
    //OUCVFH
    config_cfg.m_computeViewsDescriptors.enable_ourcvfh          = cfg.lookup ("config.compute_views_descriptors.enable_ourcvfh");
    //VFH
    config_cfg.m_computeViewsDescriptors.enable_vfh              = cfg.lookup ("config.compute_views_descriptors.enable_vfh");
    //SPIN
    config_cfg.m_computeViewsDescriptors.enable_spin              = cfg.lookup ("config.compute_views_descriptors.enable_spin");
    //USC
    config_cfg.m_computeViewsDescriptors.enable_usc              = cfg.lookup ("config.compute_views_descriptors.enable_usc");
    //3DSC
    config_cfg.m_computeViewsDescriptors.enable_sc              = cfg.lookup ("config.compute_views_descriptors.enable_sc");
    //GRSD
    config_cfg.m_computeViewsDescriptors.enable_grsd              = cfg.lookup ("config.compute_views_descriptors.enable_grsd");
    //CRH
    config_cfg.m_computeViewsDescriptors.enable_crh             = cfg.lookup ("config.compute_views_descriptors.enable_crh");
    //GShot
    config_cfg.m_computeViewsDescriptors.enable_gshot             = cfg.lookup ("config.compute_views_descriptors.enable_gshot");
    //Volume
    config_cfg.m_computeViewsDescriptors.enable_volume             = cfg.lookup ("config.compute_views_descriptors.enable_volume");
    //GOOD
    config_cfg.m_computeViewsDescriptors.enable_good             = cfg.lookup ("config.compute_views_descriptors.enable_good");
    
    
    config_cfg.m_computeViewsDescriptors.enable_compute_views             = cfg.lookup ("config.compute_views_descriptors.enable_compute_views");
    
    config_cfg.m_computeViewsDescriptors.concatenation_descriptors           = cfg.lookup ("config.compute_views_descriptors.concatenation_descriptors");

    
    

    
    
    // ##### Similarity Search retrieval
    config_cfg.m_retrievalSimilaritySearch.enable_resolution               = cfg.lookup ("config.retrieval_similarity_search.enable_resolution");
    config_cfg.m_retrievalSimilaritySearch.leaf_resolution               = cfg.lookup ("config.retrieval_similarity_search.leaf_resolution");

    

    

    
    
    
    
    
    
    
    return (0);
}
