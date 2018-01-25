#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <others/typedefs.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "configuration/configuration.hpp"

#include "recognition_database/recognition_database.h"

/**
 
 Ce programme permet d'entraîner une database d'objet 3D. L'ensemble des objets 3D doivent être entier (par exemple la base RGBD dataset non). Ici, le programme fonctionne pour la base de donnée du structure sensor. La database a la structure suivante :
 Dataset/category_i/object_j pour i de 1 à N avec N le nombre de catégories et j de 1 à N avec m le nombre d'objet de cette catégorie
 ce programme va itérer dans le dataset et pour chaque objet de chaque catégorie, une caméra virtuelle va être créer pour créer plusieurs views de l'objet. Ensuite pour chaque view de l'objet crée, un descriptor global va être calculé (ESF,VFH,CVFH ou OURCVFH). A la fin, un index FLANN est créer pour accelerer le nearest neighbor

 **/

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){}

int tesselation_level = 1;
float leaf_size_resolution = 0.002f;
int xres = 200;
int yres = 200;
int view_angle = 57;
bool build_tree = true;
bool train = false;
std::string descriptor_type = "esf";


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Training database for similarity search               *" << std::endl;
    std::cout << "*                          (compute views + build tree)                   *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << " config_file -database database_to_train -trained tained_dataset [Options]" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

int main(int argc, char** argv)
{
    
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    //If not enough parameters
    if (argc < 4)
    {
        std::cout << "[INFO] Not enough parameters " << std::endl;
        showHelp (argv[0]);
        return (-1);
    }

    
    std::string configuration_file = argv[1]; // Configuration file to take in account the parameters
    Config cfg;
    
    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0)
        return (-1);

    
    std::string database;
    if(pcl::console::parse_argument(argc, argv, "-database", database) == -1)
    {
        std::cerr<<"Please specify the database directory"<<std::endl;
        return -1;
    }
    
    std::string trained;
    if(pcl::console::parse_argument(argc, argv, "-trained", trained) == -1)
    {
        std::cerr<<"Please specify the trained directory for saving data"<<std::endl;
        return -1;
    }
    
    descriptor_type = cfg.similaritySearch.descriptor;
    build_tree = cfg.similaritySearch.build_tree;
    tesselation_level = cfg.similaritySearch.rendering.tesselation_level;
    leaf_size_resolution = cfg.filtering.leaf_resolution;
    view_angle = cfg.similaritySearch.rendering.view_angle;
    xres = cfg.similaritySearch.rendering.xres;
    yres = cfg.similaritySearch.rendering.yres;
    
    std::string debug_text = "";
    if (cfg.similaritySearch.rendering.enable_rendering){
        train = true;
        std::cout << "[INFO] Rendering" << std::endl;
    }
    else {
        std::cout << "[INFO] No rendering" << std::endl;
        train = false;
    }

    if (descriptor_type.compare("vfh") == 0){
        RecognitionDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::L1<float> > db;
        debug_text = "[INFO] Distance selected : L1 ";
        if (train){
            db.trainDB(database, trained, true, tesselation_level, leaf_size_resolution, xres, yres, view_angle);
        }
        if (build_tree){
            std::cout << "[INFO] Build tree VFH " << std::flush;
            db.setDescriptor(descriptor_type);
            db.buildTree(trained);
            std::cout << "...completed " << std::endl;
        }
        
    }else if (descriptor_type.compare("cvfh") == 0){
        RecognitionDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
        debug_text = "[INFO] Distance selected : ChiQuareDistance ";
        if (train){
            db.trainDB(database, trained, true, tesselation_level, leaf_size_resolution, xres, yres, view_angle);
        }
        if (build_tree){
            std::cout << "[INFO] Build tree CVFH " << std::flush;
            db.setDescriptor(descriptor_type);
            db.buildTree(trained);
            std::cout << "...completed " << std::endl;
        }
        
    }else if (descriptor_type.compare("ourcvfh") == 0){
        RecognitionDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
        debug_text = "[INFO] Distance selected : ChiSquareDistance ";
        if (train){
            db.trainDB(database, trained, true, tesselation_level, leaf_size_resolution, xres, yres, view_angle);
        }
        if (build_tree){
            std::cout << "[INFO] Build tree OURCVFH " << std::flush;
            db.setDescriptor(descriptor_type);
            db.buildTree(trained);
            std::cout << "...completed " << std::endl;
        }
        
    }else if (descriptor_type.compare("esf") == 0){
        RecognitionDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L2<float> > db;
        debug_text = "[INFO] Distance selected : L2 ";
        if (train){
            db.trainDB(database, trained, true, tesselation_level, leaf_size_resolution, xres, yres, view_angle);
        }
        if (build_tree){
            std::cout << "[INFO] Build tree ESF " << std::flush;
            db.setDescriptor(descriptor_type);
            db.buildTree(trained);
            std::cout << "...completed " << std::endl;
        }
        
    }
    else {
        std::cerr<<"Unknwon descriptor type"<<std::endl;
        return -1;
    }
    
    std::cout << debug_text << std::endl;
    
    
    
    return 0;
    
}
