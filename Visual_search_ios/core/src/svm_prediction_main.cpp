
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "configuration/configuration.hpp"

#include "recognition_database/hypothesis.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "descriptor_estimation/esf_estimation.h"

#include <pcl/filters/voxel_grid.h>

#include "others/utils.hpp"
#include "others/Timer.hpp"

#include "svm/libsvm/svm.h"
#include "svm/svm_predict.hpp"

std::string output_path_result = "../result.label";
std::string tmp_desc_scale = "tmp_descriptor.scale";

/**
 * Ce programme prédit la catégorie du point cloud qu'on donne en Input
 */

void getCategories(std::string path_to_data, std::vector<std::string>& categories){
    
    for(boost::filesystem::directory_iterator it(path_to_data); it!=boost::filesystem::directory_iterator(); ++it)
    {
        std::string path_current = it->path().c_str();
        boost::filesystem::path p(path_current);
        std::string filename = p.filename().c_str();
        if (boost::filesystem::is_directory(path_current)){
            categories.push_back(filename);
        }
    }
    
}



/** given a label (int) and a file which contain the categories, we associate the numer to the label (string)
 * @param label number predicted
 * @param path_to_fileCategory path to the cateogry file
 */
std::string giveNameCategoryOfLabel(const int label, const std::string path_to_fileCategory){
    std::string result;
    std::vector<std::string> categories;
    //Get category
    std::cout << "[INFO] Path to file categories : " << path_to_fileCategory << std::endl;
    Utils::getCategories (path_to_fileCategory,categories);
    
    std::map <int,string> map_categories;
    //Asociate each category with the corresponding number label
    for( int k=0;k<categories.size();k++){
        map_categories.insert(pair<int,string>(k+1,string(categories.at(k))));
    }
    //Find the label
    if ( map_categories.find(label) != map_categories.end() ) {
        result =  map_categories.find(label)->second;
    } else {
        result = "NOT FOUND LABEL";
    }
    return result;
}

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         SVM prediction                                  *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " config (config file cfg format) model_file (svm model) pcd_file (point cloud to test) type_descriptor (esf/vfh/CVFH, OURCVFH)" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    std::cout << "     -output:                where to save the label results file. Per default : ../result.label" << std::endl;
    
}

int
main (int argc, char** argv)
{
    
    
    //If not enough parameters
    if (argc < 5)
    {
        showHelp (argv[0]);
        return (-1);
    }
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    //Fill in the parameters
    std::string configuration_file = argv[1];
    std::string model_file = argv[2];
    std::string pcd_file = argv[3];
    std::string type_descriptor = argv[4];
    pcl::console::parse_argument (argc, argv, "-output", output_path_result);
    
    if (!type_descriptor.empty() || type_descriptor.compare("vfh") == 0 || type_descriptor.compare("esf") == 0 || type_descriptor.compare("cvfh") == 0 || type_descriptor.compare("ourcvfh") == 0){
        
        
        
        Config cfg;
        bool scaling = false;
        
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        if (getConfiguration (configuration_file, cfg) < 0){
            return (-1);
        }else {
            std::cout << "[INFO] Configuation file loaded" << std::endl;
        }
        
        if (cfg.classification.scaling.scale_data){
            scaling = true;
        }
        
        
        std::string path_to_rangefile = cfg.classification.scaling.path_to_rangeFile;
        std::string path_scale_program = cfg.classification.scaling.path_to_scale_program;
        std::string out_result = "./results/label_predicted_result.txt";
        std::string range_file;
        
        if (type_descriptor.compare("vfh") == 0){
            range_file = path_to_rangefile + "training_vfh.range";
        }
        else if (type_descriptor.compare("cvfh") == 0){
            range_file = path_to_rangefile  + "training_cvfh.range";
        }
        else if (type_descriptor.compare("ourcvfh") == 0){
            range_file = path_to_rangefile +  + "training_ourcvfh.range";
        }
        else if (type_descriptor.compare("esf") == 0){
            range_file = path_to_rangefile  + "training_esf.range";
        }
        
        std::cout << "[Config] Path to range file: " << path_to_rangefile << std::endl;


        //Get the svm model
        struct svm_model* model = svm_load_model (model_file.c_str ());
        if (model == NULL)
        {
            std::cerr << "[ERROR] Could not load SVM model file : " << model_file << std::endl;
            return (-1);
        }
        //Load the pcd point cloud
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file.c_str (), *cloud) < 0)
        {
            std::cerr << "[ERROR] Could not load point cloud file : " << pcd_file << std::endl;
            return (-1);
        }
        double label = 0;
        
        float leaf_size_resolution = cfg.filtering.leaf_resolution;
        if (cfg.similaritySearch.enable_resolution){
            std::cout << "[INFO] Resolution activated SVM " <<std::endl;
            pcl::VoxelGrid<pcl::PointXYZ> down;
            down.setLeafSize (leaf_size_resolution, leaf_size_resolution, leaf_size_resolution);
            down.setInputCloud (cloud);
            down.filter (*cloud);
        }
        
        
        pcl::PointCloud<pcl::PointXYZ> completeModel;
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
        OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
        ESFEstimation<pcl::PointXYZ> esf_estimation;
        
        //Compute VFH descriptor
        if (type_descriptor.compare("vfh") == 0){
            std::cout << "[INFO] Computing VFH descriptor..." << std::endl;
            //vfh
            vfh_estimation.setInputCluster(*cloud);
            vfh_estimation.estimate();
            std::cout << "[INFO] Computation completed" << std::endl;
            pcl::PointCloud<pcl::VFHSignature308> vfhs;
            vfh_estimation.getResultDescriptors(vfhs);
            
            
            
            if (doScalingVFH(range_file,path_scale_program,vfhs)){
                label =doPrediction(tmp_desc_scale,model_file,out_result);
            }
            
        }
        //Compute CVFH descriptor
        if (type_descriptor.compare("cvfh") == 0){
            std::cout << "[INFO] Computing CVFH descriptor..." << std::endl;
            //cvfh
            cvfh_estimation.setInputCluster(*cloud);
            cvfh_estimation.estimate();
            std::cout << "[INFO] Computation completed" << std::endl;
            pcl::PointCloud<pcl::VFHSignature308> cvfhs;
            cvfh_estimation.getResultDescriptors(cvfhs);
            
            if (doScalingVFH(range_file,path_scale_program,cvfhs)){
                label =doPrediction(tmp_desc_scale,model_file,out_result);
            }
            
        }
        //Compute OURCVFH descriptor
        if (type_descriptor.compare("ourcvfh") == 0){
            std::cout << "[INFO] Computing OURCVFH descriptor..." << std::endl;
            //ourcvfh
            ourcvfh_estimation.setInputCluster(*cloud);
            ourcvfh_estimation.estimate();
            std::cout << "[INFO] Computation completed" << std::endl;
            pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
            ourcvfh_estimation.getResultDescriptors(ourcvfhs);
            
            if (doScalingVFH(range_file,path_scale_program,ourcvfhs)){
                label =doPrediction(tmp_desc_scale,model_file,out_result);
            }
        }
        //Compute ESG descriptor
        if (type_descriptor.compare("esf") == 0){
            std::cout << "[INFO] Computing ESF descriptor..." << std::endl;
            //esf
            esf_estimation.setInputCluster(*cloud);
            esf_estimation.estimate();
            pcl::PointCloud<pcl::ESFSignature640> esfs;
            std::cout << "[INFO] Computation completed" << std::endl;
            esf_estimation.getResultDescriptors(esfs);
            
            if (doScalingESF(range_file,path_scale_program,esfs)){
                label =doPrediction(tmp_desc_scale,model_file,out_result);
            }
            
        }
        
        std::string name_category = giveNameCategoryOfLabel(label,cfg.path_dataset.path_to_dataset_categories);
        std::cout << "[Prediction] Label predicted : " << label << " corresponding to " << name_category << std::endl;
        
        std::cout << "[Info] Saving results to : " << output_path_result << " ... " << std::flush;
        std::ofstream file;
        file.open(output_path_result);
        file << label << "\n";
        file << name_category;
        file.close();
        std::cout << "Completed" << std::endl;
        
        //Remove tmp_descriptor scale
         if (boost::filesystem::exists(tmp_desc_scale)){
             boost::filesystem::remove(tmp_desc_scale);
         }
        
        svm_free_and_destroy_model (&model);
        
    }
    else {
        std::cerr << "[ERROR] Descriptor type no valid : " << type_descriptor << std::endl;
        return (-1);
    }
    
    return(0);
}

