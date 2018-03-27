#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/surface/mls.h>

// Boost headers
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>
//Configuration
#include "Configuration/configuration.hpp"
#include "Retrieval/retrieval_similarity_search.hpp"

#include <stdlib.h>
#include <cstdlib>

bool runEvaluation = false;
int k = 20;
float leaf_resolution = 0.01f;
bool enable_resolution = true;
bool enable_scaling = false;
bool compute_on_full = true;
std::string descriptor_type = "esf";
std::string output_path_result = "../results.json";



void getCategories(std::string path_to_data, std::vector<std::string>& categories){
    
    for(boost::filesystem::directory_iterator it(path_to_data); it!=boost::filesystem::directory_iterator(); ++it)
    {
        std::string path_current = it->path().c_str();
        boost::filesystem::path p(path_current);
        std::string filename = p.filename().c_str();
        if (boost::filesystem::is_directory(path_current) && filename.find("training_") == string::npos){
            categories.push_back(filename);
        }
    }
    
}

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                 Cloud Retrieval using similarity search                 *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " -query object.pcd -trained tained_dataset [Options]" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    std::cout << "     -descriptor:            change descriptor. Available : esf, vfh, cvfh, ourcvfh, gshot, pointnet, gshotPyramid usc, grsd. Per default : esf " << std::endl;
    std::cout << "     -output:                where to save the json results file. Per default : ../results.json" << std::endl;
    std::cout << "     -leaf_resolution:       For cloud resolution invariance,. Per defaut : 0.01 \n" << std::endl;
    std::cout << "     -compute_full:          Use full objects or views,. Per defaut : true \n" << std::endl;
    std::cout << "     -k:                     number of results to find. Per defaut : 10 \n" << std::endl;
    
}


struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};
void read_directory(const std::string& name, std::vector<std::string>& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
}


void runEvaluationSimilaritySearch(){
    std::vector<std::string> descriptors = {"esf"};
    bool resolution_activated = enable_resolution;
    bool scale_activated = enable_scaling;

    std::string database_path = "../../../Datasets/Dataset_structuresensor/";
    std::string database_trained_path = "../../cat10_views_descriptors/";
    std::string directory_to_save_result = database_trained_path + "results_similarity/";
    if (!boost::filesystem::exists(directory_to_save_result)){
        boost::filesystem::create_directory(directory_to_save_result);
    }
    
    std::vector<std::string> categories;
    if (!boost::filesystem::exists (database_trained_path)){
        std::cerr<<"The directory  " << database_trained_path << " is not valid." <<std::endl;
        exit(1);
    }
    getCategories(database_trained_path,categories);
    
    for (int j = 0; j < descriptors.size(); j++ ){
        double average_all_k1 = 0, average_all_k5 = 0, average_all_k10 = 0, average_all_k15 = 0, average_all_k20 = 0;
        double sum_all_k1 = 0, sum_all_k5 = 0, sum_all_k10 = 0, sum_all_k15 = 0, sum_all_k20 = 0;
        int count_object = 0;
        ofstream file_result;
        std::string name_file = descriptors.at(j) + "_similaritysearch.txt";
        std::string path_to_save_result = directory_to_save_result + name_file;
        file_result.open (path_to_save_result);
        file_result << "Average for k=1, k= 5, k = 10, k = 15, k = 20 and all k together \n" << std::endl;
        
        std::cout << "\n [INFO] Current descriptor : " << descriptors.at(j) << std::endl;
        for(boost::filesystem::directory_iterator it(database_path); it != boost::filesystem::directory_iterator(); ++it)
        {
            std::string directory_path = it->path().c_str();
            
            std::vector<std::string> string_split;
            boost::split(string_split, directory_path, boost::is_any_of("/"));
            std::string name_category = string_split.at(5);
           
            if (boost::filesystem::is_directory(directory_path))
            {
             std::cout << "[Category] " << name_category <<  std::endl;
                int nb_random_object = 3;
                std::vector<std::string> cloud_file;
                read_directory(directory_path, cloud_file);
                //std::copy(cloud_file.begin(), cloud_file.end());
                
                double sum_precision_category = 0;
                std::vector<std::map<int,double>> precision_object_per_category_map;
                while (nb_random_object > 0){
                    int r = rand()%(cloud_file.size()) + 0;
                    string extension = boost::filesystem::extension(cloud_file.at(r));
                    if (extension == ".pcd"){
                        nb_random_object --;
                        std::string query = directory_path + "/" + cloud_file.at(r);
                        std::cout << "Random object : " << query << std::endl;
                        cloud_file.erase(cloud_file.begin() + r);
                        
                        std::string descriptor_type = descriptors.at(j);
                        
                        if (descriptor_type.compare("vfh") == 0){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::L1<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path, categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
                            
                        }else if (descriptor_type.compare("cvfh") == 0){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
                            
                        }else if (descriptor_type.compare("ourcvfh") == 0){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
                            
                            
                        }else if (descriptor_type.compare("esf") == 0){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L1<float> > db;//L1 normalement
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
                            
                        }
                        else if (descriptor_type.compare("good") == 0 || descriptor_type.compare("pointnet") == 0 ){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L2<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
                            
                        }
                        else if (descriptor_type.compare("gshot") == 0 || descriptor_type.compare("gshotPyramid") == 0 ){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::SHOT352, flann::L2<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                            //sum_precision_category = sum_precision_category + average_precision;
            
                        }
                        else if (descriptor_type.compare("grsd") == 0){
                            
                            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::GRSDSignature21, flann::ChiSquareDistance<float> > db;
                            db.setDescriptor(descriptor_type);
                            if (resolution_activated){
                                db.setResolutionMode(true);
                                db.setResolution(leaf_resolution);
                            }
                            if (scale_activated){
                                db.setScale(scale_activated);
                            }
                            
                            db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                            precision_object_per_category_map.push_back(db.get_precisions_values());
                           // sum_precision_category = sum_precision_category + average_precision;
                            
                        }
                        else {
                            std::cerr<<"Unknwon descriptor type"<<std::endl;
                            exit(1);
                        }
                    }
                }
                //Compute average precision for each K
                double average_k1 = 0, average_k5 = 0, average_k10 = 0, average_k15 = 0, average_k20 = 0;
                double sum_k1 = 0, sum_k5 = 0, sum_k10 = 0, sum_k15 = 0, sum_k20 = 0;
                double average_precision_category = 0;
                int size_vector = precision_object_per_category_map.size();
                cout << "Vector size: " << size_vector << endl;
                //For each object which contain multiple value of precision for K
                for (int i = 0; i < size_vector; i ++){
                    //For each k
                    sum_k1 += precision_object_per_category_map.at(i)[1];
                    sum_k5 += precision_object_per_category_map.at(i)[5];
                    sum_k10 += precision_object_per_category_map.at(i)[10];
                    sum_k15 += precision_object_per_category_map.at(i)[15];
                    sum_k20 += precision_object_per_category_map.at(i)[20];
                    
                }
                average_k1 = sum_k1/size_vector;
                average_k5 = sum_k5/size_vector;
                average_k10 = sum_k10/size_vector;
                average_k15 = sum_k15/size_vector;
                average_k20 = sum_k20/size_vector;
                sum_precision_category = average_k1 + average_k5 + average_k10 + average_k15 + average_k20;
                average_precision_category = sum_precision_category/5;
                
                std::cout << "Average k1 :" << average_k1 << std::endl;
                std::cout << "Average k5 :" << average_k5 << std::endl;
                std::cout << "Average k10 :" << average_k10 << std::endl;
                std::cout << "Average k15 :" << average_k15 << std::endl;
                std::cout << "Average k20 :" << average_k20 << std::endl;
                std::cout << "Average Precision for all k  : " << average_precision_category << " % " << std::endl;
                
                //Save to file
                //file_result << name_category << std::endl;
                file_result << name_category << " " << average_k1 << " " << average_k5 << " " << average_k10 << " " << average_k15 << " " << average_k20 << " " << average_precision_category<<  std::endl;
                /*file_result << average_k5 << std::endl;
                file_result << average_k10 << std::endl;
                file_result << average_k15 << std::endl;
                file_result << average_k20 << std::endl;*/
                //file_result << average_precision_category << std::endl;
                file_result << "\n";
                
                name_file = "";

                
                sum_all_k1 += average_k1;
                sum_all_k5 += average_k5;
                sum_all_k10 += average_k10;
                sum_all_k15 += average_k15;
                sum_all_k20 += average_k20;
                
                count_object++;
            }
            
        }
        average_all_k1 = sum_all_k1/count_object;
        average_all_k5 = sum_all_k5/count_object;
        average_all_k10 = sum_all_k10/count_object;
        average_all_k15 = sum_all_k15/count_object;
        average_all_k20 = sum_all_k20/count_object;
        std::cout << "\n" << std::endl;
        std::cout << "All Average k1 :" << average_all_k1 << std::endl;
        std::cout << "All Average k5 :" << average_all_k5 << std::endl;
        std::cout << "All Average k10 :" << average_all_k10 << std::endl;
        std::cout << "All Average k15 :" << average_all_k15 << std::endl;
        std::cout << "All Average k20 :" << average_all_k20 << std::endl;
        
    }
    
}



void runEvaluationSimilaritySearchViews(){
    std::vector<std::string> descriptors = {"esf"};
    bool resolution_activated = enable_resolution;
    bool scale_activated = enable_scaling;
    
    std::string database_path = "../../structuresensor_views_descriptors/";
    std::string database_trained_path = "../../cat10_views_descriptors/";
    std::string directory_to_save_result = database_trained_path + "results_similarity/";
    if (!boost::filesystem::exists(directory_to_save_result)){
        boost::filesystem::create_directory(directory_to_save_result);
    }
    
    std::vector<std::string> categories;
    if (!boost::filesystem::exists (database_trained_path)){
        std::cerr<<"The directory  " << database_trained_path << " is not valid." <<std::endl;
        exit(1);
    }
    getCategories(database_trained_path,categories);
    std::string name_category = "";
   
    for (int j = 0; j < descriptors.size(); j++ ){
        double average_all_k1 = 0, average_all_k5 = 0, average_all_k10 = 0, average_all_k15 = 0, average_all_k20 = 0;
        double sum_all_k1 = 0, sum_all_k5 = 0, sum_all_k10 = 0, sum_all_k15 = 0, sum_all_k20 = 0;
        int count_object = 0;
        
        ofstream file_result;
        std::string name_file = descriptors.at(j) + "_views_similaritysearch.txt";
        std::string path_to_save_result = directory_to_save_result + name_file;
        file_result.open (path_to_save_result);
        file_result << "Average for k=1, k= 5, k = 10, k = 15, k = 20 and all k together \n" << std::endl;
        
        std::cout << "\n [INFO] Current descriptor : " << descriptors.at(j) << std::endl;
        for(boost::filesystem::recursive_directory_iterator it(database_path); it!=boost::filesystem::recursive_directory_iterator(); ++it)
        {
             std::string directory_path = it->path().c_str();
            if (boost::filesystem::is_directory(directory_path))
            {
                
                if (directory_path.find("/descriptors") == std::string::npos && directory_path.find("/views") == std::string::npos && directory_path.find("/training_") == std::string::npos
                    && directory_path.find("/results_similarity") == std::string::npos){
                     std::size_t found = directory_path.find_last_of("/\\");
                    name_category = directory_path.substr(found+1);
                    
                    std::cout << "[Category] " << name_category <<  std::endl;
                  
                }
                if (directory_path.find("/views") != std::string::npos) {
                    int nb_random_object = 3;
                    std::vector<std::string> cloud_file;
                    read_directory(directory_path, cloud_file);
                     std::cout << "View " << directory_path <<  std::endl;
                    double sum_precision_category = 0;
                    std::vector<std::map<int,double>> precision_object_per_category_map;
                    while (nb_random_object > 0){
                        int r = rand()%(cloud_file.size()) + 0;
                        string extension = boost::filesystem::extension(cloud_file.at(r));
                        if (extension == ".pcd"){
                            nb_random_object --;
                            std::string query = directory_path + "/" + cloud_file.at(r);
                            std::cout << "Random object : " << query << std::endl;
                            cloud_file.erase(cloud_file.begin() + r);
                            
                            std::string descriptor_type = descriptors.at(j);
                            
                            if (descriptor_type.compare("vfh") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::L1<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path, categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                            }else if (descriptor_type.compare("cvfh") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                            }else if (descriptor_type.compare("ourcvfh") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                                
                            }else if (descriptor_type.compare("esf") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L1<float> > db;//L1 normalement
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                            }
                            else if (descriptor_type.compare("good") == 0 || descriptor_type.compare("pointnet") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L2<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                            }
                            else if (descriptor_type.compare("gshot") == 0 || descriptor_type.compare("gshotPyramid") == 0 ){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::SHOT352, flann::L2<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                //sum_precision_category = sum_precision_category + average_precision;
                                
                            }
                            else if (descriptor_type.compare("grsd") == 0){
                                
                                RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::GRSDSignature21, flann::ChiSquareDistance<float> > db;
                                db.setDescriptor(descriptor_type);
                                if (resolution_activated){
                                    db.setResolutionMode(true);
                                    db.setResolution(leaf_resolution);
                                }
                                if (scale_activated){
                                    db.setScale(scale_activated);
                                }
                                
                                db.globalMatchingEvaluation(query,database_trained_path,categories,k);
                                precision_object_per_category_map.push_back(db.get_precisions_values());
                                // sum_precision_category = sum_precision_category + average_precision;
                                
                            }
                            else {
                                std::cerr<<"Unknwon descriptor type"<<std::endl;
                                exit(1);
                            }
                        }
                    }
                    //Compute average precision for each K
                    double average_k1 = 0, average_k5 = 0, average_k10 = 0, average_k15 = 0, average_k20 = 0;
                    double sum_k1 = 0, sum_k5 = 0, sum_k10 = 0, sum_k15 = 0, sum_k20 = 0;
                    double average_precision_category = 0;
                    int size_vector = precision_object_per_category_map.size();
                    cout << "Vector size: " << size_vector << endl;
                    //For each object which contain multiple value of precision for K
                    for (int i = 0; i < size_vector; i ++){
                        //For each k
                        sum_k1 += precision_object_per_category_map.at(i)[1];
                        sum_k5 += precision_object_per_category_map.at(i)[5];
                        sum_k10 += precision_object_per_category_map.at(i)[10];
                        sum_k15 += precision_object_per_category_map.at(i)[15];
                        sum_k20 += precision_object_per_category_map.at(i)[20];
                        
                    }
                    average_k1 = sum_k1/size_vector;
                    average_k5 = sum_k5/size_vector;
                    average_k10 = sum_k10/size_vector;
                    average_k15 = sum_k15/size_vector;
                    average_k20 = sum_k20/size_vector;
                    sum_precision_category = average_k1 + average_k5 + average_k10 + average_k15 + average_k20;
                    average_precision_category = sum_precision_category/5;
                    
                    std::cout << "Average k1 :" << average_k1 << std::endl;
                    std::cout << "Average k5 :" << average_k5 << std::endl;
                    std::cout << "Average k10 :" << average_k10 << std::endl;
                    std::cout << "Average k15 :" << average_k15 << std::endl;
                    std::cout << "Average k20 :" << average_k20 << std::endl;
                    std::cout << "Average Precision for all k  : " << average_precision_category << " % " << std::endl;
                    
                    //Save to file
                    //file_result << name_category << std::endl;
                    file_result << name_category << " " << average_k1 << " " << average_k5 << " " << average_k10 << " " << average_k15 << " " << average_k20 << " " << average_precision_category<<  std::endl;
                    
                    file_result << "\n";
                    
                    name_file = "";
                    
                    sum_all_k1 += average_k1;
                    sum_all_k5 += average_k5;
                    sum_all_k10 += average_k10;
                    sum_all_k15 += average_k15;
                    sum_all_k20 += average_k20;
                    
                    count_object++;
                    
                 
                }
                
                
            }

        }
        average_all_k1 = sum_all_k1/count_object;
        average_all_k5 = sum_all_k5/count_object;
        average_all_k10 = sum_all_k10/count_object;
        average_all_k15 = sum_all_k15/count_object;
        average_all_k20 = sum_all_k20/count_object;
        std::cout << "\n" << std::endl;
        std::cout << "All Average k1 :" << average_all_k1 << std::endl;
        std::cout << "All Average k5 :" << average_all_k5 << std::endl;
        std::cout << "All Average k10 :" << average_all_k10 << std::endl;
        std::cout << "All Average k15 :" << average_all_k15 << std::endl;
        std::cout << "All Average k20 :" << average_all_k20 << std::endl;
        
    }
    
    
    
}


int main(int argc, char** argv)
{
    
    if (runEvaluation){
        runEvaluationSimilaritySearch();
        //runEvaluationSimilaritySearchViews();
    }else {
        //showHelp (argv[0]);
        
        //Show help
        if (pcl::console::find_switch (argc, argv, "-h"))
        {
            showHelp (argv[0]);
            exit (0);
        }
        //If not enough parameters
        if (argc < 3)
        {
            std::cerr << "Not enough aguments " << std::endl;
            showHelp (argv[0]);
            return (-1);
        }
        
        
        
        bool resolution_activated = enable_resolution;
        bool scale_activated = enable_scaling;
        
        std::string query;
        if(pcl::console::parse_argument(argc, argv, "-query", query) == -1)
        {
            std::cerr<<"Please specify the query (.pcd file)"<<std::endl;
            return -1;
        }
        std::string database_trained;
        if(pcl::console::parse_argument(argc, argv, "-trained", database_trained) == -1)
        {
            std::cerr<<"Please specify the trained database "<<std::endl;
            return -1;
        }
        
        std::vector<std::string> categories;
        if (!boost::filesystem::exists (database_trained)){
            std::cerr<<"The directory  " << database_trained << " is not valid." <<std::endl;
            return -1;
        }
        getCategories(database_trained,categories);
        
        
        for (int i = 0;i< categories.size();i++){
            std::cout << "[Categories available] : " << categories.at(i) << std::endl;
        }
        
        
        
        pcl::console::parse_argument (argc, argv, "-descriptor", descriptor_type);
        pcl::console::parse_argument (argc, argv, "-output", output_path_result);
        pcl::console::parse_argument (argc, argv, "-k", k);
        pcl::console::parse_argument (argc, argv, "-leaf_resolution", leaf_resolution);
        std::string activate ="";
        pcl::console::parse_argument (argc, argv, "-compute_full", activate);
        
        if (activate == "true"){
            compute_on_full = true;
        }else{
            compute_on_full = false;
        }
        
        std::cout << compute_on_full << std::endl;

        if (!boost::filesystem::exists (query)){
            std::cerr<<"[ERROR] query empty - Please specify the query again using -query "<<std::endl;
            return -1;
        }
        std::cout << "[INFO] Trained dataset path : " <<database_trained << std::endl;
        
        /*Timer timetotal;
         timetotal.startTimer();*/
        
        if (descriptor_type.compare("vfh") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::L1<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained, categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
        }else if (descriptor_type.compare("cvfh") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }else if (descriptor_type.compare("ourcvfh") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::VFHSignature308, flann::ChiSquareDistance<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }else if (descriptor_type.compare("esf") == 0 || descriptor_type.compare("esfvfh") == 0 ){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L1<float> > db;//L1 normalement
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }
        else if (descriptor_type.compare("good") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L2<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }
        else if (descriptor_type.compare("pointnet") == 0 || descriptor_type.compare("pointnet_modelnet") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::ESFSignature640, flann::L1<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }
        else if (descriptor_type.compare("gshot") == 0 || descriptor_type.compare("gshotPyramid") == 0 ){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::SHOT352, flann::L2<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }
        else if (descriptor_type.compare("grsd") == 0){
            
            RetrievalSimilaritySearchDatabase<pcl::PointXYZ, pcl::GRSDSignature21, flann::ChiSquareDistance<float> > db;
            db.setDescriptor(descriptor_type);
            db.setComputeOnFull(compute_on_full);
            if (resolution_activated){
                db.setResolutionMode(true);
                db.setResolution(leaf_resolution);
            }
            if (scale_activated){
                db.setScale(scale_activated);
            }
            
            db.globalMatchingUsingViewOfCloud(query,database_trained,categories,k);
            db.saveResultsDetectionsJSON(output_path_result);
            
            
        }
        else {
            std::cerr<<"Unknwon descriptor type"<<std::endl;
            return -1;
        }
        /*timetotal.stopTimer();
         timetotal.getTime();*/
        
    }
    
    return 0;
}
