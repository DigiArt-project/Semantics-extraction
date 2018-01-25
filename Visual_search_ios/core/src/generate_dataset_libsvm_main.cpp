#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <others/typedefs.hpp>

#include "others/utils.hpp"
#include "others/Timer.hpp"
#include "configuration/configuration.hpp"
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>

#include <svm/svm_train.hpp>
#include <svm/svm_scale.hpp>

#include <boost/algorithm/string.hpp>

std::string descriptor_type = "";

/**
 
 A partir d'un fichier comprenenant le path de l'ensemble des descriptors associés à leur catégories respectifs, on rassemble tout le contenu des descriptors dans un seul et même fichier (training ou testing) qui aura le format adéquat pour libsvm c'est à dire :
 label feature_id1:feature_value1 etc
 Le fichier descriptor_file_path doit être dans le format suivant
 category_i
 path_descriptor_objet_j
 avec i le numéro de la catégorie et j le numéro du descriptor associé à l'objet de la catégorie
 C'est le cas où on a plusieurs catégories au total
 **/

void generate_libsvmdata(const std::string descriptor_file,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    int label = 1;
    int ret_val;
    
    std::vector<std::string> categories;
    //Get the category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category
    for (size_t i = 0; i < categories.size(); ++i)
    {
        
        std::cout << "Category : " << categories.at(i) << std::endl;
        
        std::vector<std::string> descriptors; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors)) < 0)
            continue;
        //For each descriptor
        while (!descriptors.empty ())
        {
            
            float feature_value;
            int index = 1;
            std::ifstream fs;
            //Returns a reference to the last descriptor in the vector.
            std::string desc = descriptors.back ();
            std::cout << "Processing : " << desc<< std::endl;
            //Removes the last element in the vector, effectively reducing the container size by one.
            descriptors.pop_back ();
            
            
            if (descriptor_type.compare("esf") == 0){
                pcl::PointCloud<pcl::ESFSignature640> esf;
                if (pcl::io::loadPCDFile<pcl::ESFSignature640>(desc,esf) != 0)
                {
                    std::cerr << "[ERROR] Error for reading ESF descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    for (int j = 0; j < pcl::ESFSignature640::descriptorSize() ;j++){
                        feature_value = esf.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }else{
                pcl::PointCloud<pcl::VFHSignature308> vfh_type;
                
                if (pcl::io::loadPCDFile<pcl::VFHSignature308>(desc,vfh_type) != 0)
                {
                    std::cerr << "[ERROR] Error for reading VFH/CVFH/OURCVFH descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    for (int j = 0; j < pcl::VFHSignature308::descriptorSize() ;j++){
                        feature_value = vfh_type.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            
        }//End while
        // Update the label when changing category
        ++label;
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        //end for
    }
    file.close();
    std::cout << "File has been saved" << std::endl;
    
}
/**
 C'est le cas où on a deux catégories au total
 A partir d'un fichier contenant le path des descripteurs de chaque categorie (on suppose deux categories au total), on met tous les descripteurs dans un seul fichier avec les label +1 et -1
 **/
void generate_libsvmdata_binary(const std::string descriptor_file,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    string label = "+1";
    int ret_val;
    
    std::vector<std::string> categories;
    //Get the category. should have only 2 categories
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category. Should have only tw ocategories
    for (size_t i = 0; i < categories.size(); ++i)
    {
        
        std::cout << "Category : " << categories.at(i) << std::endl;
        
        std::vector<std::string> descriptors; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors)) < 0)
            continue;
        //For each descriptor
        while (!descriptors.empty ())
        {
            
            float feature_value;
            int index = 1;
            std::ifstream fs;
            //Returns a reference to the last descriptor in the vector.
            std::string desc = descriptors.back ();
            std::cout << "Processing : " << desc<< std::endl;
            //Removes the last element in the vector, effectively reducing the container size by one.
            descriptors.pop_back ();
            
            
            if (descriptor_type.compare("esf") == 0){
                pcl::PointCloud<pcl::ESFSignature640> esf;
                if (pcl::io::loadPCDFile<pcl::ESFSignature640>(desc,esf) != 0)
                {
                    std::cerr << "[ERROR] Error for reading ESF descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    for (int j = 0; j < pcl::ESFSignature640::descriptorSize() ;j++){
                        feature_value = esf.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }else{
                pcl::PointCloud<pcl::VFHSignature308> vfh_type;
                
                if (pcl::io::loadPCDFile<pcl::VFHSignature308>(desc,vfh_type) != 0)
                {
                    std::cerr << "[ERROR] Error for reading VFH/CVFH/OURCVFH descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    for (int j = 0; j < pcl::VFHSignature308::descriptorSize() ;j++){
                        feature_value = vfh_type.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            
        }//End while
        // Update the label when changing category
        label = "-1";
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        //end for
    }
    file.close();
    std::cout << "File has been saved" << std::endl;
    
}
/**
 C'est le cas où on a plusieurs catégories au total mais qu'on souhaite réaliser un training/testing de descripteurs de deux catégories au total : positif et négatif.
 L'utilisateur choisis la catégorie positive et ensuite tout le reste sera négatif
 Si pas de category précisé, on assigne le positive label à la première catégorie
 **/
void generate_libsvmdata_binary_multiplecategories(const std::string descriptor_file,const std::string name_category,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    string label;
    int ret_val;
    bool isPositiveLabel;
    int countDescriptorsPositiveLabel = 0;
    int countDescriptorsNegativeLabel = 0;
    int countCategoriesPositiveLabel = 0;
    int countCategoriesNegativeLabel = 0;
    std::vector<std::string> categories;
    //Get the category. should have multiple category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category. Should have only tw ocategories
    for (size_t i = 0; i < categories.size(); ++i)
    {
        if (name_category.empty()){
            if (i==0){
                label = "+1";
                isPositiveLabel = true;
                countCategoriesPositiveLabel++;
            }else {
                label = "-1";
                isPositiveLabel = false;
                countCategoriesNegativeLabel++;
            }
        }else {
            if (categories.at(i).compare(name_category) == 0){
                label = "+1";
                isPositiveLabel = true;
                countCategoriesPositiveLabel++;
            }else {
                label = "-1";
                isPositiveLabel = false;
                countCategoriesNegativeLabel++;
            }
        }
        std::cout << "Category : " << categories.at(i) << " with label " << label << std::endl;
        std::vector<std::string> descriptors; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors)) < 0)
            continue;
        //For each descriptor
        while (!descriptors.empty ())
        {
            
            float feature_value;
            int index = 1;
            std::ifstream fs;
            //Returns a reference to the last descriptor in the vector.
            std::string desc = descriptors.back ();
            std::cout << "Processing : " << desc<< std::endl;
            //Removes the last element in the vector, effectively reducing the container size by one.
            descriptors.pop_back ();
            
            
            if (descriptor_type.compare("esf") == 0){
                pcl::PointCloud<pcl::ESFSignature640> esf;
                if (pcl::io::loadPCDFile<pcl::ESFSignature640>(desc,esf) != 0)
                {
                    std::cerr << "[ERROR] Error for reading ESF descriptor " << std::endl;
                    exit(1);
                }else {
                    if (isPositiveLabel){
                        countDescriptorsPositiveLabel++;
                    }else {
                        countDescriptorsNegativeLabel++;
                    }
                    
                    file << label;
                    for (int j = 0; j < pcl::ESFSignature640::descriptorSize() ;j++){
                        feature_value = esf.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }else{
                pcl::PointCloud<pcl::VFHSignature308> vfh_type;
                
                if (pcl::io::loadPCDFile<pcl::VFHSignature308>(desc,vfh_type) != 0)
                {
                    std::cerr << "[ERROR] Error for reading VFH/CVFH/OURCVFH descriptor " << std::endl;
                    exit(1);
                }else {
                    if (isPositiveLabel){
                        countDescriptorsPositiveLabel++;
                    }else {
                        countDescriptorsNegativeLabel++;
                    }
                    file << label;
                    for (int j = 0; j < pcl::VFHSignature308::descriptorSize() ;j++){
                        feature_value = vfh_type.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            
        }//End while
        // Update the label when changing category
        label = "-1";
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        //end for
    }
    file.close();
    std::cout << "File has been saved" << std::endl;
    std::cout << "[INFO] Number positive descriptors :  " << countDescriptorsPositiveLabel << std::endl;
    std::cout << "[INFO] Number Negative descriptors :  " << countDescriptorsNegativeLabel << std::endl;
    std::cout << "[INFO] Number positive categories :  " << countCategoriesPositiveLabel << std::endl;
    std::cout << "[INFO] Number negative categories :  " << countCategoriesNegativeLabel << std::endl;
    
    
}


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Generate libsvm type dataset                    *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " descriptor_file_path (descriptor file which contains the path to every descriptor. One line per data) output(txt file) type_descriptor (esf/vfh/ourcvfh/cvfh) multiclass (1/0)" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -posCategory:           choose the positive category you want to assign label +1 (only for binary)." << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}


int main(int argc, char** argv)
{
    //If not enough parameters
    if (argc < 4)
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
    std::cout << "argc = " << argc << std::endl;
    for(int i = 0; i < argc; i++)
        std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
    
    std::string positiveLabelCategory;
    std::string descriptor_file = argv[1]; // txt file which contains the path to the descriptors
    std::string output = argv[2]; // txt file where to save the result
    std::string type_descriptor = argv[3]; // type of descriptor (esf/cvfh/ourcvfh/vfh)
    string multiclass = argv[4]; // if we want to handle binary class (label +1 and -1)  or multiclass (1 2 ...)
    std::vector<std::string> categories; // will contain the different categorie of objects
    std::cout << "[Type descriptor] Info descriptor : " << type_descriptor << std::endl;
    std::cout << "[Info] multiclass  : " << multiclass << std::endl;
    //Binary class
    if (multiclass.compare("0") == 0){
        std::cout << "INFO : bnary class " << std::endl;
        if (argc < 7){
            std::cout << "\n[Error] You need to choose the category for positive label by using -posCategory " << std::endl;
        }else {
            if(pcl::console::parse_argument(argc, argv, "-posCategory", positiveLabelCategory) == -1)
            {
                std::cerr<<"[Error] You need to choose the category for positive label by using -posCategory"<<std::endl;
                return -1;
            }
            
        }
    }
    
    if (!type_descriptor.empty() || type_descriptor.compare("vfh") == 0 || type_descriptor.compare("esf") == 0 || type_descriptor.compare("cvfh") == 0 || type_descriptor.compare("ourcvfh") == 0){
        
        
        descriptor_type = type_descriptor;
        
        //First generate the training data
        if (boost::filesystem::exists(output)){
            std::cerr << "[INFO] The training datalibsvm already exist"<< std::endl;
            std::cout << "Check the following path : " << output << std::endl;
        }else {
            if (multiclass == "1"){
                generate_libsvmdata(descriptor_file,output);
            }else {
                //generate_libsvmdata_binary(descriptor_file,output);
                generate_libsvmdata_binary_multiplecategories(descriptor_file,positiveLabelCategory, output);
            }
            
        }
    }else {
        std::cerr << "Please add correct descriptor type (esf/vfh/cvfh/ourcvfh)"<< std::endl;
    }
    
    
    return 0;
    
}


