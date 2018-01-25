// STL
#include <iostream>
#include <fstream>
#include <vector>

//NANOFLANN
//#include <nanoflann.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include "Configuration/configuration.hpp"
//BOOST
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <boost/algorithm/string.hpp>

#include "others/utils.hpp"

using namespace std;

std::string descriptor_type = "";


void generate_libsvmdata_txt(const std::string descriptor_file,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    int label = 1;
    int ret_val;
    
    std::vector<std::string> categories;
    //Get the category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category
    for (size_t i = 0; i < categories.size(); i++)
    {
        std::cout << "Category : " << categories.at(i) << std::endl;
        
        std::vector<std::string> descriptors_path; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors_path)) < 0){
            std::cout << "CONTINUE " << std::endl;
            continue;
        }
        
        //For each descriptor
        for (size_t desc_i = 0; desc_i <descriptors_path.size(); desc_i++){
            
            int index = 1;
            //Returns a reference to the last descriptor in the vector.
            std::string current_desc_path = descriptors_path.at(desc_i);
            std::cout << "Processing : " << current_desc_path << std::endl;
            std::ifstream input(current_desc_path);
            
            file << label;
            std::vector<std::string> descriptors_values;
            
            for( std::string line; getline( input, line ); )
            {
                boost::split(descriptors_values, line, boost::is_any_of(" "), boost::token_compress_on);
                std::cout << "Size descriptors : " << descriptors_values.size() << std::endl;
                for (int v = 0; v < descriptors_values.size() - 1; v++){
                    //std::cout << index << std::endl;
                    file << " " << index <<":"<<descriptors_values.at(v);
                    index++;
                }
            }
            file << "\n";

            
        }//End for each descriptor of current category
        // Update the label when changing category
        ++label;
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        //end for
    }
    file.close();
    std::cout << "File has been saved" << std::endl;

}

void generate_libsvmdata_binary_txt(const std::string descriptor_file,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    string label = "+1";
    int ret_val;
    
    std::vector<std::string> categories;
    //Get the category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category
    for (size_t i = 0; i < categories.size(); i++)
    {
        std::cout << "Category : " << categories.at(i) << std::endl;
        
        std::vector<std::string> descriptors_path; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors_path)) < 0){
            std::cout << "CONTINUE " << std::endl;
            continue;
        }
        
        //For each descriptor
        for (size_t desc_i = 0; desc_i <descriptors_path.size(); desc_i++){
            
            int index = 1;
            //Returns a reference to the last descriptor in the vector.
            std::string current_desc_path = descriptors_path.at(desc_i);
            std::cout << "Processing : " << current_desc_path << std::endl;
            std::ifstream input(current_desc_path);
            
            file << label;
            std::vector<std::string> descriptors_values;
            
            for( std::string line; getline( input, line ); )
            {
                boost::split(descriptors_values, line, boost::is_any_of(" "), boost::token_compress_on);
                for (int v = 0; v < descriptors_values.size() - 1; v++){
                    file << " " << index <<":"<<descriptors_values.at(v);
                    ++index;
                }
            }
            file << "\n";
            
            
        }//End for each descriptor of current category
        // Update the label when changing category
        label = "-1";
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        //end for
    }
    file.close();
    std::cout << "File has been saved" << std::endl;
    
}

/**
 
 A partir d'un fichier comprenenant le path de l'ensemble des descriptors associés à leur catégories respectifs, on rassemble tout le contenu des descriptors dans un seul et même fichier (training ou testing) qui aura le format adéquat pour libsvm c'est à dire :
 label feature_id1:feature_value1 etc
 Le fichier descriptor_file_path doit être dans le format suivant
 category_i
 path_descriptor_objet_j
 avec i le numéro de la catégorie et j le numéro du descriptor associé à l'objet de la catégorie
 
 **/
void generate_libsvmdata(const std::string descriptor_file,const std::string pathToSave){
    std::ofstream file;
    file.open(pathToSave, ios::out | ios::app );
    int label = 1;
    int ret_val;
    int count_number_descriptors_total = 0;
    int count_number_descriptors_category = 0;
    
    std::vector<std::string> categories;
    //Get the category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return;
    //For each category
    std::cout << "Number of categories : " << categories.size() << std::endl;
    for (int cat = 0; cat < categories.size(); cat ++){
        std::cout << categories.at(cat) << std::endl;
    }
    for (size_t i = 0; i < categories.size(); i++)
    {
        std::cout << "[INFO] CATEGORY : " << categories.at(i) << std::endl;
        
        std::vector<std::string> descriptors_path; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors_path)) < 0){
            std::cout << "CONTINUE " << std::endl;
            continue;
        }
         std::cout << "Number of descriptors : " << descriptors_path.size() << std::endl;
        for (int path=0;path < descriptors_path.size(); path++){
            std::cout << descriptors_path.at(path) << std::endl;
        }
        
        
        //For each descriptor
        count_number_descriptors_category = 0;
        for (size_t desc_i = 0; desc_i <descriptors_path.size(); desc_i++){
            
            float feature_value;
            int index = 1;
            std::ifstream fs;
            //Returns a reference to the last descriptor in the vector.
            std::string current_desc_path = descriptors_path.at(desc_i);
            std::cout << "Processing : " << current_desc_path << std::endl;
            //Removes the last element in the vector, effectively reducing the container size by one.
            
            
            if (descriptor_type.compare("esf") == 0 || descriptor_type.compare("esffull") == 0){
                pcl::PointCloud<pcl::ESFSignature640> esf;
                if (pcl::io::loadPCDFile<pcl::ESFSignature640>(current_desc_path,esf) != 0)
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
                
            }
            else if (descriptor_type.compare("grsd") == 0 || descriptor_type.compare("grsdfull") == 0){
                pcl::PointCloud<pcl::GRSDSignature21> grsd;
                if (pcl::io::loadPCDFile<pcl::GRSDSignature21>(current_desc_path,grsd) != 0)
                {
                    std::cerr << "[ERROR] Error for reading GRSD descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    
                    for (int j = 0; j < pcl::GRSDSignature21::descriptorSize() ;j++){
                        feature_value = grsd.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            else if (descriptor_type.compare("gshot") == 0 || descriptor_type.compare("gshotfull") == 0){
                pcl::PointCloud<pcl::SHOT352> gshot;
                if (pcl::io::loadPCDFile<pcl::SHOT352>(current_desc_path,gshot) != 0)
                {
                    std::cerr << "[ERROR] Error for reading GSHOT descriptor " << std::endl;
                    exit(1);
                }else {
                    std::cout << "LABEL : " << label << std::endl;
                    file << label;
                    
                    for (int j = 0; j < pcl::SHOT352::descriptorSize() ;j++){
                        feature_value = gshot.points[0].descriptor[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }

            else if (descriptor_type.compare("sc3D") == 0){
                pcl::PointCloud<pcl::ShapeContext1980> sc3D;
                if (pcl::io::loadPCDFile<pcl::ShapeContext1980>(current_desc_path,sc3D) != 0)
                {
                    std::cerr << "[ERROR] Error for reading GSHOT descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    
                    for (int j = 0; j < pcl::SHOT352::descriptorSize() ;j++){
                        feature_value = sc3D.points[0].descriptor[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            /*else if (descriptor_type.compare("spin") == 0){
                pcl::PointCloud<pcl::Histogram<153>> spin;
                if (pcl::io::loadPCDFile<pcl::Histogram<153>>(current_desc_path,spin) != 0)
                {
                    std::cerr << "[ERROR] Error for reading SPIN descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    
                    for (int j = 0; j < pcl::Histogram<153>::descriptorSize() ;j++){
                        feature_value = spin.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }*/

            else{
                pcl::PointCloud<pcl::VFHSignature308> vfh_type;
                
                if (pcl::io::loadPCDFile<pcl::VFHSignature308>(current_desc_path,vfh_type) != 0)
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
            count_number_descriptors_category++;
            count_number_descriptors_total++;
            
        }//End for each descriptor of current category
        // Update the label when changing category
        ++label;
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
        std::cout << "[TOTAL Descriptors processed for category] :  " << count_number_descriptors_category << std::endl;
        //end for
    }
    file.close();
    std::cout << "[TOTAL Descriptors processed ] :  " << count_number_descriptors_total << std::endl;
    std::cout << "File has been saved" << std::endl;
    
}


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
        
        std::vector<std::string> descriptors_path; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors_path)) < 0)
            continue;
        //For each descriptor
        //For each descriptor
        for (size_t desc_i = 0; desc_i <descriptors_path.size(); desc_i++){
            
            float feature_value;
            int index = 1;
            std::ifstream fs;
            //Returns a reference to the last descriptor in the vector.
            std::string desc = descriptors_path.at(desc_i);
            std::cout << "Processing : " << desc<< std::endl;
            
            
            if (descriptor_type.compare("esf") == 0 || descriptor_type.compare("esffull") == 0){
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
            }
            
            else if (descriptor_type.compare("grsd") == 0 || descriptor_type.compare("grsdfull") == 0){
                pcl::PointCloud<pcl::GRSDSignature21> grsd;
                if (pcl::io::loadPCDFile<pcl::GRSDSignature21>(desc,grsd) != 0)
                {
                    std::cerr << "[ERROR] Error for reading GRSD descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    
                    for (int j = 0; j < pcl::GRSDSignature21::descriptorSize() ;j++){
                        feature_value = grsd.points[0].histogram[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            else if (descriptor_type.compare("gshot") == 0 || descriptor_type.compare("gshotfull") == 0){
                pcl::PointCloud<pcl::SHOT352> gshot;
                if (pcl::io::loadPCDFile<pcl::SHOT352>(desc,gshot) != 0)
                {
                    std::cerr << "[ERROR] Error for reading GSHOT descriptor " << std::endl;
                    exit(1);
                }else {
                    file << label;
                    
                    for (int j = 0; j < pcl::SHOT352::descriptorSize() ;j++){
                        feature_value = gshot.points[0].descriptor[j];
                        file << " " << index <<":"<<feature_value;
                        ++index;
                    }
                    file << "\n";
                    
                }
            }
            else{
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


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                         Generate libsvm type dataset                    *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " descriptor_file_pat (descriptor file which contains the path to every descriptor. One line per data) output (file which will contains the descriptors) type_descriptor (esf/vfh/ourcvfh/cvfh/grsd/gshot/scurv/good/egi/spin/sc3D/pointnet/pointnet_modelnet/esffull/gshotfull/grsdfull/all) multiclass (1/0)" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}


int main(int argc, char** argv)
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
    
    std::string descriptor_file = argv[1]; // txt file which contains the path to the descriptors
    std::string output = argv[2]; // folder where to save the result
    std::string type_descriptor = argv[3]; // type of descriptor (esf/cvfh/ourcvfh/vfh)
    string multiclass = argv[4]; // if we want to handle binary class (label +1 and -1)  or multiclass (1 2 ...)
    std::vector<std::string> categories; // will contain the different categorie of objects
    std::cout << "[Type descriptor] Info descriptor : " << type_descriptor << std::endl;
    std::cout << "[Info] multiclass  : " << multiclass << std::endl;
    
    
    if (type_descriptor.compare("vfh") == 0 || type_descriptor.compare("esf") == 0 || type_descriptor.compare("cvfh") == 0 || type_descriptor.compare("ourcvfh") == 0 || type_descriptor.compare("grsd") == 0 || type_descriptor.compare("gshot") == 0 || type_descriptor.compare("grsdfull") == 0 || type_descriptor.compare("esffull") == 0 || type_descriptor.compare("gshotfull") == 0 ||
        type_descriptor.compare("sc3D") == 0) {
        
        
        descriptor_type = type_descriptor;
        if (multiclass == "1"){
            generate_libsvmdata(descriptor_file,output);
        }else {
            generate_libsvmdata_binary(descriptor_file,output);
        }
    
       
        /*
        //First generate the training data
        if (boost::filesystem::exists(output)){
            std::cerr << "[INFO] The training datalibsvm already exist"<< std::endl;
            std::cout << "Check the following path : " << output << std::endl;
        }else {
            if (multiclass == "1"){
                generate_libsvmdata(descriptor_file,output);
            }else {
                generate_libsvmdata_binary(descriptor_file,output);
            }
            
        }*/
    }
     else if (type_descriptor.compare("scurv") == 0  || type_descriptor.compare("spin") == 0  || type_descriptor.compare("good") == 0
               || type_descriptor.compare("egi") == 0 || type_descriptor.compare("pointnet") == 0 || type_descriptor.compare("pointnet_modelnet") == 0 ){
         std::cout << type_descriptor << std::endl;
         if (multiclass == "1"){
             generate_libsvmdata_txt(descriptor_file,output);
         }else {
             generate_libsvmdata_binary_txt(descriptor_file,output);
         }

     }
    else if (type_descriptor.compare("all") == 0){
         std::cout << "ALL DESCRIPTORS"<< std::endl;
        if (multiclass == "1"){
             generate_libsvmdata_txt(descriptor_file,output);
        }else {
            generate_libsvmdata_binary_txt(descriptor_file,output);
        }
    }
    else if (type_descriptor.empty()){
         std::cerr << "Please add a descriptor"<< std::endl;
    }
    else {
        std::cerr << "Please add correct descriptor type (esf/vfh/cvfh/ourcvfh/grsd/sc3D/spin/egi/good/scurv/pointnet/all)"<< std::endl;
    }
    
    
    return 0;
    
}
