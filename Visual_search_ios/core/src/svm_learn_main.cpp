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

/**
 
 Ce programme entraîne un SVM à partir d'un fichier contenant l'ensemble des path de descripteurs. Ce fichier doit être dans le format suivant :
 category_1
 descriptor_object_1
  descriptor_object_2
 ...
 descriptor_object_j
 category_i
 
 Les paramètres du training sont à changer dans le fichier de configuration
 
 **/

void generateTraining(const std::string descriptor_file,const std::string pathToSave){
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
        
        std::vector<std::string> descriptors; // Will contain the different descriptors
        if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors)) < 0)
            continue;
        //For each descriptor
        while (!descriptors.empty ())
        {
            
            
            
            std::ifstream fs;
            float feature_value;
            int index = 1;
            //Returns a reference to the last descriptor in the vector.
            std::string desc = descriptors.back ();
            std::cout << "Processing : " << desc<< std::endl;
            //Removes the last element in the vector, effectively reducing the container size by one.
            descriptors.pop_back ();
            
            
            fs.open (desc.c_str (), std::ios::in);
            string line;
            int count = 0 ;
            if(!fs.is_open())
            {
                cout<<"\n Cannot open the text.txt file";
            }
            else
            {
                std::string string_descriptor;
                while (!fs.eof ())
                {
                    count++;
                    getline( fs , line);
                    if(count < 12)
                        continue;
                    else{
                        if (line != ""){
                            string_descriptor = line;
                            
                        }
                    }
                }
                
                //Put each number of the vector string descriptor to an array
                std::vector<std::string> vector_descriptor = Utils::split(string_descriptor,' ');
                std::cout << "Number features in vector : " << vector_descriptor.size() <<std::endl;
                file << label;
                for (int i = 0; i < vector_descriptor.size(); i++){
                    feature_value = std::stof(vector_descriptor.at(i));
                    file << " " << index <<":"<<feature_value;
                    
                    ++index;
                    
                }
                file << "\n";
            }
            
        }
        // Update the label when changing category
        
        ++label;
        std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
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
    std::cout << "*                         SVM learning                                    *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " config (config file cfg format) descriptor_file_path (descriptor file which contains the path to every descriptor. One line per data)" << std::endl << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    std::cout << "     -scale :                     (1/0) Scale or not training data." << std::endl;
    
}


int main(int argc, char** argv)
{
    //If not enough parameters
    if (argc < 3)
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
    
    std::string configuration_file = argv[1]; // Configuration file to take in account the parameters
    std::string descriptor_file = argv[2]; // txt file which contains the path to the descriptors
    std::vector<std::string> categories; // will contain the different categorie of objects
    
    std::vector<std::vector<svm_node> > data;//Handle svm
    std::vector<int> labels;
    int ret_val, label = 1;
    
    Config cfg;
    
    //Get parameters from the configuration file
    if (getConfiguration (configuration_file, cfg) < 0)
        return (-1);
    //Get the category
    if (Utils::getCategories (descriptor_file, categories) < 0)
        return (-1);
    
    std::string save_dir = cfg.classification.svm.path_to_svm; // folder where to save the svm results
    //Check if the save directory exist
    if (!boost::filesystem::exists(save_dir)){
        std::cerr << "[ERROR] The result directory does not exist - Please create one "<< std::endl;
        exit(0);
    }

    
    if (cfg.classification.scaling.scale_data){
        double lower = cfg.classification.scaling.lower_limit;
        double upper = cfg.classification.scaling.upper_limit;
        std::string path_training_data_scale = save_dir + "model_train.scale";
        std::string path_training_data_range = save_dir +"model_train.range";
        
        //First generate the training data
        std::string path_to_training_data = save_dir +"model_train.txt";
        if (boost::filesystem::exists(path_to_training_data)){
            std::cerr << "[INFO] The training data already exist - Just going to scale the data "<< std::endl;
            std::cout << "Check the following path : " << path_to_training_data << std::endl;
        }else {
            generateTraining(descriptor_file,path_to_training_data);
        }
        if (boost::filesystem::exists(path_training_data_scale)){
            std::cerr << "[INFO] The training data has already been scaled - Just going to train the SVM "<< std::endl;
            std::cout << "Check the following path : " << path_training_data_scale << std::endl;
        }else {
            char command[100];
            sprintf (command, "./svm-scale -r %s %s -l %f -u %f >> %s", path_training_data_range.c_str(), path_to_training_data.c_str(),lower,upper,path_training_data_scale.c_str());
            // system call
            system(command);
         //   scale(path_to_training_data,path_training_data_scale,path_training_data_range,false,lower,upper);
        }
        
        //Read the scaled file and perform the training
        std::ifstream fs;
        fs.open (path_training_data_scale.c_str (), std::ios::in);
        string line;
        int count = 0 ;
        if(!fs.is_open())
        {
            cout<<"\n Cannot open the text.txt file";
        }
        else
        {
            int count_data = 0;
            std::cout << "File open " << std::endl;
            //Vector which will contain all the vector descriptor for every descriptor
            std::vector<std::string> all_vector_descriptor;
            while (!fs.eof ())
            {
                std::vector<svm_node> nodes;
                svm_node node;
                double value;
                int index = 1;
                
                count++;
                //Loop over each line which correspond to one descriptor of an object
                getline( fs , line);
                // the structure is like the following label feature_id_1:feature_value_1 feature_id_2:feature_value_2
                //So we take the first caracter which is the label and then we take the left content which will be the feature vector
                std::vector<std::string> tmp = Utils::split(line,' ');
                if (!tmp.empty()){
                    int label = std::stoi(tmp.at(0));
                    line.erase (line.begin(),line.begin()+1);
                    all_vector_descriptor.push_back(line);
                    //Loop over the feature descriptor without the label element (which is the first one, position 0)
                    
                    for (int j=1; j < tmp.size() ; j++){
                        //SVM DATA
                        node.index = index;
                        value = std::stod(tmp.at(j));
                        
                        std::size_t pos = tmp.at(j).find(":");      // position of "live" in str
                        std::string feature_value = tmp.at(j).substr (pos+1);     // get from "live" to the end
                        // std::cout << "value : " << std::stof(feature_value) << std::endl;
                        node.value =std::stof(feature_value);
                        nodes.push_back (node);
                        //std::cout << "Index " << index << "Value " << node.value << std::endl;
                        ++index;
                    }
                    // Terminate the feature vector
                    node.index = -1;
                    nodes.push_back (node);
                    data.push_back (nodes);
                    labels.push_back (label);
                    count_data++;
                }
            }
            
            std::cout << "[INFO] Number data : " << data.size() << std::endl;
            std::cout << "[INFO] Number labels : " << labels.size() << std::endl;
            
            fs.close();
            
            
            std::cout << "[INFO] TRAINING... : " << std::endl;
            std::string name_model = save_dir + "model_svm.model";
            double gamma = cfg.classification.svm.gamma;
            double C = cfg.classification.svm.C;
            float eps = cfg.classification.svm.eps;
            svmTrain (gamma,C, eps, data, labels,name_model);
            std::cout << "[INFO] Number of descriptors trained : " << std::to_string(count_data) << std::endl;
        }
        
        
    }else {
        int count_data = 0;
        //For each caegory
        for (size_t i = 0; i < categories.size(); ++i)
        {
            
            std::vector<std::string> descriptors; // Will contain the different descriptors
            if ((ret_val = Utils::getDataFilename (descriptor_file, categories.at(i), descriptors)) < 0)
                continue;
            
            //For every descriptors of the current category
            while (!descriptors.empty ())
            {
                
                std::ifstream fs;
                svm_node node;
                std::vector<svm_node> nodes;
                double value;
                int index = 1;
                //Returns a reference to the last descriptor in the vector.
                std::string desc = descriptors.back ();
                std::cout << "Processing : " << desc<< std::endl;
                //Removes the last element in the vector, effectively reducing the container size by one.
                descriptors.pop_back ();
                
                
                fs.open (desc.c_str (), std::ios::in);
                string line;
                int count = 0 ;
                if(!fs.is_open())
                {
                    cout<<"\n Cannot open the text.txt file";
                }
                else
                {
                    std::string string_descriptor;
                    while (!fs.eof ())
                    {
                        count++;
                        getline( fs , line);
                        if(count < 12)
                            continue;
                        else{
                            if (line != ""){
                                string_descriptor = line;
                                
                            }
                        }
                    }
                    
                    //Put each number of the vector string descriptor to an array
                    std::vector<std::string> vector_descriptor = Utils::split(string_descriptor,' ');
                    std::cout << "Number features in vector : " << vector_descriptor.size() <<std::endl;
                    for (int i = 0; i < vector_descriptor.size(); i++){
                        //std::cout << "Label : " <<label << std::endl;
                        //std::cout << "Index : " <<index << std::endl;
                        
                        node.index = index;
                        value = std::stof(vector_descriptor.at(i));
                        //std::cout << "Value : " << value << std::endl;
                        node.value = value;
                        nodes.push_back (node);
                        //std::cout << "[INFO] Node index :  " << index << std::endl;
                        //std::cout << "[INFO] Node value :  " << node.value << std::endl;
                        ++index;
                        
                    }
                }
                // Terminate the feature vector
                node.index = -1;
                nodes.push_back (node);
                data.push_back (nodes);
                labels.push_back (label);
                fs.close ();
                
                count_data++;
                std::cout << "[INFO] Number nodes : " << nodes.size() << std::endl;
                
            }
            // Update the label when changing category
            
            ++label;
            std::cout << "[INFO] Category changed. Label changed. Number is now :  " << label << std::endl;
            
            std::cout << "[INFO] Number data : " << data.size() << std::endl;
            std::cout << "[INFO] Number labels : " << labels.size() << std::endl;
            
            
        }
        std::cout << "[INFO] TRAINING... : " << std::endl;
        std::string name_model = save_dir + "model_svm.model";
        double gamma = cfg.classification.svm.gamma;
        double C = cfg.classification.svm.C;
        float eps = cfg.classification.svm.eps;
        svmTrain (gamma, C, eps, data, labels,name_model);
        std::cout << "[INFO] Number of descriptors trained : " << std::to_string(count_data) << std::endl;
        
    }
}


