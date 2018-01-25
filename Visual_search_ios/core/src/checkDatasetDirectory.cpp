#include <typedefs.hpp>
#include <pcl/console/parse.h>
//Boost
#include <boost/filesystem.hpp>

/**
 
 Given the path of the directory, check if it exists
 Check as well if the views are already computed and if the descriptors are computed
 
 For example if -data structure-sensor-dataset, the programm will check if structure-sensor-dataset is present in the Datasets Directory and check if trained-structure-sensor-dataset (which contains descriptors and views) is also here
 
 This programm allows to know if we need to generate views from the dataset and compute the 3D descriptors
 
 **/

std::string check = "descriptors";
std::string dataset_categories_file = "dataset_categories.txt";

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                 Check validity of database directory                    *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << " -data dataset -check checking[Options] \n" << std::endl << std::endl;
    std::cout << "     -check:                     (dataset/descriptors/views) check if the dataset exists, if the descriptors or the views has been computed" << std::endl;
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
        exit(-1);
    }
    
    
    std::string dataset;
    if(pcl::console::parse_argument(argc, argv, "-data", dataset) == -1)
    {
        std::cerr<<"Please specify the dataset directory"<<std::endl;
        exit(-1);
    }
    
    if(pcl::console::parse_argument(argc, argv, "-check", check) == -1)
    {
        std::cerr<<"Please specify what to check"<<std::endl;
        exit(-1);
    }
    
    if (check != "descriptors" && check != "dataset" && check != "views"){
        std::cerr<<"[ERROR] Please check the check option"<<std::endl;
        exit(-1);
    }
    
    
    boost::filesystem::path p(dataset);
    boost::filesystem::path dir = p.parent_path();
    boost::filesystem::path filename = p.filename();
    std::cout << "Current path " << dataset << std::endl;
    std::cout << "Parent dir " << dir << std::endl;
    std::cout << "Filename " << filename << std::endl;
    std::string trained_dataset = dir.string() + "/trained_"+filename.string() ;
    std::cout << "Trained dataset " << trained_dataset << std::endl;
    
    std::string trained_index_esf = "/trained_esf" ;
    std::string trained_index_vfh = "/trained_vfh" ;
    std::string trained_index_ourcvfh = "/trained_ourcvfh" ;
    std::string trained_index_cvfh = "/trained_cvfh" ;
    
    //Check if the dataset exists well
    if (check == "dataset"){
        if (!boost::filesystem::exists (dataset) && !boost::filesystem::is_directory(dataset) ){
            std::cerr<<"[ERROR] dataset does not exist or it's not a valid directory"<<std::endl;
            return -1;
            
        }else {
            std::cout << "[INFO] Dataset directory okay" << std::endl;
            std::string dataset_directory = dataset + "/" + dataset_categories_file;
            std::cout << "[INFO] Dataset directory : " << dataset_directory << std::endl;
            //Check now if the file which contains the categories exists
            if (!boost::filesystem::exists (dataset_directory)){
                std::cout << "[INFO] The txt file with the categories does not exist. Going to create one." << std::endl;
                char command[100];
                std::string dataset_path_slash = dataset + "/";
                sprintf (command, "../scripts/generate_files_database.sh  %s ", dataset_path_slash.c_str());
                system(command);
            }
            
            return 0;
        }
    }
    
    // To be sure if the user selects another option
    if (check == "dataset"){
        if (!boost::filesystem::exists (dataset) && !boost::filesystem::is_directory(dataset) ){
            std::cerr<<"[ERROR] dataset does not exist or it's not a valid directory"<<std::endl;
            return -1;
        }
    }else {
        if (check == "descriptors"){
            if (!boost::filesystem::exists (trained_dataset) && !boost::filesystem::is_directory(trained_dataset) ){
                std::cerr<<"[ERROR] Trained directory does not exist"<<std::endl;
                return -1;
                
            }else {
                for(boost::filesystem::recursive_directory_iterator it(trained_dataset); it!=boost::filesystem::recursive_directory_iterator(); ++it)
                {
                    std::string path_current = it->path().c_str();
                    //std::cout << path_current << std::endl;
                    if (path_current.find("/descriptors")  != std::string::npos ){
                        std::cout << "[INFO] Directory descriptors okay - Descriptors have been computed for the dataset" << std::endl;
                        return 0;
                        
                    }
                }
            }
        }else if (check == "views"){
            if (!boost::filesystem::exists (trained_dataset) && !boost::filesystem::is_directory(trained_dataset) ){
                std::cerr<<"[ERROR] Trained directory does not exist"<<std::endl;
                return -1;
                
            }else {
                std::string views_directory = trained_dataset + "/";
                for(boost::filesystem::recursive_directory_iterator it(trained_dataset); it!=boost::filesystem::recursive_directory_iterator(); ++it)
                {
                    std::string path_current = it->path().c_str();
                    //std::cout << path_current << std::endl;
                    if (path_current.find("/views")  != std::string::npos ){
                        std::cout << "[INFO] Directory Views okay - Views have been computed for the dataset" << std::endl;
                        return 0;
                        
                    }
                }
            }
            
        }
        
    }

    return 0;
    
}
