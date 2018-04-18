// STL
#include <iostream>
#include <vector>

//NANOFLANN
//#include <nanoflann.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/impl/flann_search.hpp>
#include <others/typedefs.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>

//BOOST
#include <boost/algorithm/string.hpp>


#include "Configuration/configuration.hpp"

/**
 
Generate an Index KDTree structure for performing faster similarity search
@paramers trained  - This is the path to the dataset with contains the points clouds (partial views or full) along its descriptor. Follow the correct structure of the dataset
@paramers descriptor - The descriptor you want to use for building the Similarity Search Index. Check the lists of available descriptors below.
@paramers folder - If you want to build the index for partials views (so 'views') or full objects (so 'full')
 
 **/


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<90>,
                                  (float[90], histogram, histogram)
                                  )
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<153>,
                                  (float[153], histogram, histogram)
                                  )

bool build_tree = true;
std::string descriptor_type = "esf";
std::string folder_type = "full";

typedef std::pair<std::string, std::vector<float> > model;

/** \brief Loads an n-D histogram file as a VFH signature
 * \param path the input file name
 * \param vfh the resultant VFH model
 */
bool
loadHistVFH (const boost::filesystem::path &path, model &vfh)
{
    int vfh_idx;
    // Load the file as a PCD
    try
    {
        pcl::PCLPointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type; unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);
        
        vfh_idx = pcl::getFieldIndex (cloud, "vfh");
        if (vfh_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
        return (false);
    }
    
    // Treat the VFH signature as a single Point Cloud
    pcl::PointCloud <pcl::VFHSignature308> point;
    pcl::io::loadPCDFile (path.string (), point);
    vfh.second.resize (308);
    
    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "vfh", fields);
    
    for (size_t i = 0; i < fields[vfh_idx].count; ++i)
    {
        vfh.second[i] = point.points[0].histogram[i];
    }
    vfh.first = path.string ();
    return (true);
}

bool
loadHistGRSD (const boost::filesystem::path &path, model &grsd)
{
    int grsd_idx;
    // Load the file as a PCD
    try
    {
        pcl::PCLPointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type; unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);
        
        grsd_idx = pcl::getFieldIndex (cloud, "grsd");
        if (grsd_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
        return (false);
    }
    
    // Treat the VFH signature as a single Point Cloud
    pcl::PointCloud <pcl::GRSDSignature21> point;
    pcl::io::loadPCDFile (path.string (), point);
    grsd.second.resize (21);
    
    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "grsd", fields);
    
    for (size_t i = 0; i < fields[grsd_idx].count; ++i)
    {
        grsd.second[i] = point.points[0].histogram[i];
    }
    grsd.first = path.string ();
    return (true);
}

bool
loadHistGSHOT(const boost::filesystem::path &path, model &gshot)
{
    int gshot_idx;
    // Load the file as a PCD
    try
    {
        pcl::PCLPointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type; unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);
        
        gshot_idx = pcl::getFieldIndex (cloud, "shot");
        if (gshot_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
        return (false);
    }
    
    // Treat the Gshot signature as a single Point Cloud
    pcl::PointCloud <pcl::SHOT352> point;
    pcl::io::loadPCDFile (path.string (), point);
    gshot.second.resize (352);
    
    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "shot", fields);

    for (size_t i = 0; i < fields[gshot_idx].count; ++i)
    {
        gshot.second[i] = point.points[0].descriptor[i];
    }
    gshot.first = path.string ();
    return (true);
}


bool loadHistESF (const boost::filesystem::path &path, model &esf)
{
    int esf_idx;
    // Load the file as a PCD
    try
    {
        pcl::PCLPointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type; unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);
        
        esf_idx = pcl::getFieldIndex (cloud, "esf");
        if (esf_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
        return (false);
    }
    
    // Treat the esf signature as a single Point Cloud
    pcl::PointCloud <pcl::ESFSignature640> point;
    pcl::io::loadPCDFile (path.string (), point);
    esf.second.resize (640);
    
    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "esf", fields);
    
    for (size_t i = 0; i < fields[esf_idx].count; ++i)
    {
        esf.second[i] = point.points[0].histogram[i];
    }
    esf.first = path.string ();
    
    return (true);
    
}

bool loadHistESFVFH (const boost::filesystem::path &path, model &esfvfh)
{
    //Read the txtfile
    string line;
    ifstream myfile (path.c_str());
    std::vector<std::string> desc_str;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            //std::cout << "line : " << line << std::endl;
            boost::split(desc_str, line, boost::is_any_of("\t "));
            //std::cout << "Size number : " << desc_str.size() << std::endl;
        }
        myfile.close();
    }
    
    else cout << "Unable to open file";
    
    std::vector<float> desc_float;
    for (int i=0; i < desc_str.size();i++){
        if(desc_str.at(i).find_first_not_of(' ') != std::string::npos)
        {
            // There's a non-space.
            desc_float.push_back( std::stof (desc_str.at(i)));
            
        }
    }
    
    
    // Treat the esf signature as a single Point Cloud
    esfvfh.second.resize (948);

    for (size_t i = 0; i <desc_float.size(); ++i)
    {
        esfvfh.second[i] = desc_float.at(i);
        
    }
    esfvfh.first = path.string ();
    
    return (true);
    
}

bool loadHistGOOD (const boost::filesystem::path &path, model &good)
{
    //Read the txtfile
    string line;
    ifstream myfile (path.c_str());
    std::vector<std::string> desc_str;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            //std::cout << "line : " << line << std::endl;
            boost::split(desc_str, line, boost::is_any_of("\t "));
            //std::cout << "Size number : " << desc_str.size() << std::endl;
        }
        myfile.close();
    }
    
    else cout << "Unable to open file";
    
    std::vector<float> desc_float;
    for (int i=0; i < desc_str.size();i++){
        if(desc_str.at(i).find_first_not_of(' ') != std::string::npos)
        {
            // There's a non-space.
            desc_float.push_back( std::stof (desc_str.at(i)));

        }
            }
    // Treat the esf signature as a single Point Cloud
    good.second.resize (80);
    
    for (size_t i = 0; i <desc_float.size(); ++i)
    {
        good.second[i] = desc_float.at(i);
    }
    good.first = path.string ();
    
    return (true);
    
}

bool loadHistPointNet (const boost::filesystem::path &path, model &pointNet)
{
    //Read the txtfile
    string line;
    ifstream myfile (path.c_str());
    std::vector<std::string> desc_str;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            //std::cout << "line : " << line << std::endl;
            boost::split(desc_str, line, boost::is_any_of("\t "));
            //std::cout << "Size number : " << desc_str.size() << std::endl;
        }
        myfile.close();
    }
    
    else cout << "Unable to open file";
    
    std::vector<float> desc_float;
    for (int i=0; i < desc_str.size();i++){
        if(desc_str.at(i).find_first_not_of(' ') != std::string::npos)
        {
            // There's a non-space.
            desc_float.push_back( std::stof (desc_str.at(i)));
            
        }
    }

    // Treat the esf signature as a single Point Cloud
    pointNet.second.resize (1024);
    
    for (size_t i = 0; i <desc_float.size(); ++i)
    {
        pointNet.second[i] = desc_float.at(i);
    }
    pointNet.first = path.string ();
    
    return (true);
    
}

void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                   std::vector<model> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;
    
    int numberObjects = 0;
    
    for(boost::filesystem::recursive_directory_iterator it(base_dir); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {
        
        std::string path_current = it->path().c_str();
        if (path_current.find("/"+descriptor_type+"/")  != std::string::npos && path_current.find("/"+folder_type+"/")  != std::string::npos){
            
            std::cout << "File : " << it->path().c_str() << std::endl;
            model m;
            
            if (it->path().extension().string() == ".pcd"){
            if (descriptor_type == "vfh" || descriptor_type == "cvfh" || descriptor_type == "ourcvfh"){
                if (loadHistVFH (it->path().c_str(), m)){
                    models.push_back (m);
                    numberObjects++;
                }
            }
            //Supose esf
            else if (descriptor_type == "esf") {
                if (loadHistESF (it->path().c_str(), m)){
                    models.push_back (m);
                    numberObjects++;
                }
            }
            //Supose gshot
            else if (descriptor_type == "gshot" || descriptor_type == "gshotPyramid" ) {
                if (loadHistGSHOT (it->path().c_str(), m)){
                    models.push_back (m);
                    numberObjects++;
                }
            }
            //Supose grsd
            else if (descriptor_type == "grsd") {
                if (loadHistGRSD (it->path().c_str(), m)){
                    models.push_back (m);
                    numberObjects++;
                }
            }
                            //Descriptor in txt file
            }else if (it->path().extension().string() == ".txt"){
                if (descriptor_type == "alldesc0"){
                    
                    if (loadHistESFVFH (it->path().c_str(), m)){
                        models.push_back (m);
                        numberObjects++;
                    }
                   
                }else if (descriptor_type == "good") {
                    if (loadHistGOOD (it->path().c_str(), m)){
                        
                        models.push_back (m);
                        numberObjects++;
                    }
                }
                else if (descriptor_type == "pointnet") {
                    if ( loadHistPointNet (it->path().c_str(), m)){
                        models.push_back (m);
                        numberObjects++;
                    }
                }
            }
            
        }
    }
}


void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Build Index tree for indexing descriptors             *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << " -trained tained_dataset -descriptor type_descriptor -folder (views/full)  [Options]" << std::endl << std::endl;
    std::cout << "Folder need to be either views or full" << std::endl;
    std::cout << "Descriptor Availables : esf, cvfh,vfh, ourcvfh, gshot,pointnet, gshotPyramid, grsd, esfvfh " << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    
}

int
main (int argc, char** argv)
{
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }
    //If not enough parameters
    if (argc < 6)
    {
        std::cout << "[INFO] Not enough parameters " << std::endl;
        showHelp (argv[0]);
        return (-1);
    }
    
    std::string trained;
    if(pcl::console::parse_argument(argc, argv, "-trained", trained) == -1)
    {
        std::cerr<<"Please specify the trained directory for getting data"<<std::endl;
        return -1;
    }
    if(pcl::console::parse_argument(argc, argv, "-descriptor", descriptor_type) == -1)
    {
        std::cerr<<"Please specify the descriptors"<<std::endl;
        return -1;
    }
    
    if(pcl::console::parse_argument(argc, argv, "-folder", folder_type) == -1)
    {
        std::cerr<<"Please specify on which folder you want to compute (views or full)"<<std::endl;
        return -1;
    }
    
    if (descriptor_type == "esfvfh"){
        descriptor_type = "alldesc0";
    }

    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    
    std::string kdtree_idx_file_name = "kdtree_"+folder_type+".idx";
    std::string training_data_h5_file_name = "training_data_"+folder_type+".h5";
    std::string training_data_list_file_name = "training_data_"+folder_type+".list";
    
    std::vector<model> models;
    
    // Load the model histograms
    loadFeatureModels (trained, extension, models);
    pcl::console::print_highlight ("Loaded %d models. Creating training data %s/%s.\n",
                                   (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    
    if (models.size() ==0){
        std::cerr << "ERROR : 0 model loaded " << std::endl;
        exit(1);
    }
    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
    
    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];
    
    
    
    std::string path_training_Descriptor = trained + "/" + "training_" + descriptor_type + "/";
    
    std::string pathToSaveTrainingdata_h5_file_name = path_training_Descriptor + training_data_h5_file_name;
    //structure-sensor-trained/training_vfh/training_data.list
    std::string pathToSaveTrainingDataList = path_training_Descriptor + training_data_list_file_name;
    //structure-sensor-trained/training_vfh/kdtree.idx
    std::string pathToSaveKdtreeIdxFilename = path_training_Descriptor +  kdtree_idx_file_name;
    
    if (!boost::filesystem::exists(pathToSaveTrainingdata_h5_file_name)){
        boost::filesystem::create_directory(path_training_Descriptor);
    }
    
    std::cout << "[INFO] Saving H5 file to " << pathToSaveTrainingdata_h5_file_name << std::endl;
    std::cout << "[INFO] Saving training data list file to " << pathToSaveTrainingDataList << std::endl;
    std::cout << "[INFO] Saving kd tree file to " << pathToSaveKdtreeIdxFilename << std::endl;
    
    //Remove before saving
    if (boost::filesystem::exists(pathToSaveTrainingdata_h5_file_name)){
        boost::filesystem::remove(pathToSaveTrainingdata_h5_file_name);
        
    }
    if (boost::filesystem::exists(pathToSaveTrainingDataList)){
        boost::filesystem::remove(pathToSaveTrainingDataList);
        
    }
    
    
    // Save data to disk (list of models)
    flann::save_to_file (data, pathToSaveTrainingdata_h5_file_name, "training_data");
    std::ofstream fs;
    fs.open (pathToSaveTrainingDataList.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();
    
    // Build the tree index and save it to disk
    pcl::console::print_info ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
    
    if (descriptor_type == "cvfh" || descriptor_type == "ourcvfh"){
        flann::Index<flann::ChiSquareDistance<float> > index (data, KMeansIndexParams(32, 11,FLANN_CENTERS_RANDOM,0.2 ));
        //Constructs the nearest neighbor search index using the parameters provided to the constructor
        index.buildIndex ();
        
        if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
            boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
            
        }
        index.save (pathToSaveKdtreeIdxFilename);
    }else {
        
        if (descriptor_type == "vfh"){
            flann::Index<flann::L1<float> > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
                boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
                
            }
            index.save (pathToSaveKdtreeIdxFilename);
            
            //It's gshot descriptor
        }
        else if (descriptor_type == "good"){
            flann::Index<flann::L2<float> > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
                boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
                
            }
            index.save (pathToSaveKdtreeIdxFilename);
            //It's esf or GRSD
        }
        else if (descriptor_type == "pointnet"){
            flann::Index<flann::L2<float> > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
                boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
                
            }
            index.save (pathToSaveKdtreeIdxFilename);
        }
        else if (descriptor_type == "gshot" || descriptor_type == "gshotPyramid"){
            flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
                boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
                
            }
            index.save (pathToSaveKdtreeIdxFilename);
            
            //It's esf or GRSD
        }else {
            flann::Index<flann::L1<float> > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            if (boost::filesystem::exists(pathToSaveKdtreeIdxFilename)){
                boost::filesystem::remove(pathToSaveKdtreeIdxFilename);
                
            }
            index.save (pathToSaveKdtreeIdxFilename);
        }
    }
    
    
    delete[] data.ptr ();
    
    
    
}


