#ifndef RETRIEVAL_SIMILARITY_SEARCH_DATABASE
#define RETRIEVAL_SIMILARITY_SEARCH_DATABASE

// STL
#include <iostream>
#include <cctype>
#include <vector>
#include "others/utils.hpp"
#include <typeinfo>
#include <map>

//VTK
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>

//FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>

//Descriptors
#include "descriptor_estimation/vfh_estimation.hpp"
#include "descriptor_estimation/cvfh_estimation.hpp"
#include "descriptor_estimation/ourcvfh_estimation.hpp"
#include "descriptor_estimation/esf_estimation.hpp"
#include "descriptor_estimation/spin_estimation.hpp"
//#include "descriptor_estimation/spherical_harmonic_estimation.hpp"
#include "descriptor_estimation/usc_estimation.hpp"
#include "descriptor_estimation/sc_estimation.hpp"
#include "descriptor_estimation/crh_estimation.hpp"
#include "descriptor_estimation/grsd_estimation.hpp"
#include "descriptor_estimation/gshot_estimation.hpp"
#include "descriptor_estimation/gshot_pyramid_estimation.hpp"
#include "descriptor_estimation/good_estimation.cpp"
#include "descriptor_estimation/volume_area_estimation.hpp"

//BOOST
#include <boost/algorithm/string.hpp>

#include "others/Timer.hpp"
#include <stdlib.h>
#include <cstdlib>



template<typename PointT, typename DescriptorT, typename DistT>
class RetrievalSimilaritySearchDatabase
{
public:
    
    //couples together a pair of values, the path to the corresponding model and its value
    typedef std::pair<std::string, std::vector<float> > model;
    
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename PointCloudT::CloudVectorType CloudVectorT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    typedef typename pcl::search::FlannSearch<DescriptorT, DistT> SearchT;
    
    RetrievalSimilaritySearchDatabase();
    RetrievalSimilaritySearchDatabase(int trees, int checks);
    
    void loadDB(std::string db_dir);
    /**
     * Setter set the type of descriptor we are going to use
     * @param descriptor type of the descriptor. For now handling VFH, CVFH, OUR-CVFH and ESF
     * TODO : do a enum instead
     *
     */
    void setDescriptor(std::string descriptor)
    {
        m_descriptor_name = descriptor;
    }
    
    std::string getResultsDetectionJSONFormat(){
        return m_resultsdetectionJSONformat;
    }
    
    /**
     * Setter set the resolution we are going to use for downsampling the point cloud
     * @param resolution
     *
     */
    void setResolution(float resolution)
    {
        m_resolution = resolution;
    }
    
    /**
     * activate or deactivate the resolution mode
     * @param toActivate boolean true or false
     *
     */
    void setResolutionMode(bool toActivate){
        m_resolution_activated = toActivate;
    }
    
    void setComputeOnFull(bool toActivate){
        m_compute_full = toActivate;
    }
    void setScale(bool toActivate){
        m_scale_activated = toActivate;
    }
    
    
    int readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud);
    /**  Given a VIEW query of an object, try to find the most similar in the database
     * @param queryCloud the view of an object (pcd format)
     * @param path to the trained dataset
     * @param k the number of results we want
     */
    void globalMatchingUsingViewOfCloud(const std::string queryCloud, const std::string dataset_trained, const std::vector<std::string> categories, int k);
    
    void globalMatchingEvaluation(const std::string queryCloud, const std::string dataset_trained, const std::vector<std::string> categories,int k);
    
    void saveResultsDetectionsJSON(const string outputPath);
    
    double getAveragePrecision(){
        //the average may not necessarily be integer
        float avg = 0.0;  //or double for higher precision
        float sum = 0.0;
        int size = m_precision_array.size();
        for (int i = 0; i < size; ++i)
        {
            sum += m_precision_array.at(i);
        }
        avg = ((float)sum)/size; //or cast sum to double before division
        
        return avg;
    }
    
    map<int, double> get_precisions_values(){
        return m_precision_map;
    }
    
    
    
    
protected:
    /** Load the list of file model names from an ASCII file training_data.list . This file contains all the paths relative to all the descriptors of every objects of the dataset
     * \param models the output
     * \param filename path to the ASCII file training_data.list
     */
    bool loadFileList (std::vector<model> &models, const std::string &filename);
    
    /** Loads an n-D histogram file as a VFH signature model
     * @param path of the global descriptor corresponding to the object's views
     * @param vfh the resultant VFH model
     .fist = path
     .second = value of histogram
     */
    bool loadHistVFH (const boost::filesystem::path &path, model &vfh);
    /** Loads an n-D histogram file as a ESF signature model
     * @param path of the global descriptor corresponding to the object's views
     * @param esf the resultant ESF model
     */
    bool loadHistESF (const boost::filesystem::path &path, model &esf);
    /** Load a set of VFH features that will act as the model (training data)
     * @param directory the path to the dataset
     * @param models the resultant vector of histogram models
     */
    bool loadHistGSHOT (const boost::filesystem::path &path, model &gshot);
    bool loadHistGRSD (const boost::filesystem::path &path, model &grsd);
    bool loadHistESFVFH (const boost::filesystem::path &path, model &esfvfh);
    bool loadHistGOOD (const boost::filesystem::path &path, model &good);
    bool loadHistPointNet (const boost::filesystem::path &path, model &pointnet);
    
    
    void loadFeatureModelsVFH (const std::string &directory, std::vector<model> &models);
    /** Load a set of VFH features that will act as the model (training data)
     * @param directory the path to the dataset
     * @param models the resultant vector of histogram models
     */
    void loadFeatureModelsESF (const std::string &directory, std::vector<model> &models);
    void loadFeatureModelsGSHOT (const std::string &directory, std::vector<model> &models);
    void loadFeatureModelsGRSD (const std::string &directory, std::vector<model> &models);
    
    
    /** Search for the closest k neighbors
     * @param index the tree
     * @param model the query model
     * @param k the number of neighbors to search for
     * @param indices the resultant neighbor indices
     * @param distances the resultant neighbor distances
     */
    inline void nearestKSearch (flann::Index<DistT > &index, const model &model,int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
    
    void normalizePointCloud(typename pcl::PointCloud<PointT>::Ptr& cloud);
    
    std::vector<float> minMaxScaler(std::vector<float> data);
    
protected:
    void writeFloat(std::string path, float num);
    void writeMatrix4f(std::string path, Eigen::Matrix4f mat);
    void readFloat(std::string path, float& num);
    void readMatrix4f(std::string path, Eigen::Matrix4f& mat);
    std::string getDistanceName();
    std::string descriptor_name_;
    SearchT flann_search_;
    DescriptorCloudPtrT descriptors_;
    std::vector<std::string> names_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses_;
    CloudVectorT views_;
    
    std::string m_descriptor_name;
    std::string m_distance_used;
    
    std::string m_resultsdetectionJSONformat;
    
    bool m_resolution_activated;
    bool m_scale_activated;
    float m_resolution;
    bool m_compute_full;
    
    double m_precision_k1,m_precision_k5,m_precision_k10,m_precision_k15,m_precision_k20;
    std::vector<double> m_precision_array;
    map<int, double> m_precision_map;
    
};


template<typename PointT, typename DescriptorT, typename DistT>
std::vector<float> RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::minMaxScaler(std::vector<float> data){
    std::vector<float> result_min_max;
    auto max = std::max_element(std::begin(data), std::end(data));
    auto min = std::min_element(std::begin(data), std::end(data));
    for (int i = 0; i < data.size(); i++){
        float new_value = (data.at(i) - *min)/(*max - *min);
        result_min_max.push_back(new_value);
    }
    return result_min_max;
}

template<typename PointT, typename DescriptorT, typename DistT>
RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::RetrievalSimilaritySearchDatabase():flann_search_(true, typename SearchT::FlannIndexCreatorPtr(new typename SearchT::KdTreeMultiIndexCreator(4))),descriptors_(new DescriptorCloudT)
{
    flann_search_.setPointRepresentation(typename SearchT::PointRepresentationPtr (new pcl::DefaultFeatureRepresentation<DescriptorT>));
    flann_search_.setChecks(512);
}

template<typename PointT, typename DescriptorT, typename DistT>
RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::RetrievalSimilaritySearchDatabase(int trees, int checks):flann_search_(true, typename SearchT::FlannIndexCreatorPtr(new typename SearchT::KdTreeMultiIndexCreator(trees))),descriptors_(new DescriptorCloudT)
{
    flann_search_.setPointRepresentation(typename SearchT::PointRepresentationPtr (new pcl::DefaultFeatureRepresentation<DescriptorT>));
    flann_search_.setChecks(checks);
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::normalizePointCloud(typename pcl::PointCloud<PointT>::Ptr& cloud){
    //Compute Centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    //Demean Point cloud
    typename pcl::PointCloud<PointT>::Ptr cloud_demean (new pcl::PointCloud<PointT>);
    pcl::demeanPointCloud(*cloud,centroid,*cloud_demean);
    //Normalize by its root mean square distance to the origin
    int numberPoints = cloud_demean->size();
    float value_scale = 1;
    
    float accumX = 0.;
    float accumY = 0.;
    float accumZ = 0.;
    for(typename pcl::PointCloud<PointT>::iterator it = cloud_demean->begin(); it!= cloud_demean->end(); it++){
        float x = it->x;
        float y = it->y;
        float z = it->z;
        accumX += x * x;
        accumY += y * y;
        accumZ += z * z;
    }
    //Ps = Ps*np.sqrt(Ps.shape[1]/np.sum(Ps**2))
    float sumTotalNorm = accumX + accumY + accumZ;
    float res = numberPoints/sumTotalNorm;
    res =  sqrt (res);
    float scale = value_scale/(sqrt(sumTotalNorm));
    
    for(size_t i = 0; i < numberPoints; ++i){
        float x = cloud_demean->points[i].x;
        float y = cloud_demean->points[i].y;
        float z = cloud_demean->points[i].z;
        
        cloud_demean->points[i].x = x * res * scale ;
        cloud_demean->points[i].y = y * res  * scale ;
        cloud_demean->points[i].z = z * res  * scale ;
        
    }
    cloud = cloud_demean;
    
    
}

//Read point cloud from a path
template<typename PointT, typename DescriptorT, typename DistT>
int RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud)
{
    
    if (!boost::filesystem::exists (object_path)){
        std::cerr << "Error with pcd file - Check the path" << std::endl;
        return -1;
    }
    std::string extension = boost::filesystem::extension(object_path);
    if (extension == ".pcd" || extension == ".PCD")
    {
        if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else if (extension == ".ply" || extension == ".PLY")
    {
        if (pcl::io::loadPLYFile(object_path , *point_cloud) == -1)
        {
            std::cout << "\n Cloud reading failed." << std::endl;
            return (-1);
        }
    }
    else
    {
        std::cout << "\n file extension is not correct. Syntax is: compute_descriptor_cloud_main <path/file_name.pcd> [--nogui] or compute_descriptor_cloud_main <path/file_name.ply> [--nogui]" << std::endl;
        return -1;
    }
    return 1;
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadDB(std::string db_dir)
{
    descriptors_->clear();
    names_.clear();
    poses_.clear();
    views_.clear();
    
    for(boost::filesystem::directory_iterator it(db_dir); it != boost::filesystem::directory_iterator(); ++it)
    {
        for(boost::filesystem::directory_iterator descriptor_it((it->path()/"descriptors")/descriptor_name_); descriptor_it != boost::filesystem::directory_iterator(); ++descriptor_it)
        {
            std::string file_num = descriptor_it->path().filename().replace_extension().string().substr(11);
            
            //load model descriptor
            DescriptorCloudPtrT descriptor(new DescriptorCloudT);
            pcl::io::loadPCDFile<DescriptorT>(descriptor_it->path().string(), *descriptor);
            descriptors_->push_back(descriptor->at(0));
            
            //load model name
            names_.push_back(it->path().filename().string());
            
            //load model pose
            std::string pose_file_name = std::string("pose_") + file_num + std::string(".txt");
            Eigen::Matrix4f pose;
            readMatrix4f(((it->path()/"poses")/pose_file_name).string(), pose);
            poses_.push_back(pose);
            
            //load each view
            std::string view_file_name = std::string("view_") + file_num + std::string(".pcd");
            PointCloudT view;
            pcl::io::loadPCDFile<PointT>(((it->path()/"views")/view_file_name).string(), view);
            views_.push_back(view);
        }
    }
    flann_search_.setInputCloud(descriptors_);
}



/** Loads an n-D histogram file as a VFH signature model
 * @param path of the global descriptor corresponding to the object's views
 * @param esf the resultant VFH model
 */
#pragma mark -  Load VFH histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistVFH (const boost::filesystem::path &path, model &vfh)
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

/** Loads an n-D histogram file as a ESF signature model
 * @param path of the global descriptor corresponding to the object's views
 * @param esf the resultant ESF model
 */
#pragma mark -  Load ESF histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistESF (const boost::filesystem::path &path, model &esf)
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


#pragma mark -  Load GOOD histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistGOOD (const boost::filesystem::path &path, model &good)
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
#pragma mark -  Load ESFVFH histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistESFVFH (const boost::filesystem::path &path, model &esfvfh)
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
        //std::cout << desc_str.at(i) << std::endl;
        if(desc_str.at(i).find_first_not_of(' ') != std::string::npos)
        {
            if(desc_str.at(i).find_first_not_of(' ') != std::string::npos)
            {
                // There's a non-space.
                desc_float.push_back( std::stof (desc_str.at(i)));
                
            }
        }

    }
    
    
    // Treat the esf signature as a single Point Cloud
    esfvfh.second.resize (948);
    
    for (size_t i = 0; i <desc_float.size(); i++)
    {
        esfvfh.second[i] = desc_float.at(i);
    }
    esfvfh.first = path.string ();
    
    return (true);
    
}
#pragma mark -  Load POINTNET histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistPointNet (const boost::filesystem::path &path, model &pointNet)
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


#pragma mark -  Load GSHOT histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistGSHOT(const boost::filesystem::path &path, model &gshot)
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


/** Loads an n-D histogram file as a ESF signature model
 * @param path of the global descriptor corresponding to the object's views
 * @param esf the resultant ESF model
 */
#pragma mark -  Load ESF histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadHistGRSD (const boost::filesystem::path &path, model &grsd)
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
    
    // Treat the esf signature as a single Point Cloud
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

/** Load the list of file model names from an ASCII file training_data.list . This file contains all the paths relative to all the descriptors of every objects of the dataset
 * @param models the output
 * @param filename path to the ASCII file training_data.list
 */
#pragma mark -  Load list of files
template<typename PointT, typename DescriptorT, typename DistT>
bool RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::loadFileList (std::vector<model> &models, const std::string &filename)
{
    ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
        return (false);
    
    std::string line;
    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.empty ())
            continue;
        model m;
        m.first = line;
        models.push_back (m);
    }
    fs.close ();
    return (true);
}



// Define user defined literal "_quoted" operator.
std::string operator"" _quoted(const char* text, std::size_t len) {
    return "\"" + std::string(text, len) + "\"";
}

#pragma mark - precision
double computePrecision(int TP,int FP){
    double precision = (double)TP / ((double)TP + (double)FP);
    precision = precision * 100;
    return precision;
}

#pragma mark - recall
double computeRecall(int TP,int FN){
    double recall = (double)TP / ((double)TP + (double)FN);
    recall = recall * 100;
    return recall;
}

#pragma mark - accuracy
double computeAccuracy(int TP, int TN, int FP,int FN){
    double accuracy = ((double)TP + (double)TN) / ((double)TP + (double)TN + (double)FP + (double)FN);
    accuracy = accuracy * 100;
    return accuracy;
}

#pragma mark - globalMatchingUsingViewOfCloud
/**  Given a view query of an object, try to find the most similar in the database
 * @param queryCloud the view of an object (pcd format)
 * @param k the number of results we want
 */
template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::globalMatchingUsingViewOfCloud(const std::string queryCloud,const std::string dataset_trained,const std::vector<std::string> categories, int k){
    
    
    //std::size_t found = queryCloud.find_last_of("/\\");
    //std::string parentDirectory = queryCloud.substr(0,found);
    std::string parentDirectory = "..";
    boost::filesystem::path output_path(parentDirectory);
    
    std::string queryDescriptor = "";
    
    //std::cout << "[INFO] Output Path : " << output_path << std::endl;
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    std::cout << "[INFO] Loading target cloud: " << queryCloud << " ... " << std::flush;
    if (readPointCloud( queryCloud,  cloud)==-1)
        return;
    
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
    //normalizePointCloud(cloud);
    //pcl::io::savePCDFileASCII ("test_pcd_norm.pcd", *cloud);

    /*if (pcl::io::loadPCDFile<PointT>(queryCloud, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file PCD file \n");
        std::cout << "[ERROR] Couldn't read file PCD file \n" << std::endl;
        
    }else {
        //std::cout << "[INFO] Loading Complete \n" << std::endl;
    }
    */
    
    if (m_resolution_activated){
        std::cout << "[INFO] Using resolution : " << m_resolution << std::endl;
        std::cout << "Resolution of the uploaded model : " << Utils::compute_resolution(cloud) << std::endl;
        std::cout << "Number of points  : " << cloud->size() << std::endl;
        //Downsize for resolution invariance
        pcl::VoxelGrid<pcl::PointXYZ> down;
        float leaf = m_resolution;
        down.setLeafSize (leaf, leaf, leaf);
        down.setInputCloud (cloud);
        down.filter (*cloud);
        std::cout << "Resolution of the uploaded model after filtering : " << Utils::compute_resolution(cloud) << std::endl;
        std::cout << "Number of points  : " << cloud->size() << std::endl;

    }
    
    if (cloud->size() < 3000){
    //This one
    //cloud = Utils::upsampling(cloud,0.03,0.01,0.003);
    //cloud = Utils::upsampling(cloud,0.5,0.06,0.02);
    }
    Timer timetotal;
    timetotal.startTimer();
    //First compute the descriptor
    std::cout << "Computation of the descriptor ... " << std::flush;
    
    boost::filesystem::create_directory(output_path/"descriptors_view");
    std::stringstream descriptor_name;
    if (m_descriptor_name == "vfh"){
        std::cout << " VFH  " << std::flush;
        
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        vfh_estimation.setInputCluster(*cloud);
        vfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> vfhs;
        vfh_estimation.getResultDescriptors(vfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(vfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }

        
        boost::filesystem::create_directory(output_path/"descriptors_view/vfh");
        
        descriptor_name<<(output_path/"descriptors_view/vfh/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), vfhs);
        
        
    }else if (m_descriptor_name == "cvfh"){
        std::cout << " CVFH  " << std::flush;
        CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
        cvfh_estimation.setInputCluster(*cloud);
        cvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> cvfhs;
        cvfh_estimation.getResultDescriptors(cvfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(cvfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }

        
        boost::filesystem::create_directory(output_path/"descriptors_view/cvfh");
        descriptor_name<<(output_path/"descriptors_view/cvfh/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), cvfhs);
        
        
    }else if (m_descriptor_name == "ourcvfh"){
        std::cout << " OURCVFH  " << std::flush;
        OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
        ourcvfh_estimation.setInputCluster(*cloud);
        ourcvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
        ourcvfh_estimation.getResultDescriptors(ourcvfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }

        
        boost::filesystem::create_directory(output_path/"descriptors_view/ourcvfh");
        
        descriptor_name<<(output_path/"descriptors_view/ourcvfh/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), ourcvfhs);
        
        
    }else if (m_descriptor_name == "esf"){
        std::cout << " ESF  " << std::flush;
        ESFEstimation<pcl::PointXYZ> esf_estimation;
        esf_estimation.setInputCluster(*cloud);
        esf_estimation.estimate();
        pcl::PointCloud<pcl::ESFSignature640> esfs;
        esf_estimation.getResultDescriptors(esfs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
            {
                data_tmp.push_back(esfs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }

        
        boost::filesystem::create_directory(output_path/"descriptors_view/esf");
        
        descriptor_name<<(output_path/"descriptors_view/esf/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name.str(), esfs);
        
    }else if (m_descriptor_name == "gshot"){
        std::cout << " GSHOT  " << std::flush;
        GSHOTEstimation<pcl::PointXYZ> gshot_estimation;
        gshot_estimation.setInputCluster(*cloud);
        gshot_estimation.estimate();
        pcl::PointCloud<pcl::SHOT352> gshots;
        gshot_estimation.getResultDescriptors(gshots);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
            {
                data_tmp.push_back(gshots.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/gshot");
        
        descriptor_name<<(output_path/"descriptors_view/gshot/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name.str(), gshots);
    }
    else if (m_descriptor_name == "gshotPyramid"){
        std::cout << " GSHOT Pyramid" << std::flush;
        GSHOTPyramidEstimation<pcl::PointXYZ> gshot_pyramid_estimation;
        
        gshot_pyramid_estimation.setInputCloud(*cloud);
        gshot_pyramid_estimation.estimate();
        pcl::PointCloud<pcl::SHOT352> gshots = gshot_pyramid_estimation.getResultDescriptors();
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
            {
                data_tmp.push_back(gshots.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/gshotPyramid");
        
        descriptor_name<<(output_path/"descriptors_view/gshotPyramid/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name.str(), gshots);
    }
    else if (m_descriptor_name == "grsd"){
        std::cout << " GRSD  " << std::flush;
        GRSDEstimation<pcl::PointXYZ> grsd_estimation;
        grsd_estimation.setInputCluster(*cloud);
        grsd_estimation.estimate();
        pcl::PointCloud<pcl::GRSDSignature21> grsd;
        grsd_estimation.getResultDescriptors(grsd);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
            {
                data_tmp.push_back(grsd.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/grsd");
        
        descriptor_name<<(output_path/"descriptors_view/grsd/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(descriptor_name.str(), grsd);
    }
    else if (m_descriptor_name == "good"){
        GOODEstimation<pcl::PointXYZ> good_estimation;
        good_estimation.setNumberOfBins(5);
        good_estimation.setThreshold(0.0015);
        
        // Provide the original point cloud
        good_estimation.setInputCloud(cloud);
        
        // Compute GOOD discriptor for the given object
        std::vector< float > object_description;
        good_estimation.compute(object_description);
        
        if (m_scale_activated){
            std::vector<float> value_descriptor_scaled;
            
            value_descriptor_scaled = minMaxScaler(object_description);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                object_description.at(j) = value_descriptor_scaled.at(j);
            }
        }
        
        boost::filesystem::create_directory(output_path/"descriptors_view/good");
        descriptor_name<<(output_path/"descriptors_view/good/").string()<<"descriptor_0.txt";
        std::string output = descriptor_name.str();
        ofstream descfile;
        descfile.open (output,ios::out );
        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
        descfile.close();
    }
    else if (m_descriptor_name == "pointnet"){
        //TODO
        // Call the Python script passing a filename argument.
        std::string command = "/usr/local/bin/python3 /Users/lironesamoun/pytorch_exemple/pointnet.pytorch/get_descriptor.py -query ";
        //command = command + queryCloud + " -model cls_cat10_normalized/cat10_normalized_model.pth -nbclasses 10 >> test.txt";
        command = command + queryCloud + " -model cls_structureSensor_normalized/structuresensor_normalized_model.pth -nbclasses 7 >> test.txt";
        //command += filename;
        system(command.c_str());
        
        std::vector<std::string> valuesDescriptorStr;
        std::ifstream file("test.txt");
        
        std::string valueStr;
        while (file >> valueStr) {
            valuesDescriptorStr.push_back(valueStr);
        }
        
        std::cout << "SIZE VECTOR " << valuesDescriptorStr.size() << std::endl;
        std::vector< float > object_description(valuesDescriptorStr.size());
        std::transform(valuesDescriptorStr.begin(), valuesDescriptorStr.end(), object_description.begin(), [](const std::string& val)
        {
            return std::stod(val);
        });
         std::cout << "SIZE VECTOR " << object_description.size() << std::endl;
       
        /*if (m_scale_activated){
            std::vector<float> value_descriptor_scaled;
            
            value_descriptor_scaled = minMaxScaler(object_description);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                object_description.at(j) = value_descriptor_scaled.at(j);
            }
         
        }*/
        
        boost::filesystem::create_directory(output_path/"descriptors_view/pointnet");
        descriptor_name<<(output_path/"descriptors_view/pointnet/").string()<<"descriptor_0.txt";
        std::string output = descriptor_name.str();
        ofstream descfile;
        descfile.open (output,ios::out );
        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
        descfile.close();
    }
    else if (m_descriptor_name == "usc"){
        std::cout << " USC  " << std::flush;
        USCEstimation<pcl::PointXYZ> usc_estimation;
        usc_estimation.setInputCloud(*cloud);
        usc_estimation.estimate();
        pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
        usc = usc_estimation.getResultDescriptors();
        boost::filesystem::create_directory(output_path/"descriptors_view/usc");
        
        descriptor_name<<(output_path/"descriptors_view/usc/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::UniqueShapeContext1960>(descriptor_name.str(), usc);
    }
    else if (m_descriptor_name == "sc3D"){
        std::cout << " SC3D  " << std::flush;
        SC3DEstimation<PointT> sc3D_estimation;
        sc3D_estimation.setInputCluster(*cloud);
        sc3D_estimation.estimate();
        pcl::PointCloud<pcl::ShapeContext1980> sc;
        sc3D_estimation.getResultDescriptors(sc);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
            {
                data_tmp.push_back(sc.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
    
        boost::filesystem::create_directory(output_path/"descriptors_view/sc3D");
        
        descriptor_name<<(output_path/"descriptors_view/sc3D/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(descriptor_name.str(), sc);
    }else if (m_descriptor_name == "esfvfh"){
        
        std::vector<float> esfvfh_desc;
        
        std::cout << " ESFVFH  " << std::flush;
        ESFEstimation<pcl::PointXYZ> esf_estimation;
        esf_estimation.setInputCluster(*cloud);
        esf_estimation.estimate();
        pcl::PointCloud<pcl::ESFSignature640> esfs;
        esf_estimation.getResultDescriptors(esfs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
            {
                data_tmp.push_back(esfs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                ////Concatenation descriptors
                esfvfh_desc.push_back(value_descriptor_scaled.at(j));
            }
        }
        
        
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        vfh_estimation.setInputCluster(*cloud);
        vfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> vfhs;
        vfh_estimation.getResultDescriptors(vfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(vfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
                ////Concatenation descriptors
                esfvfh_desc.push_back(value_descriptor_scaled.at(j));
            }
        }
        
     
        std::cout << "SIZE FINAL CONCATENATION : " << esfvfh_desc.size() << std::endl;
        
        boost::filesystem::create_directory(output_path/"descriptors_view/esfvfh");
        descriptor_name<<(output_path/"descriptors_view/esfvfh/").string()<<"descriptor_0.txt";
        std::string output = descriptor_name.str();
        std::cout << output << std::endl;
        ofstream descfile;
        descfile.open(output,ios::out );
        copy(esfvfh_desc.begin(), esfvfh_desc.end(), std::ostream_iterator<float>(descfile ," "));
        descfile.close();
        
    }

    else{
        std::cerr << "[ERROR] Unknow type of descriptor " << std::endl;
        exit(1);
    }
    
    
    queryDescriptor = descriptor_name.str();
    std::cout << "[INFO] Descriptor computation completed \n" << std::endl;
    
    
    //std::cout << "[INFO] Matching stage ... " << std::endl;
    // Load the global descriptor of the target cloud
    model histogram;
    if(m_descriptor_name == "vfh" || m_descriptor_name == "cvfh" || m_descriptor_name == "ourcvfh"){
        std::cout << "[INFO]LOAD VFH type histogram " << std::endl;
        if (!loadHistVFH (queryDescriptor, histogram))
        {
            pcl::console::print_error (" [ERROR] Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "gshot" || m_descriptor_name == "gshotPyramid"){
        std::cout << "[INFO] LOAD SHOT histogram " << std::endl;
        if (!loadHistGSHOT (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }    
    }
    else if (m_descriptor_name == "grsd"){
        std::cout << "[INFO] LOAD GRSD histogram " << std::endl;
        if (!loadHistGRSD (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    //Suppose ESF
    else if (m_descriptor_name == "esf") {
        std::cout << "[INFO] LOAD ESF histogram " << std::endl;
        if (!loadHistESF (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "good") {
        std::cout << "[INFO] LOAD GOOD histogram " << std::endl;
        if (!loadHistGOOD (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "pointnet") {
        std::cout << "[INFO] LOAD POINTNET histogram " << std::endl;
        if (!loadHistPointNet (queryDescriptor, histogram))
        {
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "esfvfh") {
        std::cout << "[INFO] LOAD ESFVFH histogram " << std::endl;
        if (!loadHistESFVFH (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    
    
    
    
    std::cout << "[INFO]Using " << k << " nearest neighbors." <<  std::endl;
    std::string path_trained_dataset,kdtree_idx_file_name,training_data_h5_file_name,training_data_list_file_name;
    /*
    std::string path_trained_dataset = dataset_trained + "training_"+m_descriptor_name+ "/";
    std::string kdtree_idx_file_name = path_trained_dataset + "kdtree.idx";
    std::string training_data_h5_file_name = path_trained_dataset + "training_data.h5";
    std::string training_data_list_file_name = path_trained_dataset + "training_data.list";
     */
    if (m_compute_full){
        path_trained_dataset = dataset_trained + "training_"+m_descriptor_name+ "/";
        kdtree_idx_file_name = path_trained_dataset + "kdtree_full.idx";
        training_data_h5_file_name = path_trained_dataset + "training_data_full.h5";
        training_data_list_file_name = path_trained_dataset + "training_data_full.list";
    }else {
        path_trained_dataset = dataset_trained + "training_"+m_descriptor_name+ "/";
        kdtree_idx_file_name = path_trained_dataset + "kdtree_views.idx";
        training_data_h5_file_name = path_trained_dataset + "training_data_views.h5";
        training_data_list_file_name = path_trained_dataset + "training_data_views.list";
    }
  
    
    std::cout << "[DEBUG] Path dataset trained " << path_trained_dataset <<  std::endl;
    
    std::vector<model> models;
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Matrix<float> data;
    // Check if the data has already been saved to disk
    if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
    {
        pcl::console::print_error ("[ERROR] Could not find training data models files %s and %s!\n",
                                   training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
        exit(1);
    }
    //If not we load the list file of all the global descriptor
    //We load also the h5 file
    //we load the training data from disk, together with the list of file names that we previously stored
    else
    {
        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");
        pcl::console::print_highlight ("[INFO] Training data found. Loaded %d models from %s/%s.\n",
                                       (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    }
    
    
    // Check if the tree index has already been saved to disk
    if (!boost::filesystem::exists (kdtree_idx_file_name))
    {
        pcl::console::print_error ("[ERROR] Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
        exit(1);
    }
    else
    {
        flann::Index<DistT> index (data, flann::SavedIndexParams (kdtree_idx_file_name));
        index.buildIndex ();
        nearestKSearch (index, histogram, k, k_indices, k_distances);
    }
    
    timetotal.stopTimer();
    timetotal.getTime();
    
    std::string distance =  getDistanceName();
    m_resultsdetectionJSONformat = "[ \n";

    // Output the results on screen and output result to json file
    std::cout << "\n [RESULT] The closest " << k << " neighbors for "   << queryCloud << " are: " << std::endl;
    
    std::vector<std::string> string_split;
    boost::split(string_split, queryCloud, boost::is_any_of("/"));
    //For rgbd dataset, put 7 instead of 5
    //std::string category_query = string_split.at(5);
    //std::cout << "Category : " << category_query << std::endl;
  
    
   /* std::vector<std::string> strs;
    for (int i = 0; i < models.size(); ++i){
        
        boost::split(strs, models.at (i).first, boost::is_any_of("/"));
        //std::cout << strs.at(3) << std::endl;
         //std::cout <<  models.at (i).first << std::endl;
        
    }*/
    //std::cout << queryCloud << std::endl;
    //std::cout << category_query << std::endl;

    for (int i = 0; i < k; ++i){
   //std::cout <<models.at (k_indices[0][i]).first<<std::endl;
        for (int j = 0;j< categories.size();j++){
            
            std::vector<std::string> strs;
            boost::split(strs, models.at (k_indices[0][i]).first, boost::is_any_of("/"));
            if (models.at (k_indices[0][i]).first.find(categories.at(j)) != string::npos) {
            //if (strs.at(3) == categories.at(j)) {

                std::cout << i << " - " << categories.at(j) << " (" << k_indices[0][i] <<")"<< " with a distance of: " << k_distances[0][i] << "\n";
                std::cout << models.at (k_indices[0][i]).first << std::endl;
                
            
                m_resultsdetectionJSONformat += "{" + "Number"_quoted + " :" + std::to_string(i) + ", " + "Category"_quoted + " :" + "\"" + categories.at(j) + "\"" +", " + "id"_quoted + ":" + std::to_string(k_indices[0][i]) + "," + "Distance"_quoted + ":" + std::to_string(k_distances[0][i]) + " , " + "metric"_quoted + ":" +"\""+ distance+"\""  + " , " + "path"_quoted + ":" + "\""+models.at (k_indices[0][i]).first+"\""+ "}, \n";
                
                if (i == k-1){
                    m_resultsdetectionJSONformat += "{" + "Number"_quoted + " :" + std::to_string(i+1) + ", " + "Category"_quoted + " :" +  "\""+categories.at(j)+"\"" +", " + "id"_quoted + ":" + std::to_string(k_indices[0][i]) +"," + "Distance"_quoted + ":" + std::to_string(k_distances[0][i])  +" , " + "metric"_quoted + ":" + "\""+distance+"\"" + " , " + "path"_quoted + ":" + "\""+models.at (k_indices[0][i]).first+"\""+ "} \n";
                    
                }
            }
        }
    }
    m_resultsdetectionJSONformat += "] \n";
    /*
    double k_array[5] = {1, 5, 10, 15, 20};
    //Compute precision and recall
    int TP = 0, FP = 0, FN = 0;
    for (int k_v = 0; k_v < 5; k_v ++){
        for (int i = 0; i < k_array[k_v]; ++i){
            std::vector<std::string> strs;
            boost::split(strs, models.at (k_indices[0][i]).first, boost::is_any_of("/"));
            //std::cout << "QUERY : " << strs.at(1) << std::endl;
            //std::cout << "CATEGORY QUERY : " << category_query << std::endl;
            if (category_query.find(strs.at(3)) != std::string::npos) {
                TP = TP + 1;
            }else {
                FP = FP + 1;
            }
        }
        std::cout << "====> FOR K = " << k_array[k_v] << std::endl;
        //std::cout << "TRUE POSITIVE : " << TP << std::endl;
        //std::cout << "FALSE POSITIVE : " << FP << std::endl;
        float precision = computePrecision(TP,FP);
        std::cout << "PRECISION : " << precision << "% \n"<< std::endl;
   
    }*/
    
}

#pragma mark -  Evaluation similarity Search
template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::globalMatchingEvaluation(const std::string queryCloud, const std::string dataset_trained, const std::vector<std::string> categories, int k ){
    //std::size_t found = queryCloud.find_last_of("/\\");
    //std::string parentDirectory = queryCloud.substr(0,found);
    std::string parentDirectory = "..";
    boost::filesystem::path output_path(parentDirectory);
    
    std::string queryDescriptor = "";
    
    //std::cout << "[INFO] Output Path : " << output_path << std::endl;
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (readPointCloud( queryCloud,  cloud)==-1)
        return;
    
    //pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
    //normalizePointCloud(cloud);
    //pcl::io::savePCDFileASCII ("test_pcd_norm.pcd", *cloud);
    
    /*if (pcl::io::loadPCDFile<PointT>(queryCloud, *cloud) == -1) //* load the file
     {
     PCL_ERROR ("Couldn't read file PCD file \n");
     std::cout << "[ERROR] Couldn't read file PCD file \n" << std::endl;
     
     }else {
     //std::cout << "[INFO] Loading Complete \n" << std::endl;
     }
     */
    
    if (m_resolution_activated){
        //Downsize for resolution invariance
        pcl::VoxelGrid<pcl::PointXYZ> down;
        float leaf = m_resolution;
        down.setLeafSize (leaf, leaf, leaf);
        down.setInputCloud (cloud);
        down.filter (*cloud);
    }
    
    if (cloud->size() < 3000){
        //cloud = Utils::upsampling(cloud,0.03,0.01,0.003);
        //this one
        //cloud = Utils::upsampling(cloud,0.5,0.06,0.02);
    }
    Timer timetotal;
    timetotal.startTimer();
    //First compute the descriptor
    
    boost::filesystem::create_directory(output_path/"descriptors_view");
    std::stringstream descriptor_name;
    if (m_descriptor_name == "vfh"){
        std::cout << " VFH  " << std::flush;
        
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        vfh_estimation.setInputCluster(*cloud);
        vfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> vfhs;
        vfh_estimation.getResultDescriptors(vfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(vfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                vfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        
        
        boost::filesystem::create_directory(output_path/"descriptors_view/vfh");
        
        descriptor_name<<(output_path/"descriptors_view/vfh/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), vfhs);
        
        
    }else if (m_descriptor_name == "cvfh"){

        CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
        cvfh_estimation.setInputCluster(*cloud);
        cvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> cvfhs;
        cvfh_estimation.getResultDescriptors(cvfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(cvfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                cvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        
        
        boost::filesystem::create_directory(output_path/"descriptors_view/cvfh");
        descriptor_name<<(output_path/"descriptors_view/cvfh/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), cvfhs);
        
        
    }else if (m_descriptor_name == "ourcvfh"){

        OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
        ourcvfh_estimation.setInputCluster(*cloud);
        ourcvfh_estimation.estimate();
        pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
        ourcvfh_estimation.getResultDescriptors(ourcvfhs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
            {
                data_tmp.push_back(ourcvfhs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                ourcvfhs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        
        
        boost::filesystem::create_directory(output_path/"descriptors_view/ourcvfh");
        
        descriptor_name<<(output_path/"descriptors_view/ourcvfh/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::VFHSignature308>(descriptor_name.str(), ourcvfhs);
        
        
    }else if (m_descriptor_name == "esf"){

        ESFEstimation<pcl::PointXYZ> esf_estimation;
        esf_estimation.setInputCluster(*cloud);
        esf_estimation.estimate();
        pcl::PointCloud<pcl::ESFSignature640> esfs;
        esf_estimation.getResultDescriptors(esfs);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::ESFSignature640::descriptorSize(); i++)
            {
                data_tmp.push_back(esfs.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                esfs.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        
        
        boost::filesystem::create_directory(output_path/"descriptors_view/esf");
        
        descriptor_name<<(output_path/"descriptors_view/esf/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name.str(), esfs);
        
    }else if (m_descriptor_name == "gshot"){

        GSHOTEstimation<pcl::PointXYZ> gshot_estimation;
        gshot_estimation.setInputCluster(*cloud);
        gshot_estimation.estimate();
        pcl::PointCloud<pcl::SHOT352> gshots;
        gshot_estimation.getResultDescriptors(gshots);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
            {
                data_tmp.push_back(gshots.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/gshot");
        
        descriptor_name<<(output_path/"descriptors_view/gshot/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name.str(), gshots);
    }
    else if (m_descriptor_name == "gshotPyramid"){

        GSHOTPyramidEstimation<pcl::PointXYZ> gshot_pyramid_estimation;
        
        gshot_pyramid_estimation.setInputCloud(*cloud);
        gshot_pyramid_estimation.estimate();
        pcl::PointCloud<pcl::SHOT352> gshots = gshot_pyramid_estimation.getResultDescriptors();
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
            {
                data_tmp.push_back(gshots.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/gshotPyramid");
        
        descriptor_name<<(output_path/"descriptors_view/gshotPyramid/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::SHOT352>(descriptor_name.str(), gshots);
    }
    else if (m_descriptor_name == "grsd"){

        GRSDEstimation<pcl::PointXYZ> grsd_estimation;
        grsd_estimation.setInputCluster(*cloud);
        grsd_estimation.estimate();
        pcl::PointCloud<pcl::GRSDSignature21> grsd;
        grsd_estimation.getResultDescriptors(grsd);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i < pcl::GRSDSignature21::descriptorSize(); i++)
            {
                data_tmp.push_back(grsd.points[0].histogram[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                grsd.points[0].histogram[j] = value_descriptor_scaled.at(j);
            }
        }
        boost::filesystem::create_directory(output_path/"descriptors_view/grsd");
        
        descriptor_name<<(output_path/"descriptors_view/grsd/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::GRSDSignature21>(descriptor_name.str(), grsd);
    }
    else if (m_descriptor_name == "good"){
        GOODEstimation<pcl::PointXYZ> good_estimation;
        good_estimation.setNumberOfBins(5);
        good_estimation.setThreshold(0.0015);
        
        // Provide the original point cloud
        good_estimation.setInputCloud(cloud);
        
        // Compute GOOD discriptor for the given object
        std::vector< float > object_description;
        good_estimation.compute(object_description);
        
        if (m_scale_activated){
            std::vector<float> value_descriptor_scaled;
            
            value_descriptor_scaled = minMaxScaler(object_description);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                object_description.at(j) = value_descriptor_scaled.at(j);
            }
        }
        
        boost::filesystem::create_directory(output_path/"descriptors_view/good");
        descriptor_name<<(output_path/"descriptors_view/good/").string()<<"descriptor_0.txt";
        std::string output = descriptor_name.str();
        ofstream descfile;
        descfile.open (output,ios::out );
        copy(object_description.begin(), object_description.end(), std::ostream_iterator<float>(descfile , " "));
        descfile.close();
    }
    else if (m_descriptor_name == "usc"){

        USCEstimation<pcl::PointXYZ> usc_estimation;
        usc_estimation.setInputCloud(*cloud);
        usc_estimation.estimate();
        pcl::PointCloud<pcl::UniqueShapeContext1960> usc;
        usc = usc_estimation.getResultDescriptors();
        boost::filesystem::create_directory(output_path/"descriptors_view/usc");
        
        descriptor_name<<(output_path/"descriptors_view/usc/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::UniqueShapeContext1960>(descriptor_name.str(), usc);
    }
    else if (m_descriptor_name == "sc3D"){

        SC3DEstimation<PointT> sc3D_estimation;
        sc3D_estimation.setInputCluster(*cloud);
        sc3D_estimation.estimate();
        pcl::PointCloud<pcl::ShapeContext1980> sc;
        sc3D_estimation.getResultDescriptors(sc);
        
        if (m_scale_activated){
            std::vector<float> data_tmp,value_descriptor_scaled;
            for (size_t i = 0; i <pcl::ShapeContext1980::descriptorSize(); i++)
            {
                data_tmp.push_back(sc.points[0].descriptor[i]);
            }
            value_descriptor_scaled = minMaxScaler(data_tmp);
            for (int j = 0; j < value_descriptor_scaled.size(); j++){
                sc.points[0].descriptor[j] = value_descriptor_scaled.at(j);
            }
        }
        
        boost::filesystem::create_directory(output_path/"descriptors_view/sc3D");
        
        descriptor_name<<(output_path/"descriptors_view/sc3D/").string()<<"descriptor_0.pcd";
        
        pcl::io::savePCDFileASCII<pcl::ShapeContext1980>(descriptor_name.str(), sc);
    }
    
    else{
        std::cerr << "[ERROR] Unknow type of descriptor " << std::endl;
        exit(1);
    }
    
    
    queryDescriptor = descriptor_name.str();

    
    
    //std::cout << "[INFO] Matching stage ... " << std::endl;
    // Load the global descriptor of the target cloud
    model histogram;
    if(m_descriptor_name == "vfh" || m_descriptor_name == "cvfh" || m_descriptor_name == "ourcvfh"){

        if (!loadHistVFH (queryDescriptor, histogram))
        {
            pcl::console::print_error (" [ERROR] Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "gshot" || m_descriptor_name == "gshotPyramid"){

        if (!loadHistGSHOT (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "grsd"){

        if (!loadHistGRSD (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    //Suppose ESF
    else if (m_descriptor_name == "esf") {

        if (!loadHistESF (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "good") {

        if (!loadHistGOOD (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    else if (m_descriptor_name == "esfvfh") {

        if (!loadHistESFVFH (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    
    
    
    
    std::cout << "[INFO]Using " << k << " nearest neighbors." <<  std::endl;
    
    std::string path_trained_dataset = dataset_trained + "training_"+m_descriptor_name+ "/";
    std::string kdtree_idx_file_name = path_trained_dataset + "kdtree.idx";
    std::string training_data_h5_file_name = path_trained_dataset + "training_data.h5";
    std::string training_data_list_file_name = path_trained_dataset + "training_data.list";
    
    std::vector<model> models;
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Matrix<float> data;
    // Check if the data has already been saved to disk
    if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
    {
        pcl::console::print_error ("[ERROR] Could not find training data models files %s and %s!\n",
                                   training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
        exit(1);
    }
    //If not we load the list file of all the global descriptor
    //We load also the h5 file
    //we load the training data from disk, together with the list of file names that we previously stored
    else
    {
        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");

    }
    
    
    // Check if the tree index has already been saved to disk
    if (!boost::filesystem::exists (kdtree_idx_file_name))
    {
        pcl::console::print_error ("[ERROR] Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
        exit(1);
    }
    else
    {
        flann::Index<DistT> index (data, flann::SavedIndexParams (kdtree_idx_file_name));
        index.buildIndex ();
        nearestKSearch (index, histogram, k, k_indices, k_distances);
    }
    
    timetotal.stopTimer();
    timetotal.getTime();
    
    std::string distance =  getDistanceName();
    m_resultsdetectionJSONformat = "[ \n";
    
    // Output the results on screen and output result to json file
    //std::cout << "\n [RESULT] The closest " << k << " neighbors for "   << queryCloud << " are: " << std::endl;
    
    std::vector<std::string> string_split;
    boost::split(string_split, queryCloud, boost::is_any_of("/"));
    //For rgbd dataset, put 7 instead of 5
    std::string category_query = string_split.at(5);
    
    
    std::vector<std::string> strs;
    for (int i = 0; i < models.size(); ++i){
        
        boost::split(strs, models.at (i).first, boost::is_any_of("/"));
        //std::cout << strs.at(1) << std::endl;
        
    }
    
    double k_array[5] = {1, 5, 10, 15, 20};
    map<int, double> precision_map;
    m_precision_array.clear();
    //Compute precision and recall
    int TP = 0, FP = 0, FN = 0;
    for (int k_v = 0; k_v < 5; k_v ++){
        for (int i = 0; i < k_array[k_v]; ++i){
            std::vector<std::string> strs;
            boost::split(strs, models.at (k_indices[0][i]).first, boost::is_any_of("/"));
            //if (category_query == strs.at(1)){
            if (category_query.find(strs.at(1)) != std::string::npos) {
                TP = TP + 1;
            }else {
                FP = FP + 1;
            }
        }
        std::cout << "====> FOR K = " << k_array[k_v] << std::endl;
        //std::cout << "TRUE POSITIVE : " << TP << std::endl;
        //std::cout << "FALSE POSITIVE : " << FP << std::endl;
        double precision = computePrecision(TP,FP);
        std::cout << "PRECISION : " << precision << "% \n"<< std::endl;
        m_precision_map.insert(std::pair<int,double>(k_array[k_v],precision));

        m_precision_array.push_back(precision);
    }
}

#pragma mark -  save JSON
template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::saveResultsDetectionsJSON(const string outputPath){
    if (m_resultsdetectionJSONformat.empty()){
        std::cerr << "[ERROR] No results available - Please fill in the results by running the retrieval method " << std::endl;
        
    }else {
        ofstream out;
        std::cout << "Path result : " << outputPath << std::endl;
        out.open(outputPath, ios::out);
        out << m_resultsdetectionJSONformat;
        out.close();
        pcl::console::print_highlight ("[INFO] JSON results has been saved to : %s \n ", outputPath.c_str());
    }
}


/** Search for the closest k neighbors
 * @param index the tree
 * @param model the query model
 * @param k the number of neighbors to search for
 * @param indices the resultant neighbor indices
 * @param distances the resultant neighbor distances
 */
#pragma mark -  nearest ksearch
template<typename PointT, typename DescriptorT, typename DistT>
inline void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::nearestKSearch (flann::Index<DistT > &index, const model &model,int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
    // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
    memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
    
    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch (p, indices, distances, k, flann::SearchParams (1024));
    delete[] p.ptr ();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::writeFloat(std::string path, float num)
{
    std::ofstream fout(path.c_str());
    fout<<num;
    fout.close();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::writeMatrix4f(std::string path, Eigen::Matrix4f mat)
{
    std::ofstream fout(path.c_str());
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3; j++)
            fout<<mat(i,j)<<' ';
        fout<<mat(i,3)<<std::endl;
    }
    fout.close();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::readFloat(std::string path, float& num)
{
    std::ifstream fin(path.c_str());
    fin>>num;
    fin.close();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::readMatrix4f(std::string path, Eigen::Matrix4f& mat)
{
    std::ifstream fin(path.c_str());
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            fin>>mat(i,j);
    
    fin.close();
}
template<typename PointT, typename DescriptorT, typename DistT>
std::string RetrievalSimilaritySearchDatabase<PointT, DescriptorT, DistT>::getDistanceName()
{
    if(typeid(DistT) == typeid(flann::L2<float>)){
        m_distance_used = "L2";
    }
    else if(typeid(DistT) == typeid( flann::ChiSquareDistance<float>)){
        m_distance_used = "ChiSquare";
    }
    else if(typeid(DistT) == typeid(flann::L1<float>)){
        m_distance_used = "L1";
    }
    /*
    else if(typeid(DistT) == typeid(flann::HistIntersectionUnionDistance<float>)){
        m_distance_used = "HistogramIntersectionUnion";
    }
     */
    else if(typeid(DistT) == typeid(flann::HellingerDistance<float>)){
        m_distance_used = "Hellinger";
    }
    else {
        m_distance_used = "Unknown";
    }
    
    return m_distance_used;
}


#endif
