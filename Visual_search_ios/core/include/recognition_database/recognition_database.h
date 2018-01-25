#ifndef RECOGNITION_DATABASE
#define RECOGNITION_DATABASE

#include "others/utils.hpp"
#include <typeinfo>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include "recognition_database/hypothesis.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "descriptor_estimation/esf_estimation.h"



template<typename PointT, typename DescriptorT, typename DistT>
class RecognitionDatabase
{
public:
    
    //couples together a pair of values, the path to the corresponding model and its value
    typedef std::pair<std::string, std::vector<float> > model;
    
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtrT;
    typedef typename PointCloudT::CloudVectorType CloudVectorT;
    typedef typename pcl::PointCloud<DescriptorT> DescriptorCloudT;
    typedef typename DescriptorCloudT::Ptr DescriptorCloudPtrT;
    typedef typename std::vector<std::vector<Hypothesis<PointT> > > Hypotheses;
    typedef typename pcl::search::FlannSearch<DescriptorT, DistT> SearchT;
    
    RecognitionDatabase();
    RecognitionDatabase(int trees, int checks);
    void trainDB(std::string model_dir, std::string ouput_dir, bool force_retrain=false, int tesselation_level=1, float leaf=0.005f, int xres = 200, int yres = 200, int view_angle = 57);
    void loadDB(std::string db_dir);
    void queryDB(DescriptorCloudPtrT query, int k, Hypotheses& clusters_hypotheses);
    void queryDBObject(std::string queryOjbect, int k, Hypotheses& clusters_hypotheses);
    
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
    
    
    /**
     *   Crawled a directory structure, looked at all the .PCD files found corresponding to the global descriptors, tested them whether they are VFH signatures (or others depending of the descriptors) and loaded them in memory; Then converted the data into FLANN format and dumped it to disk;
     
     *   Finally built a kd-tree structure and dumped it to disk.
     *   @param databaseTrained where the dataset is. But need previously to compute all the global descriptors (cf trainDatabase() )
     */
    void buildTree(const std::string databaseTrained);
    
    /**  Given a VIEW query of an object, try to find the most similar in the database
     * @param queryCloud the view of an object (pcd format)
     * @param path to the trained dataset
     * @param k the number of results we want
     */
    void globalMatchingUsingViewOfCloud(const std::string queryCloud, const std::string dataset_trained, const std::vector<std::string> categories, int k);
    
    /**  Given a  query object point cloud, try to find the most similar in the database
     * @param queryCloud the view of an object (pcd format)
     * @param path to the trained dataset
     * @param k the number of results we want
     */
    void globalMatchingUsingCloud(const std::string queryCloud, const std::string dataset_trained, int k);
    
    void saveResultsDetectionsJSON(const string outputPath);
    
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
    void loadFeatureModelsVFH (const std::string &directory, std::vector<model> &models);
    /** Load a set of VFH features that will act as the model (training data)
     * @param directory the path to the dataset
     * @param models the resultant vector of histogram models
     */
    void loadFeatureModelsESF (const std::string &directory, std::vector<model> &models);
    
    /** Search for the closest k neighbors
     * @param index the tree
     * @param model the query model
     * @param k the number of neighbors to search for
     * @param indices the resultant neighbor indices
     * @param distances the resultant neighbor distances
     */
    inline void nearestKSearch (flann::Index<DistT > &index, const model &model,int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
    
    
    
    
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
    float m_resolution;

    
    //Rotate cloud
    void rotateCloudAxisZ(std::string path_cloud,float theta = M_PI/2);
};




template<typename PointT, typename DescriptorT, typename DistT>
RecognitionDatabase<PointT, DescriptorT, DistT>::RecognitionDatabase():flann_search_(true, typename SearchT::FlannIndexCreatorPtr(new typename SearchT::KdTreeMultiIndexCreator(4))),descriptors_(new DescriptorCloudT)
{
    flann_search_.setPointRepresentation(typename SearchT::PointRepresentationPtr (new pcl::DefaultFeatureRepresentation<DescriptorT>));
    flann_search_.setChecks(512);
}

template<typename PointT, typename DescriptorT, typename DistT>
RecognitionDatabase<PointT, DescriptorT, DistT>::RecognitionDatabase(int trees, int checks):flann_search_(true, typename SearchT::FlannIndexCreatorPtr(new typename SearchT::KdTreeMultiIndexCreator(trees))),descriptors_(new DescriptorCloudT)
{
    flann_search_.setPointRepresentation(typename SearchT::PointRepresentationPtr (new pcl::DefaultFeatureRepresentation<DescriptorT>));
    flann_search_.setChecks(checks);
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::rotateCloudAxisZ(std::string path_cloud,float theta){
    std::cout << "[INFO] Rotation AXIS Z " << std::endl;
    
    boost::filesystem::path model_path(path_cloud);
    std::string parent_path = model_path.parent_path().c_str();
    std::cout << "Parent path : " << parent_path << std::endl;
    // Load file | Works with PCD and PLY files
    bool file_is_pcd = false;
    typename pcl::PointCloud<PointT>::Ptr source_cloud (new pcl::PointCloud<PointT> ());
    pcl::PolygonMesh mesh;
    
    if(path_cloud.substr(path_cloud.find_last_of(".") + 1) == "obj") {
        std::cout << "OBJ file found " << std::endl;
        if (pcl::io::loadOBJFile (path_cloud, mesh) < 0)  {
            std::cout << "[ERROR] Error loading point cloud " << path_cloud << std::endl << std::endl;
            return;
        }
        
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        
        // Define a translation of 0 meters on the x axis.
        transform_2.translation() << 0.0, 0.0, 0.0;
        
        // The same rotation matrix as before; theta radians arround Z axis
        transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
        
        
        //Important part starts here
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        pcl::transformPointCloud(cloud, cloud, transform_2);
        pcl::toPCLPointCloud2(cloud, mesh.cloud);
        
        /*  // Executing the transformation
         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
         pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
         
         */
        std::string pathPLY = parent_path + "/Model.ply";
        pcl::io::savePLYFileBinary(pathPLY, mesh);
        
        // pcl::io::savePLYFileBinary<PointT>(pathPLY, *transformed_cloud);
        
        
    }
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::loadDB(std::string db_dir)
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

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::trainDB(std::string model_dir, std::string output_dir, bool force_retrain, int tesselation_level, float leaf, int xres, int yres, int view_angle)
{
    
    if (!boost::filesystem::exists(output_dir)){
        std::cerr << "[ERROR] Output Trained database does not exist - going to create one "<< std::endl;
        boost::filesystem::create_directory(output_dir);
        
    }
    bool rotateAxisZ = false;
    int count = 0;
    pcl::visualization::PCLVisualizer renderer("render");
    renderer.setBackgroundColor( 1, 1, 1 );
    
    boost::filesystem::path model_path(model_dir);
    boost::filesystem::path output_path(output_dir);
    
    //iterate through all the database
    
    for(boost::filesystem::recursive_directory_iterator it(model_path); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {
        
        //We can use only ply file to render
        if(it->path().filename().extension().string() != ".ply")
            continue;
        
        count ++;
        //As all the model have the same name Model.ply, take the parent path to have the name of the model
        std::string model_name = it->path().parent_path().filename().string();
        std::cout << "[INFO] model name " <<model_name << std::endl;
        
        //check if the model has been trained
        if(boost::filesystem::exists(output_path/model_name))
        {
            if(force_retrain)
                boost::filesystem::remove_all(output_path/model_name);
            else
            {
                pcl::console::print_highlight("Model: %s has been trained\n", model_name.c_str());
                continue;
            }
        }
        
        
        
        pcl::console::print_highlight("Rendering model:%s\n", model_name.c_str());
        
        //create some directories to store the trained models
        boost::filesystem::create_directory(output_path/model_name);
        boost::filesystem::create_directory(output_path/model_name/"views");
        boost::filesystem::create_directory(output_path/model_name/"poses");
        boost::filesystem::create_directory(output_path/model_name/"enthropies");
        boost::filesystem::create_directory(output_path/model_name/"descriptors");
        boost::filesystem::copy_file(it->path(), output_path/model_name/(model_name+".ply"));
        
        //read in the ply model
        vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
        readerQuery->SetFileName (it->path().c_str());
        vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
        readerQuery->Update();
        
        /*render the ply model from different view*/
        //add the model to PCLVisualizer
        renderer.addModelFromPolyData(polydata, "model", 0);
        //input parameter
        //const int xres = 200;
        //const int yres = 200;
        //const float view_angle = 57;
        //output parameter
        pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
        std::vector<float> enthropies;
        //do the rendering
        renderer.renderViewTesselatedSphere(xres, yres, views, poses, enthropies, tesselation_level, view_angle);
        renderer.removeCorrespondences("model");
        renderer.close();
        
        PointCloudT completeModel;
        
        VFHEstimation<pcl::PointXYZ> vfh_estimation;
        CVFHEstimation<pcl::PointXYZ> cvfh_estimation;
        OURCVFHEstimation<pcl::PointXYZ> ourcvfh_estimation;
        ESFEstimation<pcl::PointXYZ> esf_estimation;
        //iterate through all the views
#if(_OPENMP)
#pragma omp parallel for
#endif
        for(int i = 0; i < views.size(); ++i)
        {
            //add the partial view to the complete model
            Eigen::Matrix4f pose_inverse = Eigen::Matrix4f::Identity();
            pose_inverse.block(0,0,3,3) = poses[i].block(0,0,3,3).transpose();
            pose_inverse.block(0,3,3,1) = - poses[i].block(0,0,3,3).transpose() * poses[i].block(0,3,3,1);
            PointCloudT transformed_view;
            pcl::transformPointCloud<pcl::PointXYZ>(views[i], transformed_view, pose_inverse);
            completeModel += transformed_view;
            
            /*save the view, pose, enthropy, and descriptor to the disk*/
            //down sample the current view
            pcl::VoxelGrid<pcl::PointXYZ> down;
            down.setLeafSize (leaf, leaf, leaf);
            down.setInputCloud (views[i].makeShared());
            down.filter (views[i]);
            
            //save the view to a pcd file
            std::stringstream view_name;
            view_name<<(output_path/model_name/"views/").string()<<"view_"<<i<<".pcd";
            pcl::io::savePCDFileBinary<pcl::PointXYZ>(view_name.str(), views[i]);
            
            //save the transform of each view to a txt file
            std::stringstream pose_name;
            pose_name<<(output_path/model_name/"poses/").string()<<"pose_"<<i<<".txt";
            writeMatrix4f(pose_name.str(), poses[i]);
            
            //save the enthropy of each view to a txt file
            std::stringstream enthropy_name;
            enthropy_name<<(output_path/model_name/"enthropies/").string()<<"enthropy_"<<i<<".txt";
            writeFloat(enthropy_name.str(), enthropies[i]);
            
            //save the descriptor of each view
            //vfh
            vfh_estimation.setInputCluster(views[i]);
            vfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> vfhs;
            vfh_estimation.getResultDescriptors(vfhs);
            
            boost::filesystem::create_directory(output_path/model_name/"descriptors/vfh");
            std::stringstream descriptor_name;
            descriptor_name<<(output_path/model_name/"descriptors/vfh/").string()<<"descriptor_"<<i<<".pcd";
            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name.str(), vfhs);
            //cvfh
            cvfh_estimation.setInputCluster(views[i]);
            cvfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> cvfhs;
            cvfh_estimation.getResultDescriptors(cvfhs);
            
            boost::filesystem::create_directory(output_path/model_name/"descriptors/cvfh");
            descriptor_name.str("");
            descriptor_name<<(output_path/model_name/"descriptors/cvfh/").string()<<"descriptor_"<<i<<".pcd";
            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name.str(), cvfhs);
            //ourcvfh
            ourcvfh_estimation.setInputCluster(views[i]);
            ourcvfh_estimation.estimate();
            pcl::PointCloud<pcl::VFHSignature308> ourcvfhs;
            ourcvfh_estimation.getResultDescriptors(ourcvfhs);
            
            boost::filesystem::create_directory(output_path/model_name/"descriptors/ourcvfh");
            descriptor_name.str("");
            descriptor_name<<(output_path/model_name/"descriptors/ourcvfh/").string()<<"descriptor_"<<i<<".pcd";
            pcl::io::savePCDFileBinary<pcl::VFHSignature308>(descriptor_name.str(), ourcvfhs);
            //esf
            esf_estimation.setInputCluster(views[i]);
            esf_estimation.estimate();
            pcl::PointCloud<pcl::ESFSignature640> esfs;
            esf_estimation.getResultDescriptors(esfs);
            
            boost::filesystem::create_directory(output_path/model_name/"descriptors/esf");
            descriptor_name.str("");
            descriptor_name<<(output_path/model_name/"descriptors/esf/").string()<<"descriptor_"<<i<<".pcd";
            pcl::io::savePCDFileBinary<pcl::ESFSignature640>(descriptor_name.str(), esfs);
        }
        
        //save the complete model to a pcd file
        pcl::VoxelGrid<pcl::PointXYZ> down;
        down.setLeafSize(leaf, leaf, leaf);
        down.setInputCloud(completeModel.makeShared());
        down.filter(completeModel);
        std::stringstream complete_model_name;
        complete_model_name<<(output_path/model_name).string()<<'/'<<model_name<<".pcd";
        pcl::io::savePCDFileBinary<pcl::PointXYZ>(complete_model_name.str(), completeModel);
    }
    std::cout << "[INFO] Number of model trained : " << count << std::endl;
}



/** Loads an n-D histogram file as a VFH signature model
 * @param path of the global descriptor corresponding to the object's views
 * @param esf the resultant VFH model
 */
#pragma mark -  Load VFH histogram
template<typename PointT, typename DescriptorT, typename DistT>
bool RecognitionDatabase<PointT, DescriptorT, DistT>::loadHistVFH (const boost::filesystem::path &path, model &vfh)
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
bool RecognitionDatabase<PointT, DescriptorT, DistT>::loadHistESF (const boost::filesystem::path &path, model &esf)
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

/** Load the list of file model names from an ASCII file training_data.list . This file contains all the paths relative to all the descriptors of every objects of the dataset
 * @param models the output
 * @param filename path to the ASCII file training_data.list
 */
#pragma mark -  Load list of files
template<typename PointT, typename DescriptorT, typename DistT>
bool RecognitionDatabase<PointT, DescriptorT, DistT>::loadFileList (std::vector<model> &models, const std::string &filename)
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

/** Load a set of VFH features that will act as the model (training data)
 * \param directory the path to the dataset
 * \param models the resultant vector of histogram models
 */
#pragma mark -  load features models
template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::loadFeatureModelsVFH (const std::string &directory, std::vector< model> &models)
{
    //Check if the path is good
    if (!boost::filesystem::exists (directory) && !boost::filesystem::is_directory (directory))
        return;
    
    boost::filesystem::path targetDir(directory);
    
    boost::filesystem::recursive_directory_iterator iter(targetDir), eod;
    
    BOOST_FOREACH(boost::filesystem::path const& i, std::make_pair(iter, eod)){
        if (is_regular_file(i)){
            //Check to take only the cloud and not the possible existing descriptor
            std::string pathToCheck =  i.string();
            // std::cout << pathToCheck << std::endl;
            //take only CVFH descriptors
            
            if (i.extension().string() == ".pcd" && pathToCheck.find("globaldesc")  != std::string::npos ){
                std::cout << "Path: " << pathToCheck << std::endl;
                model m;
                if (loadHistVFH(pathToCheck, m)){
                    std::cout << "VFH DESCRIPTOR LOADED " << std::endl;
                    models.push_back (m);
                }
                
            }
        }
    }
}

/** Load a set of ESF features that will act as the model (training data)
 * \param directory the path to the dataset
 * \param models the resultant vector of histogram models
 */
#pragma mark -  load features models
template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::loadFeatureModelsESF (const std::string &directory, std::vector<model> &models)
{
    //Check if the path is good
    if (!boost::filesystem::exists (directory) && !boost::filesystem::is_directory (directory))
        return;
    
    boost::filesystem::path targetDir(directory);
    
    boost::filesystem::recursive_directory_iterator iter(targetDir), eod;
    
    BOOST_FOREACH(boost::filesystem::path const& i, std::make_pair(iter, eod)){
        if (is_regular_file(i)){
            //Check to take only the cloud and not the possible existing descriptor
            std::string pathToCheck =  i.string();
            
            if (i.extension().string() == ".pcd" && pathToCheck.find("globaldesc")  != std::string::npos ){
                std::cout << "Path: " << pathToCheck << std::endl;
                model m;
                if (loadHistESF(pathToCheck, m)){
                    std::cout << "ESF DESCRIPTOR LOADED " << std::endl;
                    models.push_back (m);
                }
                
            }
        }
    }
}

#pragma mark -  build tree
/** Crawled a directory structure, looked at all the .PCD files found corresponding to the global descriptors, tested them whether they are VFH signatures (or others depending of the descriptors) and loaded them in memory; Then
 *  converted the data into FLANN format and dumped it to disk;
 *  Finally built a kd-tree structure and dumped it to disk.
 * @param databaseTrained where the dataset is. But need previously to compute all the global descriptors (cf trainDatabase() )
 */
template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::buildTree(const std::string databaseTrained){
    
    std::string desc_rgbd = "/desc_";
    std::string desc_structure_sensor = "/descriptor_";
    if (!boost::filesystem::exists(databaseTrained)){
        std::cerr << "[ERROR] Trained database not correct - Abort "<< std::endl;
        return;
    }
    boost::filesystem::path dataset_directory_path(databaseTrained);
    std::string typeDesc = "";
    // Choose to save in the same directory of the trained dataset
    std::string pathToSaveTreeStructure =  dataset_directory_path.c_str();
    
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    
    std::vector<model> models;
    
    //Loop over all the trained dataset
    int numberObjects = 0;
    for(boost::filesystem::recursive_directory_iterator it(dataset_directory_path); it!=boost::filesystem::recursive_directory_iterator(); ++it)
    {
        std::string path_current = it->path().c_str();
        //std::cout << "[INFO] Build tree current path: " << path_current << std::endl;
        
        //VFH / CVFH / OURCVFH descriptor case
        //get all the VFH features of the given directory to then build the KDtree
        if(typeid(DescriptorT) == typeid(pcl::VFHSignature308)){
            //Get only the VFH descriptors
            std::string queryStringDescriptor = "";
            if (m_descriptor_name == "vfh"){
                queryStringDescriptor = "/vfh"+desc_structure_sensor;
                typeDesc = "vfh";
            }else if (m_descriptor_name == "ourcvfh"){
                queryStringDescriptor = "/ourcvfh"+desc_structure_sensor;
                typeDesc = "ourcvfh";
            }
            else if (m_descriptor_name == "cvfh"){
                queryStringDescriptor = "/cvfh"+desc_structure_sensor;
                typeDesc = "cvfh";
            }else {
                std::cerr << " [ERROR] Descriptor type unknow " << std::endl;
                exit(1);
            }
            // std::cout << "Path current : " << path_current << std::endl;
            if (path_current.find(queryStringDescriptor)  != std::string::npos  && it->path().extension().string() == ".pcd" ){
                std::cout << "[INFO] Path descriptor type vfh: " << it->path() << std::endl;
                
                model m;
                if (loadHistVFH (path_current, m)){
                    std::cout << "[INFO] VFH DESCRIPTOR LOADED " << std::endl;
                    models.push_back (m);
                    numberObjects++;
                }
                
            }
            //ESF descriptor case
        }else if (typeid(DescriptorT) == typeid(pcl::ESFSignature640)){
            if (path_current.find("/esf"+desc_structure_sensor)  != std::string::npos  && it->path().extension().string() == ".pcd" ){
                std::cout << "Path esf: " << it->path() << std::endl;
                typeDesc = "esf";
                model m;
                if (loadHistESF (path_current, m)){
                    std::cout << "[INFO] ESF DESCRIPTOR LOADED " << std::endl;
                    models.push_back (m);
                    numberObjects++;
                }
            }
        }
        
    }
    if ((int)models.size () == 0){
        std::cerr << "[ERROR] No models available. Either the trained dataset path is wrong or either you didn't train yet the database.  Abort "<< std::endl;
        return;
    }
    
    //Name of the kdtree structure
    std::string kdtree_idx_file_name = "kdtree.idx";
    // Name of the training data in the H5 format (contains the path to all the descriptors)
    std::string training_data_h5_file_name = "training_data.h5";
    // Name of the training data list (contains the path to all the descriptors)
    std::string training_data_list_file_name = "training_data.list";
    
    
    std::cout << "[INFO] Loaded " << (int)models.size () << " models. Creating training data" << training_data_h5_file_name.c_str () << " | " << training_data_list_file_name.c_str() << std::endl;
    
    std::cout << "\n [INFO] Number of models available: " << models.size() << std::endl;
    
    
    
    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
    
    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];
    
    
    // Save data to disk (list of models)
    // structure-sensor-trained/training_vfh/training_data.h5
    
    
    std::string path_training_Descriptor = pathToSaveTreeStructure + "/" + "training_" + typeDesc + "/";
    
    std::string pathToSaveTraining = path_training_Descriptor + training_data_h5_file_name;
    //structure-sensor-trained/training_vfh/training_data.list
    std::string pathToSaveTrainingDataList = path_training_Descriptor + training_data_list_file_name;
    //structure-sensor-trained/training_vfh/kdtree.idx
    std::string pathToSaveKdtree = path_training_Descriptor +  kdtree_idx_file_name;
    
    if (!boost::filesystem::exists(pathToSaveTraining)){
        boost::filesystem::create_directory(path_training_Descriptor);
    }
    
    std::cout << "[INFO] Saving H5 file to " << pathToSaveTraining << std::endl;
    std::cout << "[INFO] Saving training data list file to " << pathToSaveTrainingDataList << std::endl;
    std::cout << "[INFO] Saving kd tree file to " << pathToSaveKdtree << std::endl;
    
    //Remove before saving
    if (boost::filesystem::exists(pathToSaveTraining)){
        boost::filesystem::remove(pathToSaveTraining);
        
    }
    if (boost::filesystem::exists(pathToSaveTrainingDataList)){
        boost::filesystem::remove(pathToSaveTrainingDataList);
        
    }
    
    
    flann::save_to_file (data, pathToSaveTraining, "training_data");
    std::ofstream fs;
    fs.open (pathToSaveTrainingDataList.c_str());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();
    
    
    
    // Build the tree index and save it to disk
    pcl::console::print_highlight("[INFO] Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
    
    
    
    if (typeDesc == "cvfh" || typeDesc == "ourcvfh"){
        flann::Index<DistT > index (data, KMeansIndexParams(32, 11,FLANN_CENTERS_RANDOM,0.2 ));
        //Constructs the nearest neighbor search index using the parameters provided to the constructor
        index.buildIndex ();
        
        if (boost::filesystem::exists(pathToSaveKdtree)){
            boost::filesystem::remove(pathToSaveKdtree);
            
        }
        index.save (pathToSaveKdtree);
        
        
    }else {
        
        if (typeDesc == "vfh"){
            flann::Index<DistT > index (data, flann::KDTreeIndexParams (8));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            if (boost::filesystem::exists(pathToSaveKdtree)){
                boost::filesystem::remove(pathToSaveKdtree);
                
            }
            index.save (pathToSaveKdtree);
            //It's esf
        }else {
            flann::Index<DistT > index (data, flann::KDTreeIndexParams (6));
            //Constructs the nearest neighbor search index using the parameters provided to the constructor
            index.buildIndex ();
            
            
            
            if (boost::filesystem::exists(pathToSaveKdtree)){
                boost::filesystem::remove(pathToSaveKdtree);
                
            }
            index.save (pathToSaveKdtree);
        }
    }
    
    
    
    delete[] data.ptr ();
    pcl::console::print_highlight("[INFO] DONE \n");
    
    
    
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::queryDBObject(std::string queryOjbect, int k, Hypotheses& clusters_hypotheses){
    // Read a PCD file from disk.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(queryOjbect, *cloud) != 0)
    {
        return;
    }
    
    /* ESFEstimation<pcl::PointXYZRGB> esf_estimation;
     //ESF
     esf_estimation.setInputCluster(*cloud);
     esf_estimation.estimate();
     pcl::PointCloud<pcl::ESFSignature640>::Ptr esfs(new pcl::PointCloud<pcl::ESFSignature640>);
     esf_estimation.getResultDescriptors(*esfs);*/
    
    VFHEstimation<pcl::PointXYZRGB> vfh_estimation;
    vfh_estimation.setInputCluster(*cloud);
    vfh_estimation.estimate();
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>);
    vfh_estimation.getResultDescriptors(*vfhs);
    
    
    queryDB(vfhs, k, clusters_hypotheses);
    
    
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::queryDB(DescriptorCloudPtrT query, int k, Hypotheses& clusters_hypotheses)
{
    clusters_hypotheses.clear();
    clusters_hypotheses.resize(query->size());
    for(unsigned int i = 0; i < clusters_hypotheses.size(); ++i)
        clusters_hypotheses[i].resize(k);
    
    std::vector<std::vector<int> > matched_k_indices;
    std::vector<std::vector<float> > matched_k_sqr_distances;
    flann_search_.nearestKSearch(*query, std::vector<int>(), k, matched_k_indices, matched_k_sqr_distances);
    
    for(unsigned int i = 0; i < clusters_hypotheses.size(); ++i)
    {
        for(unsigned int j = 0; j < k; ++j)
        {
            int index = matched_k_indices[i][j];
            clusters_hypotheses[i][j].type = names_[index];
            clusters_hypotheses[i][j].pose = poses_[index];
            clusters_hypotheses[i][j].fitness = matched_k_sqr_distances[i][index];
            clusters_hypotheses[i][j].cloud = views_[index];
        }
    }
}

// Define user defined literal "_quoted" operator.
std::string operator"" _quoted(const char* text, std::size_t len) {
    return "\"" + std::string(text, len) + "\"";
}


#pragma mark - globalMatchingUsingViewOfCloud
/**  Given a view query of an object, try to find the most similar in the database
 * @param queryCloud the view of an object (pcd format)
 * @param k the number of results we want
 */
template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::globalMatchingUsingViewOfCloud(const std::string queryCloud,const std::string dataset_trained,const std::vector<std::string> categories, int k){
    
    
    //std::size_t found = queryCloud.find_last_of("/\\");
    //std::string parentDirectory = queryCloud.substr(0,found);
    std::string parentDirectory = "..";
    boost::filesystem::path output_path(parentDirectory);
    
    std::string queryDescriptor = "";
    
    //std::cout << "[INFO] Output Path : " << output_path << std::endl;
    std::cout << "[INFO] Loading target cloud: " << queryCloud << " ... " << std::flush;
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile<PointT>(queryCloud, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file PCD file \n");
        std::cout << "[ERROR] Couldn't read file PCD file \n" << std::endl;
        
    }else {
        //std::cout << "[INFO] Loading Complete \n" << std::endl;
    }
    
    if (m_resolution_activated){
        std::cout << "[INFO] Using resolution : " << m_resolution << std::endl;
        std::cout << "Resolution of the uploaded model : " << Utils::compute_resolution(cloud) << std::endl;
        //Downsize for resolution invariance
        pcl::VoxelGrid<pcl::PointXYZ> down;
        float leaf = m_resolution;
        down.setLeafSize (leaf, leaf, leaf);
        down.setInputCloud (cloud);
        down.filter (*cloud);
        std::cout << "Resolution of the uploaded model after filtering : " << Utils::compute_resolution(cloud) << std::endl;
    }
    
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
        
        boost::filesystem::create_directory(output_path/"descriptors_view/esf");
        
        descriptor_name<<(output_path/"descriptors_view/esf/").string()<<"descriptor_0.pcd";
        pcl::io::savePCDFileASCII<pcl::ESFSignature640>(descriptor_name.str(), esfs);
        
    }else{
        std::cerr << "[ERROR] Unknow type of descriptor " << std::endl;
        exit(1);
    }
    
    
    
    queryDescriptor = descriptor_name.str();
    std::cout << "[INFO] Descriptor computation completed \n" << std::endl;
    
    
    std::cout << "[INFO] Matching stage ... " << std::endl;
    // Load the global descriptor of the target cloud
    model histogram;
    if(m_descriptor_name == "vfh" || m_descriptor_name == "cvfh" || m_descriptor_name == "ourcvfh"){
        std::cout << "[INFO]LOAD VFH type histogram " << std::endl;
        if (!loadHistVFH (queryDescriptor, histogram))
        {
            pcl::console::print_error (" [ERROR] Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
        //Suppose ESF
    }else {
        std::cout << "[INFO] LOAD ESF histogram " << std::endl;
        if (!loadHistESF (queryDescriptor, histogram))
        {
            
            pcl::console::print_error ("[ERROR] Cloud search - Cannot load global descriptor of the target file %s\n", queryCloud.c_str());
        }
    }
    
    std::cout << "[INFO]Using " << k << " nearest neighbors." <<  std::endl;
    
    std::string path_trained_dataset = dataset_trained + "training_"+m_descriptor_name+ "/";
    std::string kdtree_idx_file_name = path_trained_dataset + "kdtree.idx";
    std::string training_data_h5_file_name = path_trained_dataset + "training_data.h5";
    std::string training_data_list_file_name = path_trained_dataset + "training_data.list";
    
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
    
    std::string distance =  getDistanceName();
    m_resultsdetectionJSONformat = "[ \n";
    
    // Output the results on screen and output result to json file
    std::cout << "[RESULT] The closest " << k << "neighbors for "   << queryCloud << " are: " << std::endl;
    for (int i = 0; i < k; ++i){
        
        for (int j = 0;j< categories.size();j++){
            if (models.at (k_indices[0][i]).first.find(categories.at(j)) != std::string::npos) {
                std::cout << i << " - " << categories.at(j) << " (" << k_indices[0][i] <<")"<< " with a distance of: " << k_distances[0][i] << "\n";
                
                
                //Fill the map vector of results
                // m_result_cloudRecognition.insert( std::pair<std::string,float>(models.at (k_indices[0][i]).first.c_str (), k_distances[0][i]) );
                //m_result_cloudRecogni( std::make_pair(models.at (k_indices[0][i]).first.c_str (), k_distances[0][i]) );
                m_resultsdetectionJSONformat += "{" + "Number"_quoted + " :" + std::to_string(i) + ", " + "Category"_quoted + " :" + "\"" + categories.at(j) + "\"" +", " + "id"_quoted + ":" + std::to_string(k_indices[0][i]) + "," + "Distance"_quoted + ":" + std::to_string(k_distances[0][i]) + " , " + "metric"_quoted + ":" +"\""+ distance+"\"" + "}, \n";

                
                
                if (i == k-1){
                     m_resultsdetectionJSONformat += "{" + "Number"_quoted + " :" + std::to_string(i+1) + ", " + "Category"_quoted + " :" +  "\""+categories.at(j)+"\"" +", " + "id"_quoted + ":" + std::to_string(k_indices[0][i]) +"," + "Distance"_quoted + ":" + std::to_string(k_distances[0][i])  +" , " + "metric"_quoted + ":" + "\""+distance+"\"" + "}] \n";
                    
                }
            }
        }
    }
}

#pragma mark -  save JSON
template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::saveResultsDetectionsJSON(const string outputPath){
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
inline void RecognitionDatabase<PointT, DescriptorT, DistT>::nearestKSearch (flann::Index<DistT > &index, const model &model,int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
    // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
    memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
    
    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr ();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::writeFloat(std::string path, float num)
{
    std::ofstream fout(path.c_str());
    fout<<num;
    fout.close();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::writeMatrix4f(std::string path, Eigen::Matrix4f mat)
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
void RecognitionDatabase<PointT, DescriptorT, DistT>::readFloat(std::string path, float& num)
{
    std::ifstream fin(path.c_str());
    fin>>num;
    fin.close();
}

template<typename PointT, typename DescriptorT, typename DistT>
void RecognitionDatabase<PointT, DescriptorT, DistT>::readMatrix4f(std::string path, Eigen::Matrix4f& mat)
{
    std::ifstream fin(path.c_str());
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            fin>>mat(i,j);
    
    fin.close();
}
template<typename PointT, typename DescriptorT, typename DistT>
std::string RecognitionDatabase<PointT, DescriptorT, DistT>::getDistanceName()
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
    else if(typeid(DistT) == typeid(flann::HistIntersectionUnionDistance<float>)){
        m_distance_used = "HistogramIntersectionUnion";
    }
    else if(typeid(DistT) == typeid(flann::HellingerDistance<float>)){
        m_distance_used = "Hellinger";
    }
    else {
        m_distance_used = "Unknown";
    }
        
    return m_distance_used;
}


#endif
