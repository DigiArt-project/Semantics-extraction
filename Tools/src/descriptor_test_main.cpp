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

#include "descriptor_estimation/gshot_pyramid_estimation.hpp"
//#include <pcl/features/gshot.h>
#include <pcl/features/normal_3d.h>
#include "descriptor_estimation/esf_estimation.hpp"
#include "descriptor_estimation/svd_estimation.hpp"
#include "descriptor_estimation/gshot_estimation.hpp"


//EIGEN
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ PointType;

//Read point cloud from a path
int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointType> > point_cloud)
{
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
        std::cout << "\n file extension is not correct. Syntax is: ./main <path/file_name.pcd> [--nogui] or ./main <path/file_name.ply>" << std::endl;
        return -1;
    }
    return 1;
}


VectorXd Compute(MatrixXd D)
{
    // The matrix must be square matrix.
    assert(D.rows() == D.cols());
    int N = D.rows();
    
    // 1. Compute the mean image
    MatrixXd mean(1, N);
    mean.setZero();
    
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            mean(0, j) += D(i, j) / N;
        }
    }
    
    // 2. Subtract mean image from the data set to get mean centered data vector
    MatrixXd U = D;
    
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            U(i, j) -= mean(0, j);
        }
    }
    
    // 3. Compute the covariance matrix from the mean centered data matrix
    MatrixXd covariance = (U.transpose() * U) / (double)(N);
    
    // cout << covariance << endl;
    
    // 4. Calculate the eigenvalues and eigenvectors for the covariance matrix
    EigenSolver<MatrixXd> solver(covariance);
    MatrixXd eigenVectors = solver.eigenvectors().real();
    VectorXd eigenValues = solver.eigenvalues().real();
    
    // 5. Normalize the eigen vectors
    eigenVectors.normalize();
    
    // cout << eigenVectors << endl;
    // cout << eigenValues << endl;
    
    // 6. Find out an eigenvector with the largest eigenvalue
    //    which distingushes the data
    sort(eigenValues.derived().data(), eigenValues.derived().data() + eigenValues.derived().size());
    short index = eigenValues.size() - 1;
    VectorXd featureVector = eigenVectors.row(index);
    
    return featureVector;
}

std::vector<float> minMaxScaler(std::vector<float> data){
    std::vector<float> result_min_max;
    auto max = std::max_element(std::begin(data), std::end(data));
    auto min = std::min_element(std::begin(data), std::end(data));
    for (int i = 0; i < data.size(); i++){
        float new_value = (data.at(i) - *min)/(*max - *min);
        result_min_max.push_back(new_value);
    }
    return result_min_max;
}
void test(){
    MatrixXd m = MatrixXd::Random(1,10);
    cout << "Here is the matrix m:" << endl << m << endl;
    JacobiSVD<MatrixXd> svd(m, ComputeThinU | ComputeThinV);
    cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    cout << "B = U * S :" <<  svd.matrixU() * svd.singularValues().asDiagonal() << endl;
    
    MatrixXd vector_propre = svd.matrixV().transpose();
    cout << "Vector propre : " << vector_propre << "\n "<< endl;
    cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
    MatrixXd Cp = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    cout << "\n" << Cp << endl;
    MatrixXd diff = Cp - m;
    cout << "diff:\n" << diff.array().abs().sum() << "\n";
    cout << "First column" << endl;
    cout << vector_propre.block<1,3>(0,0) << endl << endl;

}

void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                   Compute descriptor of point cloud                     *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    
    std::cout << "Usage: " << filename << "-cloud pointCloud" << std::endl << std::endl;
    
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
    if (argc < 3)
    {
        std::cout << "[INFO] Not enough parameters " << std::endl;
        showHelp (argv[0]);
        return (-1);
    }

    std::string cloud_path;
    if(pcl::console::parse_argument(argc, argv, "-cloud", cloud_path) == -1)
    {
        std::cerr<<"Please specify the cloud"<<std::endl;
        return -1;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (readPointCloud(cloud_path,  cloud)==-1)
        return -1;
    
    std::cout << "Loaded "
    << cloud->width * cloud->height
    << std::endl;
    
    /*
    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (cloud);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod (tree);
    
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
    normalEstimation.setRadiusSearch (0.03);
    normalEstimation.compute (*normals);
    
    // Setup the SHOT features
    //typedef pcl::FPFHSignature33 ShotFeature; // Can't use this, even despite: http://docs.pointclouds.org/trunk/structpcl_1_1_f_p_f_h_signature33.html
    typedef  pcl::SHOT352 ShotFeature;
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, ShotFeature> shotEstimation;
    shotEstimation.setInputCloud(cloud);
    shotEstimation.setInputNormals(normals);
    
    // Use the same KdTree from the normal estimation
    shotEstimation.setSearchMethod (tree);
    pcl::PointCloud<ShotFeature>::Ptr shotFeatures(new pcl::PointCloud<ShotFeature>);
    //spinImageEstimation.setRadiusSearch (0.2);
    shotEstimation.setKSearch(0);
    shotEstimation.setRadiusSearch( 0.2 );
    
    // Actually compute the spin images
    shotEstimation.compute (*shotFeatures);
    std::cout << "SHOT output points.size (): " << shotFeatures->points.size () << std::endl;
    
    // Display and retrieve the SHOT descriptor for the first point.
    ShotFeature descriptor = shotFeatures->points[0];
    std::cout << descriptor << std::endl;*/
    /*
    GSHOTEstimation<pcl::PointXYZ> gshot_estimation;
    gshot_estimation.setInputCluster(*cloud);
    gshot_estimation.estimate();
    pcl::PointCloud<pcl::SHOT352> gshots;
    gshot_estimation.getResultDescriptors(gshots);
    std::cout << "Before scale : " << gshots.points[0] << std::endl;
    std::cout << "\n" << std::endl;
    std::vector<float> data_tmp,value_descriptor_scaled;
    for (size_t i = 0; i < pcl::SHOT352::descriptorSize(); i++)
    {
        data_tmp.push_back(gshots.points[0].descriptor[i]);
    
    }
    
    value_descriptor_scaled = minMaxScaler(data_tmp);
    for (int j = 0; j < value_descriptor_scaled.size(); j++){
        std::cout << value_descriptor_scaled.at(j) << std::endl;
        gshots.points[0].descriptor[j] = value_descriptor_scaled.at(j);
    }
    std::cout << "Size : " << gshots.points[0].descriptorSize() << std::endl;
    std::cout << "After scale : " << gshots.points[0] << std::endl;

*/
    //pcl::io::savePCDFileASCII<pcl::SHOT352>("test_descriptor.pcd", gshots);

    /*
    ESFEstimation<pcl::PointXYZ> esf_estimation;
    esf_estimation.setInputCluster(*cloud);
    esf_estimation.estimate();
    pcl::PointCloud<pcl::ESFSignature640> esfs;
    esf_estimation.getResultDescriptors(esfs);
     */
    
    
    /*
    MatrixXf m(1, pcl::SHOT352::descriptorSize());
    for (int j = 0; j < pcl::SHOT352::descriptorSize() ;j++){
        float feature_value = gshots.points[0].histogram[j];
        //std::cout << feature_value << std::endl;
        m(0,j) = feature_value;

    }

    JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
    MatrixXf vector_propre = svd.matrixV().transpose();
    cout << vector_propre.block<1,100>(0,0) << endl << endl;
    
    SVDEstimation svd_est;
    svd_est.setMatrixInput(m);
    svd_est.compute();
    MatrixXf vector_propre = svd_est.getVectorPropre();
    cout << vector_propre.block<1,100>(0,0) << endl << endl;
    
    bool ok = svd_est.saveData(vector_propre.block<1,100>(0,0), "test.txt");
    
    //cout << "MATRIX : " << m << endl;
    
    /*
    // The number of rows in the input matrix
    int rows = 3;
    
    // The number of columns in the input matrix
    int columns = 3;
    
    // Initialize the matrix
    MatrixXd m(rows, columns);
    
    // Set the matrix values.
    m << 1, 2, 3,
    4, 5, 6,
    7, 8, 9;
    
    // Compute the principal component and show results.
    cout << Compute(m) << endl;*/
    
     GSHOTPyramidEstimation<pcl::PointXYZ> gshot_pyramid_estimation;
     gshot_pyramid_estimation.setInputCloud(*cloud);
     gshot_pyramid_estimation.estimate();
    pcl::PointCloud<pcl::SHOT352> shots = gshot_pyramid_estimation.getResultDescriptors();


  
    return 0;

    
}


