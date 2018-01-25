//
//  OctomapHandler.hpp
//  RECONNAISSANCE
//
//  Created by Lirone Samoun on 27/04/2016.
//
//

#ifndef SVDESTIMATION_hpp
#define SVDESTIMATION_hpp

#include <stdio.h>
//EIGEN
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"

using namespace Eigen;

class SVDEstimation {
    
public:
    
    
public:
    /** \brief cconstructor
     */
    SVDEstimation(){};
    
    void compute();
    bool saveData(MatrixXf m,std::string path);
    
    void setMatrixInput(MatrixXf m){
        m_data = m;
    }
    void setMatrixU(MatrixXf m){
        m_U = m;
    }
    void setMatrixV(MatrixXf m){
        m_V = m;
    }
    void setVectorPropre(MatrixXf m){
        m_vectorPropre = m;
    }
    MatrixXf getMatrixU(){
        return m_U;
    }
    MatrixXf getMatrixV(){
        return m_V;
    }
    MatrixXf getVectorPropre(){
        return m_vectorPropre;
    }


protected:
    MatrixXf m_data;
    MatrixXf m_V;
    MatrixXf m_U;
    MatrixXf m_singulareValue;
    MatrixXf m_vectorPropre;
    
};

void SVDEstimation::compute(){
    JacobiSVD<MatrixXf> svd(m_data, ComputeThinU | ComputeThinV);
    MatrixXf m_V = svd.matrixV();
    setMatrixV(m_V);
    MatrixXf m_U = svd.matrixU();
    setMatrixV(m_U);
    MatrixXf vector_propre = svd.matrixV().transpose();
    setVectorPropre(vector_propre);
}

bool SVDEstimation::saveData(MatrixXf m,std::string path){
    std::ofstream file(path);
    if (file.is_open())
    {
        file << m << '\n';
        std::cout << "[INFO] Saving data...OK" << std::endl;
        return true;
    }else {
        return false;
    }
}

#endif /* SVDESTIMATION_hpp */
