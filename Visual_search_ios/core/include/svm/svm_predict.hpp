

#ifndef SVM_PREDICT_H
#define SVM_PREDICT_H

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "libsvm/svm.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "others/utils.hpp"
#include "recognition_database/hypothesis.h"
#include "descriptor_estimation/vfh_estimation.h"
#include "descriptor_estimation/cvfh_estimation.h"
#include "descriptor_estimation/ourcvfh_estimation.h"
#include "descriptor_estimation/esf_estimation.h"

#include "configuration/configuration.hpp"
#include "svm_scale.hpp"


#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

void output_target(double value);
void output(int index, double value);
char* readline_scale(FILE *input);
int clean_up(FILE *fp_restore, FILE *fp, const char *msg);

bool doScalingESF(const std::string path_to_rangeFile,const std::string path_to_scale_program, const pcl::PointCloud<pcl::ESFSignature640> esfs);
bool doScalingVFH(const std::string path_to_rangeFile,const std::string path_to_scale_program, const pcl::PointCloud<pcl::VFHSignature308> vfhs);
bool doScalingGSHOT(const std::string path_to_rangeFile,const std::string path_to_scale_program, const pcl::PointCloud<pcl::SHOT352> gshots);
int doPrediction(const std::string tmp_descriptor_scale,const std::string svm_m,const std::string output);

//double svmESFPredictClass(const struct svm_model*,const pcl::PointCloud<pcl::ESFSignature640> vfhs, bool scaling);
double svmCovariancePredictClass (const struct svm_model*, const Eigen::MatrixXd &);

#endif  // SVM_PREDICT_H
