
#ifndef SVM_TRAIN_H
#define SVM_TRAIN_H

#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <cerrno>
#include <iostream>
#include "libsvm/svm.h"


int svmTrain (double gamma, double C, float eps, std::vector<std::vector<svm_node> > data, std::vector<int> labels, const std::string path_to_saveModel);
void do_cross_validation();

#endif  // SVM_TRAIN_H
