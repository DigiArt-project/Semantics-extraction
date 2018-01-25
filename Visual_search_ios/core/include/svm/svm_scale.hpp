

#ifndef SVM_SCALE_H
#define SVM_SCALE_H

#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>


#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

void output_target(double value);
void output(int index, double value);
char* readline_scale(FILE *input);
int clean_up(FILE *fp_restore, FILE *fp, const char *msg);


bool scale(const std::string data_filename,const std::string path_to_saveScaleFileResult,const std::string path_to_saveRangeFile, bool restore,double lower = - 1, double upper = 1, bool scaling_limits = false);

#endif  // SVM_SCALE_H
