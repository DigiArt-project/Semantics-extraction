#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#define GetCurrentDir getcwd
#include "svm_predict.hpp"

static int (*infos)(const char *fmt,...) = &printf;

struct svm_node *x;
int max_nr_attr = 64;
struct svm_model* model_s;
static char *line1 = NULL;
static int max_line_len1;

std::string GetCurrentWorkingDir( void ) {
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}


bool doScalingVFH(const std::string path_to_rangeFile, const std::string path_to_scale_program,  const pcl::PointCloud<pcl::VFHSignature308> vfhs){
    int size_descriptor = vfhs.points[0].descriptorSize();
    std::string name_tmp_desc = "tmp_descriptor.txt";
    std::string name_tmp_desc_scale = "tmp_descriptor.scale";
    //../../svm/dataset_descriptor_training_esf_libsvm_ratio3.txt.range
    std::cout << "Path to range : " << path_to_rangeFile << std::endl;
    if (!boost::filesystem::exists (path_to_rangeFile)){
        std::cerr << "[ERROR] Are you sure the range file is here and you scaled the training data ? " << std::endl;
        exit(1);
    }
    if (boost::filesystem::exists (name_tmp_desc)){
        remove(name_tmp_desc.c_str());
    }
    if (boost::filesystem::exists ("tmp_descriptor.scale")){
        remove("tmp_descriptor.scale");
    }
    std::cout << "[INFO] Writting VFH type descriptor to a file in order to scale it... " << std::flush;
    std::vector<float> tmp_vector_descriptor;
    std::ofstream file;
    //Create a temporary file where we are going to write the feature descriptor
    file.open(name_tmp_desc, ios::out | ios::app );
    file <<"1 ";
    for (int i = 0; i < size_descriptor;i++){
        tmp_vector_descriptor.push_back(vfhs.points[0].histogram[i]);
        //   std::cout << "Value: " << vfhs.points[0].histogram[i] << std::endl;
        file << i+1<<":"<<vfhs.points[0].histogram[i] << " ";
        
    }
    file << "\n";
    
    file.close();
    std::cout << "Completed" << std::endl;
    //Scale the file
    // create a string, i.e. an array  of 100 char for the command line
    char command[100];
    sprintf (command, "../core/3rdparty/libsvm/svm-scale -r %s %s >> %s", path_to_rangeFile.c_str(), name_tmp_desc.c_str(), name_tmp_desc_scale.c_str());
    //sprintf (command, "./svm-scale -r %s %s ", path_to_rangeFile.c_str(), name_tmp_desc.c_str());
    // system call
    system(command);
    
    //Read the scaled file to be sure it's okay
    if (!std::ifstream(name_tmp_desc_scale))
    {
        std::cerr << "[ERROR]" << name_tmp_desc_scale << " could not be created" << std::endl;
        return false;
        
    }
    //Remove tmp descriptor txt
    remove(name_tmp_desc.c_str());
    
    
    return true;
    
}



bool doScalingESF(const std::string path_to_rangeFile, const std::string path_to_scale_program, const pcl::PointCloud<pcl::ESFSignature640> esfs){
    std::cout << "[Config] Path to svm scale : " << path_to_scale_program << std::endl;
    std::string file_scale = "svm-scale";
    //std::cout << "Current working dir : " << GetCurrentWorkingDir() << std::endl;
    if (!boost::filesystem::exists (path_to_scale_program)){
        std::cerr << "[ERROR] " << path_to_scale_program << " Scaling program does not exist" << std::endl;
        exit(1);
    }else {
          //std::cerr << "[DO SCALING] " << file_scale << " pOK" << std::endl;
    }

    int size_descriptor = esfs.points[0].descriptorSize();
    std::string name_tmp_desc = "tmp_descriptor.txt";
    std::string name_tmp_desc_scale = "tmp_descriptor.scale";
    //../../svm/dataset_descriptor_training_esf_libsvm_ratio3.txt.range
    //std::cout << "Path to range : " << path_to_rangeFile << std::endl;
    if (!boost::filesystem::exists (path_to_rangeFile)){
        std::cerr << "[ERROR] Are you sure the range file is here and you scaled the training data ? " << std::endl;
        exit(1);
    }
    if (boost::filesystem::exists (name_tmp_desc)){
        remove(name_tmp_desc.c_str());
    }
    if (boost::filesystem::exists ("tmp_descriptor.scale")){
        remove("tmp_descriptor.scale");
    }
    std::cout << "[INFO] Writting ESF type descriptor to a file in order to scale it... " << std::flush;
    std::vector<float> tmp_vector_descriptor;
    std::ofstream file;
    //Create a temporary file where we are going to write the feature descriptor
    file.open(name_tmp_desc, ios::out | ios::app );
    file <<"1 ";
    for (int i = 0; i < size_descriptor;i++){
        tmp_vector_descriptor.push_back(esfs.points[0].histogram[i]);
        //   std::cout << "Value: " << vfhs.points[0].histogram[i] << std::endl;
        file << i+1<<":"<<esfs.points[0].histogram[i] << " ";
        
    }
    file << "\n";
    
    file.close();
    std::cout << "Completed" << std::endl;
    //Scale the file
    // create a string, i.e. an array  of 100 char for the command line
    char command[100];
    sprintf (command, "%s -r %s %s >> %s",path_to_scale_program.c_str(), path_to_rangeFile.c_str(), name_tmp_desc.c_str(),name_tmp_desc_scale.c_str());
    //sprintf (command, "./svm-scale -r %s %s ", path_to_rangeFile.c_str(), name_tmp_desc.c_str());
    // system call
    system(command);

    
    //Read the scaled file to be sure it's okay
    if (!std::ifstream(name_tmp_desc_scale))
    {
        std::cerr << "[ERROR]" << name_tmp_desc_scale << " could not be created" << std::endl;
        return false;
        
    }
    
    if (boost::filesystem::exists (name_tmp_desc)){
        remove(name_tmp_desc.c_str());
    }

    
    return true;
    
}

bool doScalingGSHOT(const std::string path_to_rangeFile, const std::string path_to_scale_program, const pcl::PointCloud<pcl::SHOT352> gshots){
    std::cout << "[Config] Path to svm scale : " << path_to_scale_program << std::endl;
    std::string file_scale = "svm-scale";
    //std::cout << "Current working dir : " << GetCurrentWorkingDir() << std::endl;
    if (!boost::filesystem::exists (path_to_scale_program)){
        std::cerr << "[ERROR] " << path_to_scale_program << " Scaling program does not exist" << std::endl;
        exit(1);
    }else {
        //std::cerr << "[DO SCALING] " << file_scale << " pOK" << std::endl;
    }
    
    int size_descriptor = gshots.points[0].descriptorSize();
    std::string name_tmp_desc = "tmp_descriptor.txt";
    std::string name_tmp_desc_scale = "tmp_descriptor.scale";
    //../../svm/dataset_descriptor_training_esf_libsvm_ratio3.txt.range
    //std::cout << "Path to range : " << path_to_rangeFile << std::endl;
    if (!boost::filesystem::exists (path_to_rangeFile)){
        std::cerr << "[ERROR] Are you sure the range file is here and you scaled the training data ? " << std::endl;
        exit(1);
    }
    if (boost::filesystem::exists (name_tmp_desc)){
        remove(name_tmp_desc.c_str());
    }
    if (boost::filesystem::exists ("tmp_descriptor.scale")){
        remove("tmp_descriptor.scale");
    }
    std::cout << "[INFO] Writting GSHOT type descriptor to a file in order to scale it... " << std::flush;
    std::vector<float> tmp_vector_descriptor;
    std::ofstream file;
    //Create a temporary file where we are going to write the feature descriptor
    file.open(name_tmp_desc, ios::out | ios::app );
    file <<"1 ";
    for (int i = 0; i < size_descriptor;i++){
        tmp_vector_descriptor.push_back(gshots.points[0].descriptor[i]);
        //   std::cout << "Value: " << vfhs.points[0].histogram[i] << std::endl;
        file << i+1<<":"<<gshots.points[0].descriptor[i] << " ";
        
    }
    file << "\n";
    
    file.close();
    std::cout << "Completed" << std::endl;
    //Scale the file
    // create a string, i.e. an array  of 100 char for the command line
    char command[100];
    sprintf (command, "%s -r %s %s >> %s",path_to_scale_program.c_str(), path_to_rangeFile.c_str(), name_tmp_desc.c_str(),name_tmp_desc_scale.c_str());
    //sprintf (command, "./svm-scale -r %s %s ", path_to_rangeFile.c_str(), name_tmp_desc.c_str());
    // system call
    system(command);
    
    
    //Read the scaled file to be sure it's okay
    if (!std::ifstream(name_tmp_desc_scale))
    {
        std::cerr << "[ERROR]" << name_tmp_desc_scale << " could not be created" << std::endl;
        return false;
        
    }
    
    if (boost::filesystem::exists (name_tmp_desc)){
        remove(name_tmp_desc.c_str());
    }
    
    
    return true;
    
}



char* readline(FILE *input)
{
	int len;

	if(fgets(line1,max_line_len1,input) == NULL)
		return NULL;

	while(strrchr(line1,'\n') == NULL)
	{
		max_line_len1 *= 2;
		line1 = (char *) realloc(line1,max_line_len1);
		len = (int) strlen(line1);
		if(fgets(line1+len,max_line_len1-len,input) == NULL)
			break;
	}
	return line1;
}

void exit_input_error(int line_num)
{
	fprintf(stderr,"Wrong input format at line %d\n", line_num);
	exit(1);
}

int predict(FILE *input, FILE *output)
{
    		double target_label, predict_label;
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

	int svm_type=svm_get_svm_type(model_s);
	int nr_class=svm_get_nr_class(model_s);
	
	max_line_len1 = 1024;
	line1 = (char *)malloc(max_line_len1*sizeof(char));
	while(readline(input) != NULL)
	{
		int i = 0;

		char *idx, *val, *label, *endptr;
		int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

		label = strtok(line1," \t\n");
		if(label == NULL) // empty line
			exit_input_error(total+1);

		target_label = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			exit_input_error(total+1);

		while(1)
		{
			if(i>=max_nr_attr-1)	// need one more for index = -1
			{
				max_nr_attr *= 2;
				x = (struct svm_node *) realloc(x,max_nr_attr*sizeof(struct svm_node));
			}

			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;
			errno = 0;
			x[i].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index)
				exit_input_error(total+1);
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				exit_input_error(total+1);

			++i;
		}
		x[i].index = -1;

        predict_label = svm_predict(model_s,x);
        fprintf(output,"%g\n",predict_label);
    

		if(predict_label == target_label)
			++correct;
		error += (predict_label-target_label)*(predict_label-target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label*predict_label;
		sumtt += target_label*target_label;
		sumpt += predict_label*target_label;
		++total;
	}
	if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
	{
		infos("Mean squared error = %g (regression)\n",error/total);
		infos("Squared correlation coefficient = %g (regression)\n",
			((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
			((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
			);
	}
    
    return predict_label;
	
}


int doPrediction(const std::string tmp_descriptor_scale,const std::string svm_m, const std::string out){
    bool useProb = true;
    
    if(useProb)
    {
       
    }
    
    if (!boost::filesystem::exists (svm_m)){
        std::cerr << "[ERROR] SVM model does not exist " << std::endl;
        exit(1);
    }
    FILE *input, *output;

    model_s=svm_load_model(svm_m.c_str());
    input = fopen(tmp_descriptor_scale.c_str(),"r");
    if(input == NULL)
    {
        fprintf(stderr,"can't open input file %s\n",tmp_descriptor_scale.c_str());
        exit(1);
    }
    output = fopen(out.c_str(),"w");
    if(output == NULL)
    {
        fprintf(stderr,"can't open output file %s\n",out.c_str());
        exit(1);
    }
    x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
    
    int label = predict(input,output);
    //std::cout << "Label predicted final : " << label << std::endl;
    svm_free_and_destroy_model(&model_s);
    free(x);
    free(line1);
    fclose(input);
    fclose(output);
    
    return label;
    return 0;

}
