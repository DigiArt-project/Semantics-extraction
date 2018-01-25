
#include "svm_train.hpp"


struct svm_model* model;
struct svm_problem prob;
struct svm_node* x_space;
struct svm_parameter param;

/**  train a svm
 * @param data data (libsvm format)
 * @param labels vectors of labels
 * @param path_to_saveModel path to save the SVM model
 */

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
int
svmTrain (double gamma, double C, float eps, std::vector<std::vector<svm_node> > data, std::vector<int> labels, const std::string path_to_saveModel)
{
    
  // Make sure we have data
  if (data.empty ())
  {
      std::cerr << "[ERROR] No training data\n" << std::endl;
    return (-1);  
  }
  
    //l is the number of training data
  prob.l = data.size ();
    //`y' is an array containing their target values. (integers in classification, real numbers in regression)
  prob.y = Malloc (double, prob.l);
    //`x' is an array of pointers, each of which points to a sparse representation (array of svm_node) of one training vector
  prob.x = Malloc (struct svm_node* , prob.l);
  x_space = Malloc (struct svm_node, data.size () * data[0].size ());
    
  
  
  int j = 0;
  for (int i = 0; i < prob.l; ++i)
  {
    prob.y[i] = *(labels.begin () + i);
    prob.x[i] = &x_space[j];
    for (std::vector<svm_node>::iterator it = data[i].begin (); it != data[i].end (); ++it)
    {
      x_space[j].index = it->index;
      x_space[j].value = it->value;
      ++j;
    } 
  }

  param.svm_type = C_SVC;
  param.kernel_type = RBF;
  param.gamma = gamma; 
  param.degree = 3;
  param.coef0 = 0;
  param.nu = 0.5;
  param.cache_size = 100;
  param.C = C;
  param.eps = eps;
  param.p = 0.1;
  param.shrinking = 1;
  param.probability = 0;
  param.nr_weight = 0;
  param.weight_label = NULL;
  param.weight = NULL;

    
  // Check whether the parameters are within a feasible range of the problem
  const char* error_msg = svm_check_parameter (&prob, &param);
  if (error_msg)
  {
      printf ("Error checking SVM parameters: %s\n", error_msg);
    return (-1);
  }

  model = svm_train (&prob, &param);
  if (svm_save_model(path_to_saveModel.c_str(), model))
  {
    std::cerr << "[ERROR] Save SVM model failed\n" << std::endl;
    return (-1);
  }
  else {
      std::cout << "Model SVM has been saved to " << path_to_saveModel << std::endl;
  }

  svm_free_and_destroy_model (&model);
  svm_destroy_param (&param);
  free (prob.y);
  free (prob.x);
  free (x_space);

  return (0);
}

