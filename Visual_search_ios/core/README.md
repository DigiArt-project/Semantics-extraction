# Similarity Search, SVM training and testing 

In this readme, we will see how to perform classification using **similarity search** and **svm**.
We will use two database : 

* **Structure sensor dataset** : A dataset which has been created manually from the structure sensor device
* **RGBD Dataset** : an object RGBD dataset from this [institute](https://rgbd-dataset.cs.washington.edu/).

## Dependency Requirements

* BOOST
* OPEN MP
* PCL
* OpenCV
* libsvm
* HDF5
* FLANN

## How to build

On the directory, run the following command line in the terminal:`cmake -G Xcode .`

Then, click on the ==core.xcodeproj== generated. Build the project using XCode or another IDE.
All the binaries files will be generated.

In the config.cfg, many parameters can be changed. These parameters are under categories.
1.	`path_dataset`  path to the datasets.
2.	`SimilaritySearch`  correspond to the similarity search step. The user can changed the virtual rendering parameters but as well the type of descriptor we want to use for building the index KDTree structure.3.	`Filter`  the filter resolution in order to have cloud resolution invariance. Each cloud is down sampled using the indicated resolution.4.	`Descriptors`  descriptors for the rendering
5. `Classification` –> related to svm 

## Content of the package

Here the necessary binary files (with a brief explanation) which have been compiled. Can be compiled again if you want to modify the source code.
### C++ binary

* **`svm_prediction_main`** —> Predict the label given a svm model file, a pcd file (point cloud to test) and the descriptor we wan to use
* **`svm_scale`** —> scale data between [-1,+1] for svm training and testing 
* **`svm_train`** —> Train a svm model given a training txt file which contains path to descriptors we want to train
* **`transform_point_cloud_main`**  —> Given a pcd point cloud and a transformation (translation and/or rotation), transform the point cloud.
* **`pcl_obj2ply `** —> Convert a obj point cloud to a ply point cloud
* **`retrieval_similaritySearch`**—> Given a query point cloud point and the path to the KDTree index structure, retrieve the k top results using k nearest neighbors
* **`train_similaritySearch `**—> Generate multiple views for each object and build KDTree index structure for similarity search

### Python binary
* **`checkdata.py `**—> Given a file, check if the data is well formatted for using it with libsvm
* **`grid.py `**—>  Cross validation process for svm
* **`easy.py `**—>  Perfom in one shot, the scaling, training and testing for svm using cross validation
* **`subset.py `**—> take a subset of a given dataset

### Bash script

* **`generate_files_database.sh `**—> Given a dataset, generate a txt file which contains the categories and absolute path to 3D objects from the dataset
* **`generate_files_descriptors_training_testing.sh `**—> Generate two files : training and testing of the descriptors based on a given ratio (for example with a ratio of 3, 1/3 descriptor will be on the testing file)
* **`generate_file_descriptors_oneshot.sh `** --> from a folder which contains the 3D descriptors, generate a txt file which contains the absolute path of 3D descriptors.
* **`generate_training_testing_libsvm_global.sh `** --> A one shot script which generate training and testing file for the ESF, CVFH, VFH and OURCVFH descriptors with ratio 2,3 and 5.
* **`obj2ply.sh `** --> Given a dataset, take every obj point cloud and convert them to ply point cloud* **`transform_point_cloud_all.sh `** --> From a given dataset, get all the points cloud and perform a given transformation (rotation and/or translation)

# Similarity Search training and testing 

##With the structure sensor dataset

#### 1. Generate a txt file which contains the category and the absolute path of 3D object of a given dataset

**Script**:
`./generate_files_database.sh dataset_path format_file`

The dataset must have the following structure : 

Dataset Directory/categoryi/objectj/*.ply (or *.pcd) for i [1...N] N number of categories and for j [1...M] number of objects inside the category

**Example**: `./generate_file_database.sh structure-sensor_dataset/ ply`
Here we generate a txt file from a dataset called structure sensor dataset which contains 3D object in PLY format.
==Return dataset_categories.txt file== 

#### 2a. Generate multiple views from an object (get a collection of partial snapshots of the model using a virtual scanner)

Given a txt file which contains all the absolute path of 3D object from a dataset (thank to the previous script), this script generate multiple views from each of the object.Then, for each view, compute 3D descriptors (ESF, VFH, CVFH, OURCVFH). All the descriptors are saved locally abd every view can be saved as well.

**Script**:
`./compute_views_descriptor_cloud_main config_file path_to_datasetsCategories path_to_save_results`

**Example**:  `./compute_views_descriptor_cloud_main ../config.cfg ../Dataset/ dataset_categories.txt ../Dataset/trained_dataset`

_Note_ : In the config.cfg file, we can change several parameters for the rendering step (views computation).
Need as well to precise the relative path of the structure sensor dataset.
Once the script has been executed, new files apparead in trained dataset with the following structure:

objecti/descriptors/cvfh(esf,ourcvfh,vfh)  
objecti/enthropies  
objecti/poses  
objecti/views  
objecti.pcd
In config.cfg, need to activate `enable_rendering = true` and to enable all the descriptors in order to compute each descriptor for each view generated. For example `enable_vfh = true` 

#### 2b. Generate FLANN index Tree for similarity search for all the descriptors. It's allow to retrieve faster descriptors

**Script**:
`./train_similaritySearch config_file path_to_datasets path_to_trainedDataset`

**Example**:  `./train_similaritySearch ../config.cfg -database ../../Datasets/structure-sensor-dataset/ - trained ../../Datasets/trained_dataset`

In config.cfg, need to activate `enable_rendering = false`, `build_tree = true`, `descriptor = vfh` (choose the one you want) and to enable all the descriptors in order to compute each descriptor for each view generated. For example `enable_vfh = true` 
==Instead of doing 2a and 2b, can make the whole process with only one step by doing
`./train_similaritySearch ../config.cfg -database ../../Datasets/structure-sensor-dataset/ -trained ../../ Datasets/trained_dataset`==by enabling `enable_rendering = true` and `build_tree = true`

==At this step, on a given dataset, all the descriptors for every views of every objects are generated and computed==

#### 3. (Optional) Generate a txt file which contain the absolute path of every 3D descriptors computed previously
The descriptors dataset must have the following structure  
trainedDataset/objecti/descriptors/nameDescriptor/descriptorj.pcd with i [0...N] N number of objects , j [0...M] M number descriptor corresponding to the object  
nameDescriptor is esf, vhf, cvfh, or our-cvfh

**Script**:
`./generate_file_descriptors.sh trained_dataset type_descriptor`**Example**:
`../generate_file_descriptors.sh trained_dataset esf`==Return one file : `dataset_descriptor_typeDescriptor.txt`==

#### 4. Testing retrieval similarity search

**Script**:
`./retrieval_similaritySearch config_file (.cfg) -query object.pcd -trained tained_dataset [Options]`**Example**:
`../retrieval_similaritySearch ../../training_database/config.cfg -query ../structure-sensor- dataset/mug/mug_1/Model.pcd -trained ../trained-struture-sensor/ -descriptor esf`

==Return of k best results==

##With the RGBD dataset

#### 1. Generate a txt file which contains the category and the absolute path of 3D object of a given dataset

**Script**:
`./generate_files_database.sh dataset_path format_file`

The dataset must have the following structure : 

Dataset Directory/categoryi/objectj/*.ply (or *.pcd) for i [1...N] N number of categories and for j [1...M] number of objects inside the category

**Example**: `./generate_file_database.sh structure-sensor_dataset/ ply`
Here we generate a txt file from a dataset called structure sensor dataset which contains 3D object in PLY format.
==Return dataset_categories.txt file== 

#### 2. Compute global descriptors of every views of objects in the RGBD dataset 

Given a txt file which contains all the absolute path of 3D object from a dataset (thank to the previous script), this script generate multiple views from each of the object.

**Script**:
`./compute_descriptors_RGBD_dataset_main config_file path_to_datasetsCategories path_to_save_results`

**Example**:  `./compute_descriptors_RGBD_dataset_main ../config.cfg ../../dataset_categories.txt ../../ descriptor-rgbd-dataset`

In config.cfg, need to activate `enable_rendering = true` . For perfomance reasons, one descriptor each time. All the descriptors **cannot be activated at the same time** like in the structure sensor dataset.

==Every descriptors of every views of objects have been generated at this step==

#### 3. Generate a txt file which contain the absolute path of every 3D descriptors computed previously
The descriptors dataset must have the following structure  
trainedDataset/objecti/descriptors/nameDescriptor/descriptorj.pcd with i [0...N] N number of objects , j [0...M] M number descriptor corresponding to the object  
nameDescriptor is esf, vhf, cvfh, or our-cvfh

**Script**:
`./generate_file_descriptors.sh trained_dataset type_descriptor`**Example**:
`../generate_file_descriptors.sh trained_dataset esf`==Return one file : `dataset_descriptor_typeDescriptor.txt`==

#### 4. Testing retrieval similarity search

**Script**:
`./retrieval_similaritySearch config_file (.cfg) -query object.pcd -trained tained_dataset [Options]`**Example**:
`../retrieval_similaritySearch ../../training_database/config.cfg -query ../structure-sensor- dataset/mug/mug_1/Model.pcd -trained ../trained-struture-sensor/ -descriptor esf`

==Return of k best results==



# SVM training and testing 

For SVM we use:

* cross validation of 5 (can be changed)
* RBF Kernel 
* LibSVM library
* Multi class One Again One (done by libSVM)

## With structure sensor dataset

#### 1. Instead of generating one file which contains all the descriptors, we generate here two files, one training and one testing 
Allow to split our dataset in training and testing for svm

**Script**:
`./generate_file_descriptors_training_testing.sh trained_dataset type_descriptor ratio output`

**Example**:
`./generate_file_descriptors_training_testing.sh trained_dataset/ esf 2 dataslibsvm`
Here, we have a reatio of 2 meaning that the script will take 1 descriptor over 2 to put it in the training file and the other 1/2 will be on the testing.
dataslibsvm is the folder where we are going to save the lib svm data file.
==Return two files : datasetDescriptorTrainingTypeDescriptor.txt and datasetDescriptorTestingTypeDescriptor.txt==

#### 2. Transformation of the 3D descriptors into a good format in order to use them in our SVM using libsvm

So far, thank to the first step, we have two txt file which contains respectively path to descriptors for training and path to descriptors for testing.  
Now we need to get the content of every descriptors and put them successively in one global training file and one global testing file compatible with libSVM that is to say :  
**label featureId1:featureValue1 featureId2:featureValue2 ...**  ...

**Script**:`./generate_dataset_libsvm_main input output`

**Example**:`./generate_dataset_libsvm_main ../../script/dataset_descriptor_training_esf.txt ../../script/ dataset_descriptor_training_esf_libsvm.txt esf`
`./generate_dataset_libsvm_main ../../script/dataset_descriptor_testing_esf.txt ../../script/ dataset_descriptor_testing_esf_libsvm.txt esf`

==Return two files datasetDescriptorTrainingTypeDescriptorLibsvm.txt and datasetDescriptorTestingTypeDescriptorLibsvm.txt==

**_NOTE_:**
Step 1 and 2 can be done in one step using the following script:
**Script**:`/generate_training_testing_libsvm_global.sh path_descriptors type_descriptor output`

#### 3. (optional) Check if the files are well formated (good format for libSVM)

In order to run the training with libSVM, the training file and testing file need to be in the good format.

**Script**:`checkdata.py input`

**Example**:`python checkdata.py ../../../script/dataset_descriptor_training_esf_libsvm.txt`

If no errors, okay. Can continue the process.

#### 4. SVM training - Cross validation for obtaining the best parameters

**Script**:
`easy.py training_file testing_file`

**Example**:
`python easy.py ../../../script/dataset_descriptor_training_esf_libsvm.txt ../../../script/ dataset_descriptor_testing_esf_libsvm.txt`
==Return the accuracy with the best parameters (C and Gamma). Return as well the svm model file trained datasetDescriptorTrainingEsfLibsvm.model==

If we want, we can run our own training (in order to build a svm model) using the C++ binary svm-train.  
**Training**svm learn main allows to train an SVM without convert into libSVM format (do it internally) . Need to change parameters in config.cfg and in the svmTrain function of the file `svm_train.cpp`

**Script**:
`./svm_learn_main config_file descriptor_esf (pas au format libsvm) output_svm`

**Example**:
`./svm_learn_main ../config.cfg ../Dataset/dataset_descriptor_esf.txt ../Dataset/svm/`

`./svm-train -c 128 -g 0.125 dataset_descriptor_training_vfh_libsvm_ratio3.txt svm_vfh.model`**_For easy process, it's good first to use the cross validation and to use the process explained previously._**

#### 5. SVM testing

**Script**:  
`./svm_prediction_main config_file model_svm pcd_file_3Dobject`

**Example**:
`./svm_prediction_main ../config.cfg ../Dataset/svm/model_svm.model ../Dataset/ structure-sensor_dataset/teabox/teabox_1/Model.pcd`

return the label of the category.


## With RGBD dataset

Same process as previously with the structure sensor dataset.


