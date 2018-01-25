# Tools


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


## Content of the package

Here the necessary binary files (with a brief explanation) which have been compiled. Can be compiled again if you want to modify the source code.

### C++ binaries 
* **`compute_views_descriptor_cloud_main `**—> This program take a list of ply object point cloud and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed (VFH, CVFH, ESF, OURCVFH, GSHOT, GRSD). Each view can be saved and every descriptors are saved.
* **`generate_libsvm_data_main.py `**—> from a list of path of descriptors, generate a file which contain the descriptors compatible with libsvm

### Python Script
* **`compute_views_descriptor_cloud_main `**—> This programs allows to concatenate some descriptors in one file. It can scale descriptors independently before concatening each one. 

### Bash script

* **`rename_objects_structure_sensor `**—> Given a structure sensor dataset (specific structure) dataset, rename correctly all the objects in order to have something better visually. For example : bottle/bottle_0.ply bottle/bottle_1.ply
* **`rename_objects_dataset`**—> Given a 3DNet dataset (Cat10, Cat31, Cat60), rename all of the objects correctly
* **`create_list_path_desriptors`** --> Given a dataset of descriptors, generate a txt file which contain the path to every descriptors
* **`generate_training_testing_libsvm_global `** --> From the txt file which contain the path of descriptors, generate a file which contain all the desriptors concatenated all together. Compatible with LibSVM
* **`concatenate_descriptor`** --> Given a dataset, take every obj point cloud and convert them to ply point cloud## How to do ?

#### 1. Rename objects of datasets

If the dataset has the following structure like the 3D net dataset : categories_1/objecti.ply, use this command line

	./rename_objects_dataset.sh -d Cat10Dataset -s true 
	
If the dataset has the following structure like the structure sensor dataset : categories_1/Object_i/object.ply, use this command line

	./rename_objects_structure_sensor.sh dataset_path 
	
#### 2. Generate views of objects and compute descriptors

In the config.cfg file, you can activate or deactivate computations of certains descriptors and change others parameters.
You need to change the path to the dataset :  
	
	path_to_dataset = "../../Dataset_cat31";
	
Then run the program : 

**Script**: `./compute_views_descriptor_cloud_main config_file output_folder`

**Exemples**:

	./compute_views_descriptor_cloud_main  ../../config.cfg ../../cat31_descriptors

#### 3. Create list of paths of descriptors

Create a txt file which contains the path to every descriptors.

**Script**: `./create_list_path_desriptors.sh path_to_descriptors type_descriptors `

**Exemples**:

	./create_list_path_desriptors.sh ../Datasets/cat31_descriptors esffull 
	

#### 3 (optional). Concatenate certains descriptors togethers
	
#### 4. Generate libsvm data from descriptors

**Script**: `python3 concatenate_descriptor.py -d dataset_descriptors -s scale_boolean`

**Exemples**:

	python3 concatenate_descriptor.py -d ../Datasets/cat31_descriptors -s true


