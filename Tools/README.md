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

### On linux system

You just have to run the following command line in the terminal:`cmake .`

then `make` in order to compile. All the binaries files will be generated.

### On OSx system

On the directory, run the following command line in the terminal:`cmake -G Xcode .`

Then, click on the ==core.xcodeproj== generated. Build the project using XCode.
All the binaries files will be generated.


## Content of the package

Here the necessary binary files (with a brief explanation) which have been compiled. Can be compiled again if you want to modify the source code.

### C++ binaries 
* **`compute_views_descriptor_cloud_main `**—> This program take a list of ply object point cloud and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed (need to activate the descriptors you want on config.cfg). Each view is saved along its descriptor.
* **`generate_libsvm_data_main `**—> from a list of path of descriptors, generate a file which contain the descriptors compatible with the libsvm format
* **`compute_descriptor_cloud_main`**—> Compute descriptors given a dataset
* **`cloudRetrieval_main`**—> Perform similarity search given a query and an index KD Tree structure
* **`build_tree_main`**—>  Generate an Index KDTree structure for performing faster similarity search

### Python Script
* **`egi_spin_descriptors`**—> This programs allows to compute on point clouds EGI and SPIN descriptor
* **`compute_mean_score`**—> Given a txt file with an array, compute the mean
* **`egi_spin_descriptors`**—> Compute EGI and SPIN descriptors for a specific point cloud or a dataset of point cloud
* **`scale_descriptors`**—> scale descriptors between a range [min,max] inside a dataset

### Bash script

* **`rename_objects_structure_sensor `**—> Given a structure sensor dataset (specific structure) dataset, rename correctly all the objects in order to have something better visually. For example : bottle/bottle_0.ply bottle/bottle_1.ply
* **`rename_objects_dataset`**—> Given a 3DNet dataset (Cat10, Cat31, Cat60), rename all of the objects correctly
* **`create_list_path_desriptors`** --> Given a dataset of descriptors, generate a txt file which contain the path to every descriptors
* **`generate_training_testing_libsvm_global `** --> From the txt file which contain the path of descriptors, generate a file which contain all the desriptors concatenated all together. Compatible with LibSVM
* **`concatenate_descriptor`** --> Given a dataset, take every obj point cloud and convert them to ply point cloud## How to do ?

#### 1. Rename objects of datasets

If the dataset has the following structure like the 3D net dataset : categories_1/object\_i.ply, use this command line

	./rename_objects_dataset.sh -d Dataset_Cat10_normalized -s true 
	
If the dataset has the following structure like the structure sensor dataset : categories\_1/Object\_i/object.ply, use this command line

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


#### 4 . Build Index KD Tree structure	
For performing similarity search you need to build a KD Tree Index structure. 
For the parameters, you need to put the dataset with all the descriptors, the descriptor you want to build the KD Tree structure and if you want to build the index for partials views (views) or full object (full)

**Script**: 
`./build_tree_main -trained tained_dataset -descriptor type_descriptor -folder  [Options]`

**Exemples**:

	./build_tree_main -trained ../../Datasets/Dataset_cat10_normalized -descriptor esf -folder views

#### 5 . Cloud Retrieval

If you want to perform similarity search, you can run this script. Before running it, you need to perform step 4 i.e build the KD Tree Index structure.
The parameters trained if the folder where the Index Structure has been built thank to the previous step.
The query parameter is the cloud you want to query.

**Script**: 
`./cloudRetrieval_main -query object.pcd -trained tained_dataset [Options]`

**Exemples**:

	./cloudRetrieval_main -query bottle0.pcd -trained Dataset_cat10_normalized [Options]

#### (optional). Concatenate certains descriptors togethers
	
#### (optional) Generate libsvm data from descriptors

**Script**: `python3 concatenate_descriptor.py -d dataset_descriptors -s scale_boolean`

**Exemples**:

	python3 concatenate_descriptor.py -d ../Datasets/cat31_descriptors -s true


