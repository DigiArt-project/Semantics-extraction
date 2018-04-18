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

Then `make` in order to compile. All the binaries files will be generated.

### On OSx system

On the directory, run the following command line in the terminal:`cmake -G Xcode .`

Then, click on the ==core.xcodeproj== generated. Build the project using XCode.
All the binaries files will be generated.


## Content of the package

Here the necessary binary files (with a brief explanation) which have been compiled. Can be compiled again if you want to modify the source code.

### C++ binaries 
* **`compute_views_descriptor_cloud_main `**—> This program take a list of ply object point cloud and generate multiple views of each object. Then for each object, one or multiple global descriptors are computed (need to activate the descriptors you want on config.cfg). Each view is saved along its descriptor.
* **`generate_libsvm_data_main `**—> from a list of path of descriptors generate a file which contains the descriptors compatible with the libsvm format
* **`compute_descriptor_cloud_main`**—> Compute descriptors given a dataset
* **`cloudRetrieval_main`**—> Perform similarity search given a query and an index KD Tree structure
* **`build_tree_main`**—> Generate an Index KDTree structure for performing faster similarity search
* **`normalize_pointcloud_main`** --> Given a point cloud, normalize it
* **`upsampling_main`** --> Given a point cloud, upsample it i.e add more points to it and raise his resolution by applying an upsampling method
* **`compute_resolution_main`** --> Compute resolution of a point cloud

### Python Script
* **`egi_spin_descriptors`**—> This program allows to compute on point clouds EGI and SPIN descriptor
* **`compute_mean_score`**—> Given a txt file with an array of score, compute the mean
* **`egi_spin_descriptors`**—> Compute EGI and SPIN descriptors for a specific point cloud or a dataset of point cloud
* **`scale_descriptors`**—> scale descriptors between a range [Min,Max] inside a dataset
* **`normalizePC`**—> Given a dataset, normalize all the points cloud inside it, thank to the normalize_pointcloud_main program


### Bash script

* **`rename_objects_structure_sensor `**—> Given a structure sensor dataset (specific structure) dataset, rename correctly all the objects in order to have something better visually. For example : bottle/bottle_0.ply bottle/bottle_1.ply
* **`rename_objects_dataset`**—> Given a 3DNet dataset (Cat10, Cat31, Cat60), rename all of the objects correctly
* **`create_list_path_desriptors`** --> Given a dataset of descriptors, generate a txt file which contains the path to every descriptor
* **`generate_training_testing_libsvm_global `** --> From the txt file which contains the path of descriptors, generate a file which contains all the descriptors concatenated all together. Compatible with LibSVM
* **`concatenate_descriptor`** --> Given a dataset, take every obj point cloud and convert them to ply point cloud
* **`obj2ply_all`** --> Given a dataset, convert every obj file to ply file. Need the program pcl_obj2ply given by PCL.

## Step by step guide for running Interactive Learning ?

#### 1. Prepare the dataset (structure + format point cloud)

You need first to build the right structure for the dataset. It must follow the following structure :
**name\_dataset/category\_i/object\_j**
i.e inside a dataset, you need to have multiple objects categories and inside each category, you have multiple 3D point cloud in **PLY** format.

**Depending of your dataset, you may want to run the right script :**

If the dataset has the following structure (like the [3DNet datasets](https://repo.acin.tuwien.ac.at/tmp/permanent/3d-net.org/)) : categories_i/object\_j.ply, use this command line

	./rename_objects_dataset.sh -d Dataset_Cat10_normalized -s true 
	
If the dataset has the following structure (like the structure sensor dataset) : categories\_i/Object\_j/object.ply, use this command line

	./rename_objects_structure_sensor.sh dataset_path 
	
#### 2. Generate views of objects and compute descriptors

Once the dataset is ready, you can generate partial views from 3D objects.

You need to change the path to the dataset :  
	
	path_to_dataset = "../../Dataset_cat31";
	
Then run the program : 

**Script**: `./compute_views_descriptor_cloud_main config_file output_folder`

**Exemples**:

	./compute_views_descriptor_cloud_main  ../../config.cfg ../../Dataset_cat31_views
	
#### 3 Normalize dataset

Once the partial views are generated, run the normalization step.

You need to compile first the program normalize\_pointcloud_main.

**Script**:

	python normalizePC.py --folder --new folder

**Example**: `python normalizePC.py --folder Dataset_cat31 --new-folder Dataset_cat31_normalized`


#### 4. Compute descriptors of partial/full objects

Once your dataset is normalized, you can compute the descriptors.

**Script**: `./compute_descriptor_cloud_main`

Inside the program, you need to change the **dataset\_folder** parameter and the **onFull** parameter.

For example:

	std::string dataset_folder = "./Datasets Dataset_pottery_normalized";
	bool onFull = false;

If you activate onFull you will compute descriptors for only full objects. If you set onFull on false, you will compute descriptors of partial views.
You can make both.


#### 5. Create list of paths of descriptors

So far, you have your dataset normalized which contains objects and descriptors.

Now you have to create a txt file which contains the path to every descriptor.

**Script**: `./create_list_path_desriptors.sh path_to_descriptors type_descriptors `

**Exemples**:

	./create_list_path_desriptors.sh ../Datasets/Dataset_cat31_normalized esf


#### 6 . Build Index KD Tree structure for similarity search	
For performing similarity search you need to build a KD Tree Index structure. 
For the parameters, you need to put the dataset with all the descriptors, the descriptor you want to build the KD Tree structure and if you want to build the index for partial views (views) or full object (full)

**Script**: 
`./build_tree_main -trained tained_dataset -descriptor type_descriptor -folder  [Options]`

**Exemples**:

	./build_tree_main -trained ./Dataset_cat31_normalized -descriptor esf -folder views

#### 7 . Cloud Retrieval

If you want to perform similarity search, you can run this script. Before running it, you need to perform step 6 i.e build the KD Tree Index structure.

**Script**: 
`./cloudRetrieval_main -query object.pcd -trained tained_dataset [Options]`

**Exemples**:

	./cloudRetrieval_main -query bottle0.pcd -trained Dataset_cat31_normalized [Options]

#### 8 . Generate data for interactive learning


So far, you have your datasets with the good structure, and inside your dataset you have full, partial views and associated descriptors. You can now create the data for Interactive Learning.

* **1. create\_list\_path_desriptors.sh**

Given a dataset of descriptors, generate a txt file which contains the path to every descriptor.
You should run this script for the root of your dataset path. For the parameters, you write the name of the dataset, the descriptor you want (choose a descriptor that you have computed before) and finally if you want to work with full objects or partial views of objects.

		./create_list_path_desriptors.sh dataset_path esf/vfh/cvvfh/ourcvfh/grsd/gshot/gshotPyramid/spin/good/scurv/sc3D/egigshotfull/esffull/grsdfull/pointnet/alldesc[i] full/views
		
	==Exemples==
	
		./create_list_path_desriptors.sh Dataset_cat31_normalized esf full
		./create_list_path_desriptors.sh Dataset_cat31_normalized esf views


* **2. generate\_libsvm\_data\_main**

Once you have your file with all the descriptors path, you can use this script.

From a list of path of descriptors, it generates a file which contains the descriptors with libsvm format.

		./generate_libsvm_data_main descriptor_file_path (descriptor file which contains the path to every descriptor. One line per data) output (file which will contains the descriptors) type_descriptor (esf/vfh/ourcvfh/cvfh/grsd/gshot/scurv/good/egi/spin/sc3D/pointnet/esffull/gshotfull/grsdfull/all) multiclass (1/0)
		
	==Exemple==

		./generate_libsvm_data_main dataset_descriptor_esf.txt descriptors_esf.txt esf 1


Once generated, those two files need to be added in the `Data`folder of Interactive Learning folder.

So after this step you need to have for Cat31 dataset (or other dataset):

* Cat31_views/dataset\_descriptors\_esf.txt
* Cat31_views/descriptors\_esf.txt
* Cat31_full/dataset\_descriptors\_esf.txt
* Cat31_full/descriptors\_esf.txt

If you just want to work with partials views, you will just have the views folder, and same things with full folders.


#### (optional). Concatenate certain descriptors together


**Script**: `python3 concatenate_descriptor.py -d dataset_descriptors -s scale_boolean`

**Exemples**:

	python3 concatenate_descriptor.py -d ../Datasets/cat31_descriptors -s true
	
#### (optional) Generate libsvm data from descriptors



