# 3D Interactive Learning 

## Overview
=======
This is the code for 3D object classification using Interactive Learning. 
We propose to leverage the challenge of designing a Content-Based 3D shape Retrieval (CB3DR) adapted for non-expert users using 3D point cloud acquired with low-cost sensors.
Given a point cloud acquired by a scanner device, our CB3R retrieve similar shapes from a database. 

Interactive Learning algorithm in based on SVM which requires very simple positive-negative annotations from the users.
We have evaluated our system on several standard 3D datasets (Cat10, Cat31, Cat60, RGB-D, ModelNet10, and our own low-resolution shape dataset made with the manual Occipital Structure Sensor considering 8 state-of-the-art descriptors. 

* **main_interactivelearning.py** —> Main function for running Interactive 


## Requirements 

Before running the code, you need to perform some steps.

### 1. Compile "CloudRetrieval_main"

The binary executable is given. If it is not the case, you need to compile it from Tools Package.

### 2. Create the data 

#### A. Compute the descriptors

Compute descriptors of each object of the database. You can use `compute_descriptor_cloud_main`from TOOLS package for computing descriptors.

### B. Dataset

An exemple is given. **Dataset\_cat10_normalized.**
==You need to follow the same structure.==

In our experiment we hade done multiple experiments using different datasets :

* Cat10
* Cat31
* Cat60
* RGBD Dataset
* ModelNet10
* Our own structure sensor Dataset

Globally, the dataset folder contains the categories of each object and inside each category, you can found the full objects and the partial objects alon with their descriptors.

### C. Running the Scripts for generating Interactive Learning Data

* **create\_list\_path_desriptors.sh**

Given a dataset of descriptors, generate a txt file which contain the path to every descriptors.

		./create_list_path_desriptors.sh dataset_path esf/vfh/cvvfh/ourcvfh/grsd/gshot/gshotPyramid/spin/good/scurv/sc3D/egigshotfull/esffull/grsdfull/pointnet/alldesc[i] full/views
		
	==Exemples==
	
		./create_list_path_desriptors.sh Dataset_cat10_normalized esf full
		./create_list_path_desriptors.sh Dataset_cat10_normalized esf views


* **generate\_libsvm\_data\_main**

From a list of path of descriptors, generate a file which contain the descriptors compatible with libsvm.

		./generate_libsvm_data_main descriptor_file_path (descriptor file which contains the path to every descriptor. One line per data) output (file which will contains the descriptors) type_descriptor (esf/vfh/ourcvfh/cvfh/grsd/gshot/scurv/good/egi/spin/sc3D/pointnet/esffull/gshotfull/grsdfull/all) multiclass (1/0)
		
	==Exemple==

		./generate\_libsvm\_data\_main dataset\_descriptor\_est.txt descriptors\_esf.txt esf 1


Once generated, those two files need to be added in the `Data`folder.

So after this part you need to have for Cat10 dataset:

* Cat10_views/dataset\_descriptors\_esf.txt
* Cat10_views/descriptors\_esf.txt
* Cat10_full/dataset\_descriptors\_esf.txt
* Cat10_full/descriptors\_esf.txt

==Those files are necessary for the interactive Learning process==
`

## How to run the code

### Important global variable
			#The dataset which contains the 3D objects and the descriptors
			path_dataset = "/Users/lironesamoun/digiArt/Datasets/Dataset_cat10_normalized/"
			training_file_full = path_dataset + "train_full.txt"
			testing_file_full =  path_dataset + "test_full.txt"
			training_file_views =  path_dataset +"train_views.txt"
			testing_file_views =  path_dataset +"test_views.txt"
			#if you set compute full to true, you run interactive learning on full objects and not partials views
			compute_full = "false"
			#Descriptors you want to use
			descriptors = ['esf','pointnet']
			#Name of the dataset
			name_dataset = 'cat10_views'
			root_data = "data/"
			#Path for option : basic interactive learning
			view_path_file = "data/cat10_views/dataset_descriptor_esf.txt"
			dataset_descriptor = "data/cat10_views/descriptors_esf.txt"

* **path\_dataset** --> This is the path of the dataset. Remember, you need to follow the same structure (indicated by the given example Dataset\_Cat10\_normalized)
* **compute\_full** --> Indicate if you want to perform interactive learning on full objects or on partial views. Both case, pay attention that you have computed the descriptors.
* **descriptors** --> Array of descriptors you want to use if you apply the option similaritySearch\_automatic\_oneobject and similaritySearch\_automatic\_alltesting
* **name_dataset** --> Name of the dataset
* **view\_path_file** --> For option without\_similaritySearch and similaritySearch\_manuel, you need to manually say where is the view path file (computed thank to the script)
* **dataset\_descriptor** -> For option without\_similaritySearch and similaritySearch\_manuel, you need to manually say where is the descriptors path file (computed thank to the script)


### Four options

You can choose among four options.

In the main function, you have the following :

    class option_interactiveLearning:
        without_similaritySearch = 0
        similaritySearch_manuel = 1
        similaritySearch_automatic_oneobject = 2
        similaritySearch_automatic_alltesting = 3

    #Choose here the option you want
    current_option = option_interactiveLearning.similaritySearch_automatic_oneobject

### 1. **Basic Interactive Learning**

Option **without_similaritySearch**

		python3 main_interactivelearning.py
		
Description: Basic process of interactive learning. The user choose the categories and then he need to label positively and negatively samples that are displayed.

### 2. **Interactive Learning with Similarity Search - Manual version**

Option **similaritySearch_manuel**

Description: Given a point cloud query, similarity search is performed. For visual display you to comment `matplotlib.use('Agg')`. After similarity search, the user need to label manually positives and negatives samples.

		python3 main_interactivelearning.py

		  -query QUERY, --query QUERY
		                        The query point cloud (in PCD)
		  -trained TRAINED, --trained TRAINED
		                        The trained dataset where the index of similarity
		                        search is located
		  -descriptor DESCRIPTOR, --descriptor DESCRIPTOR
		                        change descriptor. Available : esf, vfh, cvfh,
		                        ourcvfh, gshot, usc, grsd. Per default : esf
		  -output OUTPUT, --output OUTPUT
		                        where to save the json results file. Per default :
		                        results.json
		  -leaf_resolution RESOLUTION, --resolution RESOLUTION
		                        For cloud resolution invariance,. Per defaut : 0.01
		  -k K, --k K           number of results to find. Per defaut : 10
		  -nbiterations ITERATIONS, --iterations ITERATIONS
		                        Number of iterations for automatic labelling. Per
		                        defaut : 10
	
	
	
	
==_Exemple_== :
	
			python3 interactivelearning3D.py -query ../Datasets/Dataset_cat10_normalized/bottle/views/view_bottle1_1.ply -trained ../Datasets/Dataset_cat10_normalized/

### 3.  **Interactive Learning with Similarity Search - Automatic version**

Option **similaritySearch_automatic_oneobject**

Description: Given a point cloud query, similarity search is performed and then Interactive Learning is performed automatically without any help from the user. Visual display possible if you comment `matplotlib.use('Agg')`. Images of the results are save automatically.

		python3 main_interactivelearning.py
	
		  -query QUERY, --query QUERY
		                        The query point cloud (in PCD)
		  -trained TRAINED, --trained TRAINED
		                        The trained dataset where the index of similarity
		                        search is located
		  -descriptor DESCRIPTOR, --descriptor DESCRIPTOR
		                        change descriptor. Available : esf, vfh, cvfh,
		                        ourcvfh, gshot, usc, grsd. Per default : esf
		  -output OUTPUT, --output OUTPUT
		                        where to save the json results file. Per default :
		                        results.json
		  -leaf_resolution RESOLUTION, --resolution RESOLUTION
		                        For cloud resolution invariance,. Per defaut : 0.01
		  -k K, --k K           number of results to find. Per defaut : 10
		  -nbiterations ITERATIONS, --iterations ITERATIONS
		                        Number of iterations for automatic labelling. Per
		                        defaut : 10


==_Exemple_== :

		python3 interactivelearning3D.py -query ../Datasets/Dataset_cat10_normalized/bottle/views/view_bottle1_1.ply -trained ../Datasets/Dataset_cat10_normalized/

### 4. **Interactive Learning all Testing**

Option **similaritySearch_automatic_alltesting**

Description : Interactive Learning process is run automatically according to the number of iteration the user want to use.
On the parameters, the user need to choose the database. After each iteration, images of results are saved automatically.
No display during the process.


		python3 main_interactivelearning.py -nbiterations - multiprocessing - cores
		
		 -nbiterations ITERATIONS, --iterations ITERATIONS
		                        Number of iterations max for each interactive learning
		                        session (Per defaut : 20)
		  -multiprocessing MULTIPROCESSING, --multiprocessing MULTIPROCESSING
		                        If run the program in one cpu or multiple (Per defaut
		                        : True)
		  -cores NUMCORES, --numcores NUMCORES
		                        Number of cores to use (Per defaut : cpu_count())
			
	==_Exemple_== :
				
				python3 interactivelearning3D.py -nbiterations 2 -multiprocessing true -cores 2

	

For any help :
		
		python3 main_interactivelearning.py --help


		


