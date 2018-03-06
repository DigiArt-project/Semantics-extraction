# 3D Interactive Learning 

## Overview
=======
This is the code for 3D object classification using Interactive Learning. 
We propose to leverage the challenge of designing a Content-Based 3D shape Retrieval (CB3DR) adapted for non-expert users using 3D point cloud acquired with low-cost sensors.
Given a point cloud acquired by a scanner device, our CB3R retrieve similar shapes from a database. 

Interactive Learning algorithms in based on SVM which requires very simple positive-negative annotations from the users.
We have evaluated our system on several standard 3D datasets (Cat10, Cat31, Cat60, RGB-D, ModelNet10, and our own low-resolution shape dataset made with the manual Occipital Structure Sensor considering 8 state-of-the-art descriptors. 

* **main_interactivelearning.py** —> Main function for running Interactive Learning process


## Requirements 

### 0. Package requirement

* Sklearn-scikit 0.18
* Matplotlib
* Numpy
* Scikit
* multiprocessing

You can install the necessary package by using the following command :
` pip3 install -r requirements.txt `

Before running the code, you need to perform some steps.

### 1. Compile "CloudRetrieval_main"

The binary executable is given. If it is not the case, you need to compile it from Tools Package.


### 2. Create the data 

### A. Dataset

An example is given. **Dataset\_cat10_normalized.**
==You need to follow the same structure.==

In our experiment we have done multiple experiments using different datasets :

* Cat10
* Cat31
* Cat60
* RGBD Dataset
* ModelNet10
* Our own structure sensor Dataset

Globally, the dataset folder contains the categories of each object and inside each category, you can found the full objects and the partial views of objects along with their descriptors.

For example : 

* Dataset\_cat10\_normalized/bottle/full/bottle1.ply 
* Dataset\_cat10\_normalized/bottle/views/views_bottle0\_0.ply 

At this point, you may face up to two cases :

1. You do not have any partial views generated from your data 
2. You have partial views and full objects in your dataset

If you are in case 1, you should compute the partial views of descriptors using the package Tools. (Name of the program : **compute\_views_descriptors\_main** ). It will generate views and associated descriptors.
If you just want to work with full objects but not partial views, just be careful that you have the right structure for your dataset.

If you are in case 2, just be careful that you have the right structure for your dataset.

#### B. Compute the descriptors

Once you have your dataset set up, you can compute (if it is not done yet) the descriptors.

Compute descriptors of each object of the database. You can use `compute_descriptor_cloud_main`from TOOLS package for computing descriptors.

### C. Running the Scripts for generating Interactive Learning Data

So far, you have your datasets with the good structure, and inside your dataset you have full, partial views (if you want), and associated descriptors. You can now create the data for Interactive Learning.

* **1. create\_list\_path_desriptors.sh**

Given a dataset of descriptors, generate a txt file which contains the path to every descriptor.
You should run this script for the root of your dataset path. For the parameters, you write the name of the dataset, the descriptor your want (choose a descriptor that you have computed before) and finally if you want to work with full objects or partial views of objects.

		./create_list_path_desriptors.sh dataset_path esf/vfh/cvvfh/ourcvfh/grsd/gshot/gshotPyramid/spin/good/scurv/sc3D/egigshotfull/esffull/grsdfull/pointnet/alldesc[i] full/views
		
	==Exemples==
	
		./create_list_path_desriptors.sh Dataset_cat10_normalized esf full
		./create_list_path_desriptors.sh Dataset_cat10_normalized esf views


* **2. generate\_libsvm\_data\_main**

Once you have your file with all the descriptors path, you can use this script.

From a list of path of descriptors, it generates a file which contains the descriptors with libsvm format.

		./generate_libsvm_data_main descriptor_file_path (descriptor file which contains the path to every descriptor. One line per data) output (file which will contains the descriptors) type_descriptor (esf/vfh/ourcvfh/cvfh/grsd/gshot/scurv/good/egi/spin/sc3D/pointnet/esffull/gshotfull/grsdfull/all) multiclass (1/0)
		
	==Exemple==

		./generate_libsvm_data_main dataset_descriptor_esf.txt descriptors_esf.txt esf 1


Once generated, those two files need to be added in the `Data`folder of Interactive Learning folder.

So after this step you need to have for Cat10 dataset (or other dataset):

* Cat10_views/dataset\_descriptors\_esf.txt
* Cat10_views/descriptors\_esf.txt
* Cat10_full/dataset\_descriptors\_esf.txt
* Cat10_full/descriptors\_esf.txt

If you just want to work with partials views, you will just have the views folder, and same things with full folders.

==Those files are necessary for the interactive Learning process==
`

## How to run the code for interactive learning

### Important global variable in main_interactivelearning.py
			#The dataset which contains the 3D objects and the descriptors
			path_dataset = "/Users/digiArt/Datasets/Dataset_cat10_normalized/"
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

* **path\_dataset** --> This is the path of the dataset. Remember, you need to follow the same structure (indicated by the given example Dataset\_Cat10\_normalized)
* **compute\_full** --> Indicate if you want to perform interactive learning on full objects or on partial views. Both case, pay attention that you have computed the descriptors and you followed the previous steps. "true" or "false".
* **descriptors** --> Array of descriptors you want to use if you apply the option similaritySearch\_automatic\_oneobject and similaritySearch\_automatic\_alltesting
* **name_dataset** --> Name of the dataset inside your data folder

Example :
If you want to perform interactive learning on partial views with the Dataset\_cat10\_normalized dataset, you should write : 

==compute_full="false"==

==name\_dataset = 'cat10\_views'==

if you want to perform interactive learning on full objects with the Dataset\_cat10\_normalized dataset, you should write : 

==compute_full="true"==

==name\_dataset = 'cat10\_full'==

### Four options for running Interactive Learning

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

		python3 main_interactivelearning.py [-descriptorInteractive]
		
Description: Basic process of interactive learning. The user chooses the categories and then he needs to label positively and negatively samples that are displayed.

Per defaut, it works with ESF descriptor, wut you can change it by using the parameter `-descriptorInteractive`

### 2. **Interactive Learning with Similarity Search - Manual version**

Option **similaritySearch_manuel**

Description: Given a point cloud query, similarity search is performed. For visual display you need to comment `matplotlib.use('Agg')`. After similarity search, the user need to label manually positives and negatives samples.

		python3 main_interactivelearning.py

		  -query QUERY, --query QUERY
		                        The query point cloud (in PCD)
		  -trained TRAINED, --trained TRAINED
		                        The trained dataset where the index of similarity
		                        search is located
		  -descriptorSimilarity DESCRIPTORSIMILARITY 
		  						Descriptor for similarity Search. Per default : esf
		  -descriptorInteractive DESCRIPTORINTERACTIVE
		  						Descriptor for Interactive Learning. Per default : esf
		  
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
			
Per defaut, it works with ESF descriptor, wut you can change it by using the parameter `-descriptorInteractive`. Same with similarity search.

### 3.  **Interactive Learning with Similarity Search - Automatic version**

Option **similaritySearch\_automatic\_oneobject**

Description: Given a point cloud query, similarity search is performed and then Interactive Learning is performed automatically without any help from the user. Visual display possible if you comment `matplotlib.use('Agg')`. Images of the results are save automatically.

		python3 main_interactivelearning.py
	
		  -query QUERY, --query QUERY
		                        The query point cloud (in PCD)
		  -trained TRAINED, --trained TRAINED
		                        The trained dataset where the index of similarity
		                        search is located
		  -descriptorSimilarity DESCRIPTORSIMILARITY 
		  						Descriptor for similarity Search. Per default : esf
		  -descriptorInteractive DESCRIPTORINTERACTIVE
		  						Descriptor for Interactive Learning. Per default : esf
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
		
Per defaut, it works with ESF descriptor, wut you can change it by using the parameter `-descriptorInteractive`. Same with similarity search.

### 4. **Interactive Learning all Testing**

Option **similaritySearch\_automatic\_alltesting**

Description : Interactive Learning process is run automatically according to the number of iteration the user want to use.
On the parameters, the user need to choose the database. After each iteration, images of results are saved automatically.
No display during the process.
The descriptors are the one from the array 

			#Descriptors you want to use
			descriptors = ['esf','pointnet']
-

		python3 main_interactivelearning.py -nbiterations - multiprocessing - cores
		
		 -nbiterations ITERATIONS, --iterations ITERATIONS
		                        Number of iterations max for each interactive learning
		                        session (Per defaut : 20)
		  -cores NUMCORES, --numcores NUMCORES
		                        Number of cores to use (Per defaut : cpu_count())
			
	
==_Exemple_== :
				
	python3 interactivelearning3D.py -nbiterations 2 -cores 2
				
It will run the whole process of interactive learning with ESF and PointNet descriptor, with 2 cores and 2 steps of interactive learning.

For any help :
		
	python3 main_interactivelearning.py --help


		


