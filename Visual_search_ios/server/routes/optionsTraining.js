/**
 * @author Lirone Samoun
 * @brief 
 */

//Path where to find the descriptors of the RGBD dataset
 var trainedDatabaseRGBD = "../Datasets/rgbd-dataset/descriptors-resolution-rgbd-database/";
 // Path where to find the descriptors of the structure sensor dataset
 var trainedDatabaseStructureSensor = "../Datasets/trained_structure_sensor_dataset/";
 // Path of the dataset
 var database = "../Datasets/structure_sensor_dataset";
 //Path of the config file for C++ program
 var config_file = "../core/config.cfg";


// Requirements
var express = require('express');
var router = express.Router();
var multer = require('multer');
var fs = require('fs');
var mkdirp = require('mkdirp');
var process = require('../modules/optionsUtilities');

//Part which handles the POST method from HTTP
router.post('/', function (req, res) {
  //Call the upload variable we defined before
  console.log(req.body);

  //Get the variable from the multi part form
  var buildTree = req.body.buildTree;
  var checkDatabase = req.body.checkDatabase;
  var generateView = req.body.generateView;
  var trainSVMdesc = req.body.trainSVMdescriptor;
  var ratioTraining = req.body.ratioTraining;


  if (checkDatabase != "false"){
    //Run the c++ program for checking the dataset
    checkDataset(database,checkDatabase, function(valueReturn){
    if (valueReturn){
      console.log("[NODE JS INFO HTTP] OK");
      res.status(200).send('[NODE JS INFO HTTP] Check database okay');
    }else {
      console.log("[NODE JS INFO HTTP] Error");
      res.status(420).send('Check database not ok');
    }
  });

  }else {
    if (generateView == "true"){
      console.log("[Node JS] Generating views ...");
      generateViewsFromDataset(database,trainedDatabaseStructureSensor, config_file, function(valueReturn){
        if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          res.status(200).send('Views have been generated');
        }else {
          console.log("[NODE JS INFO HTTP] Error");
          res.status(420).send('Views has not been generated');
        }
      });

      //Generate views of each object of the dataset using virtual scanner and compute descriptors. We do it for 4 descriptors
      //Here work in synchrone way i.e the next command begin to work once the previous one has finished
    }else if (buildTree == "true"){
      var everythingIsFine = false;
      console.log("[Node JS] Building index tree ...");

      generateIndexTree(trainedDatabaseStructureSensor,"esf", function(valueReturn) {
        if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          everythingIsFine = true;
        }else {
          console.log("[NODE JS INFO HTTP] Error");
          everythingIsFine = false; 
        }
        generateIndexTree(trainedDatabaseStructureSensor,"vfh", function(valueReturn) {
         if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          everythingIsFine = true;
        }else {
          console.log("[NODE JS INFO HTTP] Error");
          everythingIsFine = false; 
        }
        generateIndexTree(trainedDatabaseStructureSensor,"cvfh", function(valueReturn) {
         if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          everythingIsFine = true;
        }else {
          console.log("[NODE JS INFO HTTP] Error");
          everythingIsFine = false; 
        }
        generateIndexTree(trainedDatabaseStructureSensor,"ourcvfh", function(valueReturn) {
         if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          everythingIsFine = true;
        }else {
          console.log("[NODE JS INFO HTTP] Error");;
          everythingIsFine = false; 
        }
        if (everythingIsFine){
         console.log("[NODE JS INFO HTTP] OK");
         res.status(200).send('Index have been generated');
       }else {
        console.log("[NODE JS INFO HTTP] Error");
        res.status(420).send('Index have not been generated');
      }
    });
      });
      });
      });

      //If this part we perform a svm training. But before that, we generate training and testing file if not exist.
    }else if (trainSVMdesc != "false"){
      //Create directory where going to save the training and testing file
      var output = trainedDatabaseStructureSensor + "libsvmdata/" + trainSVMdesc + "/" ;
      var training_file = output + "dataset_descriptor_training_" + trainSVMdesc + "_libsvm_ratio" + ratioTraining + ".txt";
      var testing_file = output + "dataset_descriptor_testing_" + trainSVMdesc + "_libsvm_ratio" + ratioTraining + ".txt";
      mkdirp(output, function(err) { 
        });
      
      if (fs.existsSync(output)) {
       startTrainingSVM(training_file,testing_file, function(valueReturn){

        if (valueReturn){
          console.log("[NODE JS INFO HTTP] OK");
          res.status(200).send('Training SVM okay');
        }else {
          console.log("[NODE JS INFO HTTP] Error");
          res.status(420).send('Training SVM not ok');
        }
      });
     }else {
      startGeneratingTrainingTestingFiles(trainedDatabaseStructureSensor,trainSVMdesc,output, function(callback){
        startTrainingSVM(training_file,testing_file, function(valueReturn){

          if (valueReturn){
            console.log("OKK");
            res.status(200).send('Training SVM okay');
          }else {
            console.log("Nooooo");
            res.status(420).send('Training SVM not ok');
          }
        });
      });
    }
  }else {
   console.err("Error in the multiform part ");

 }
}
});

/**
* Run the traning of the DB process
**/
var startTrainingDB = function (type_descriptor, trainedDatabase, train, buildTree, callback) {
  process.trainDB(type_descriptor, database, trainedDatabase, buildTree, train, callback); 
};


/**
* Run the generate view process and compute 3D descriptors
**/
var generateViewsFromDataset = function (dataset_folder, result_folder, config_file, callback) {
  process.generateViewsFromDataset(dataset_folder, result_folder,config_file, function(isOk) {
    callback(isOk);      
  }); 
};

/**
* Run the load dataset process
**/
var checkDataset = function (dataset,check, callback) {
  process.loadDBCheck(dataset, check, function(isOk) {
    callback(isOk);      
  }); 
};

/**
* Run the generate index KDtree structure
**/
var generateIndexTree = function (trained_dataset_folder,descriptor, callback) {
  process.buildTree(trained_dataset_folder, descriptor, function(isOk) {
    callback(isOk);      
  }); 
};

/**
* Run the SVM training process
**/
var startTrainingSVM = function (training_file, testing_file, callback) {
  process.trainSVM(training_file, testing_file, function(isOk) {
    callback(isOk);      
  }); 
};

/**
* Run the generate training and testing file process
**/
var startGeneratingTrainingTestingFiles = function (descriptors_dataset_folder, type_descriptor, output, callback) {
  process.generateTrainingTestingFiles(descriptors_dataset_folder, type_descriptor, output, function(isOk) {
    callback(isOk);      
  }); 
};


module.exports = router;