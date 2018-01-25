  /**
   * @author Lirone Samoun
   * @brief Initial route.
   */

  // Requirements
  var express = require('express');
  var router = express.Router();
  var multer = require('multer');
  var fs = require('fs');
  require('../modules/response');
  var process = require('../modules/retrieval');
  var request = require('request');

  var trainedDatabaseRGBD = "/Users/lironesamoun/Datasets/rgbd-dataset/descriptors-resolution-rgbd-database/";
  var trainedDatabaseStructureSensor = "../Datasets/trained_structure_sensor_dataset/";
  
  var trainedDatabase;
  var retrieval= true;

  //Parameters for uploading file using Multer
  var storage = multer.diskStorage({
    destination: function (request, file, callback) {
      callback(null, './uploads/');
    },
    filename: function (request, file, callback) {
      console.log(file);
      callback(null, file.originalname)
    }
  });
  // Multer parameter. Cloud correspond to the filename of the form sent by the http request in iOS
  var upload = multer({ storage: storage }).single('cloud');


  //check is the parameters are good
  function check(req,res,callback) {
    var method = req.query.method;
    var descriptor = req.query.descriptor;
    var cloud = req.query.cloud;
    var json_error;
    res.setHeader('Content-Type', 'application/json');
    if (method!="svm" && method!="similaritysearch" ){
      json_error = {'error':'methods selected not good. Can choose similaritysearch or svm'};
      res.write(JSON.stringify(json_error));
      res.end();
      callback(false);
    }else if (descriptor !="esf" && descriptor !="vfh" && descriptor !="cvfh"  && descriptor !="ourcvfh" ){
      json_error = {'error':'descriptor selected not good. Can choose esf, vfh, cvfh, ourcvfh'};
      res.write(JSON.stringify(json_error));
      res.end();
      callback(false);
    }
    else if (!fs.existsSync(req.query.cloud)){
      json_error = {'error':'cloud does not exist'};
      res.write(JSON.stringify(json_error));
      res.end();
      callback(false);
    }else {
     callback(true);
   }
 };


  //When user post on http://localhost:8080/api/upload/
  router.post('/', function (req, res) {
    console.log(req)
    var method = req.query.method;
    var descriptor = req.query.descriptor;
    var cloud = req.query.cloud;  
    console.log("method " + method)
    console.log("DESCRIPTOR " + descriptor)
    console.log("CLOUD " + cloud)
    check(req,res,function(isokay){
      //Parameters are okay
      if (isokay){
        //Multi part form using request module
        var formData = {
          type_descriptor_selected:descriptor,
          method_search_selected:method,
          cloud: {  //Name "cloud" need to correspond to the field of Multer framework multer({ storage: storage }).single('cloud');
            value:  fs.createReadStream(cloud),
            options: {
              filename: cloud,
            }
          }
        };
        request.post({url:'http://localhost:8080/api/upload/action_upload', formData: formData}, function optionalCallback(err, httpResponse, body) {
          if (err) {
            return console.error('upload failed:', err);
          }
          console.log('Upload successful!  Server responded with:', body);
          //Get the results
          request.get('http://localhost:8080/api/results', function (error, response, body) {
            if (!error && response.statusCode == 200) {
              res.setHeader('Content-Type', 'application/json');
              res.write(JSON.stringify(body));
              res.end();
            }
          })
        });
      }
    });
  });


  //http://localhost:8080/api/upload/action_upload
  router.post('/action_upload', function (req, res) {
    //Call the upload variable we defined before
    //console.log(req);
    upload(req, res, function (err) {
      if (err) {
        // An error occurred when uploading
        console.error("[NODE JS ERROR] ERROR during the upload");
        console.error(err);
        return
      }else {
       
       //console.log("req.file" + req.file);
       var type_descriptor = "esf";
       var descriptor_selected = req.body.type_descriptor_selected;
       var method_search_selected = req.body.method_search_selected;
        // Everything went fine
        console.info("[NODE JS INFO] Descriptor selected: ",descriptor_selected);
        console.info("[NODE JS INFO] Method Search selected: ",method_search_selected);
        console.info("[NODE JS INFO] Original name File uploaded : ",req.file.originalname);
        console.info("[NODE JS INFO] Saved to : ",req.file.destination);

        trainedDatabase = trainedDatabaseStructureSensor;

        checkExtensionCloud(req.file.originalname, function() {     
              //Once we got the callback we can continue
              //Cloud uploaded is still the cloud.obj   need to get the pcd file
              var cloud_uploaded = req.file.originalname;
              var objectCloud_without_extension = cloud_uploaded.slice(0, -4);
              var extension = cloud_uploaded.split('.').pop();
              var pcd_cloud = objectCloud_without_extension +".pcd";
               //console.log("[NODE JS INFO] PCD cloud: ",extension);
              //Transform the point cloud
              transform_cloud(pcd_cloud,0,0,0,0,0,0,function(){
                if (retrieval){
                    //Start the retrieval process
                    //Either similarity search or svm
                    if (method_search_selected === "similaritysearch"){
                      console.info("[NODE JS INFO] SIMILARITY SEARCH RETRIEVAL");
                      var k = 10;
                      startRetrievalSimilaritySearch(pcd_cloud,descriptor_selected, k, function(valueReturn){
                        if (valueReturn){
                          console.log("[NODE JS INFO HTTP] OK");
                          res.status(200).send('Retrieval Search ok');
                        }else {
                          console.log("[NODE JS INFO HTTP] Error");
                          res.status(420).send('Retrieval search not OK');
                        }
                      });
                    }else if  (method_search_selected === "svm"){

                      console.info("[NODE JS INFO] SVM RETRIEVAL");
                      startRetrievalSVM(pcd_cloud,descriptor_selected, function(valueReturn){
                        if (valueReturn){
                          console.log("[NODE JS INFO HTTP] OK");
                          res.status(200).send('SVM prediction ok');
                        }else {
                          console.log("[NODE JS INFO HTTP] Error");
                          res.status(420).send('SVM prediction not OK');
                        }
                      });

                    }else {
                     console.error("[NODE JS ERROR] No method for retrieval has been selected");
                     res.status(420).send('No method for retrieval has been selected');
                   }
                   
                 }
               });
            });
      }
    })

  });

  var checkExtensionCloud = function (filename,callback) {
    var extension = filename.split('.').pop();
    if (extension === "obj"){
      console.info("[NODE JS INFO] OBJ file detected - Conversion to PCD file");
      process.obj2pcd(filename,function(isOk) {       
        callback();
      });
    }else {
      callback();
    }
  }

  var transform_cloud = function (filename,tx,ty,tz,rx,ry,rz,callback) {
    process.transform_point_cloud(filename,tx,ty,tz,rx,ry,rz,function(isOk) {       
      callback(isOk);
    });

  }

  /**
  * Run the retrieval similarity search process
  **/
  var startRetrievalSimilaritySearch = function (object_cloud, type_descriptor, k, callback) {
    process.similaritySearchPrediction(object_cloud, trainedDatabase, type_descriptor, k,  function(isOk) {
     callback(isOk);      
   })
  };

  /**
  * Run the retrieval svm search process
  **/
  var startRetrievalSVM = function (object_cloud, type_descriptor, callback) {
   process.svmPrediction(object_cloud, type_descriptor, function(isOk) {
    callback(isOk);      
  }); 
 };

  /**
  * Run the traning of the DB process
  **/
  var startTrainingDB = function (type_descriptor, trainedDatabase, train, buildTree, callback) {
    process.trainDB(type_descriptor, database, trainedDatabase, buildTree, train, function(isOk) {
     callback(isOk);      
   }); 
  };




  module.exports = router;