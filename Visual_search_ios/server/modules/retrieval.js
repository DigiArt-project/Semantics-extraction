	var exec = require('child_process').spawn;

	var path_output_result = "./results/";
	var path_program = "./binaries/";
	var core_config = "config.cfg";

	/**
	 * Function which call the C++ programm with parameters corresponding to the training of the dataset
	 * @param type_descriptor : esf, cvfh, vfh, ourcvfh, 
	 * @param trainDatabase 1 or 0
	 * @param buildTree 1 or 0
	 */
	 var trainDB = function (type_descriptor, database, trainedDatabase, buildTree, train, callback) {
	 	var program = path_program + train_similaritySearch;
	 	var tesselation_level = 1;
	 	if (type_descriptor !== "esf" && type_descriptor !== "vfh" && type_descriptor !== "cvfh" && type_descriptor !== "ourcvfh"){
	 		console.error("[NODE JS ERROR] Unknown descriptor type"); 
	 		return;
	 	}else {
			//Call the command line of the C++ programm
			var cmd = program + " -database "+  database + " -trained " + trainedDatabase + " -descriptor " + type_descriptor + " -build_tree " + buildTree + " -train " + train + " -tesselation_level" + tesselation_level;
			console.log(cmd);
			var process = exec(cmd,  {maxBuffer: 1024 * 500},(error, stdout, stderr) => {
				if (error) {
					console.error(`exec error: ${error}`);
					return;
				}
				console.log(`stdout: ${stdout}`);
				console.log(`stderr: ${stderr}`);
			});

		}
	}

	/**
	 * Function which call the C++ programm with parameters corresponding to retrieval process
	 * @param object_cloud : name of the pcd file (assuming that the file is inside the uploads folder)
	 */
	 var similaritySearchPrediction = function (object_cloud, trainedDatabase, type_descriptor, k, callback){
	 console.log('[NODE JS INFO] trained database path : ' + trainedDatabase);
	  //The binary of the C++ program
	  var program = path_program + "retrieval_similaritySearch";
		//The cloud we want to retrieve in the database
		var myCloud = "uploads/";
		myCloud += object_cloud;
	  //__dirname The name of the directory that the currently executing script resides in.
	  // ./ refer to the directory from which you ran the node command in your terminal window (i.e. your working directory)
	  var objectCloud_without_extension = object_cloud.slice(0, -4);
	  var output_result = path_output_result + objectCloud_without_extension+"_result.json"
	   console.log('[NODE JS INFO]Output result : ' + output_result);

	  //arguments
	  const args = [
	  core_config,
	  "-query", 
	  myCloud,
	  "-trained", 
	  trainedDatabase,
	  "-k",
	  k,
	  "-descriptor",
	  type_descriptor,
	  "-output",
	  output_result,
	  "-compute_full",
	  "false",
	  ];
	  const proc = exec(program, args,  {maxBuffer: 1024 * 500});

	  proc.stdout.on('data',
	  	function (data) {
	  		console.log('Similarity Search prediction stdcout: ' + data);
	  	});
	  proc.stderr.on('data', function (data) {
	        //throw errors
	        console.log('[ERROR] Similarity Search prediction stderr: ' + data);
	    });

	  proc.on('close', function (code) {
	  	console.log('Similarity Search prediction process exited with code ' + code);
	  	if (code == 0 ){
	 			callback(true);
	 		}else {
	 			callback(false);
	 		}
	  });

	}

	/**
	 * Function which call the C++ programm with parameters corresponding to retrieval process svm
	 * @param object_cloud : name of the pcd file (assuming that the file is inside the uploads folder)
	 */
	 var svmPrediction = function (object_cloud, type_descriptor, type_dataset, trainedDatabase, callback){
	 	console.log('[SVM prediction] Type descriptor: ' + type_descriptor);
	 	console.log('[SVM prediction] Type dataset: ' + type_dataset);
	 	console.log('[SVM prediction] Path dataset: ' + trainedDatabase);

	 	var program = path_program + "svm_prediction_main";
	  	//The cloud we want to retrieve in the database
	  	var myCloud = "uploads/";
	  	myCloud += object_cloud;
	  	//__dirname The name of the directory that the currently executing script resides in.
	  	// ./ refer to the directory from which you ran the node command in your terminal window (i.e. your working directory)
	  	var results = "./results/";
	  	var objectCloud_without_extension = object_cloud.slice(0, -4);
	  	results += objectCloud_without_extension+"_result.json"
	  	var path_to_svm = "../svm/svm_"+type_dataset+"_"+type_descriptor+".model";
	  	var path_to_range_file = "../svm/training_"+type_dataset+"_"+type_descriptor+".range";
	  	var path_to_categories_file = trainedDatabase + "dataset_" + type_dataset + "_categories.txt";

	  	var output_result = "./results/";
	  	var objectCloud_without_extension = object_cloud.slice(0, -4);
	  	output_result += objectCloud_without_extension+"_result.label"

	  	const args = [
	  	core_config,
	  	path_to_svm,
	  	path_to_range_file, 
	  	myCloud, 
	  	type_descriptor,
	  	path_to_categories_file,
	  	"-output",
	  	output_result,
	  	];

	  	const proc = exec(program, args,  {maxBuffer: 1024 * 500});

	  	proc.stdout.on('data',
	  		function (data) {
	  			console.log('svm prediction stdcout: ' + data);
	  		});
	  	proc.stderr.on('data', function (data) {
	    //throw errors
	    console.log('[NODE JS ERROR] svm prediction stderr: ' + data);
		});

	  	proc.on('close', function (code) {
	  		console.log('svm prediction process exited with code ' + code);
	  		if (code == 0 ){
	 			callback(true);
	 		}else {
	 			callback(false);
	 		}
	  	});

	}


	/**
	 * obj2pcd function
	 * convert obj file to pcd
	 */
	 var obj2pcd = function (filename, callback) {
	 	var program = path_program + "obj2pcd";
	 	var uploadPath = "uploads/";
	 	var myCloud = uploadPath + filename;

	 	var objectCloud_without_extension = filename.slice(0, -4);
	 	var pcdCloud = objectCloud_without_extension + ".pcd";
	 	pcdCloud = uploadPath + pcdCloud;

	 	const args = [
	 	myCloud, 
	 	pcdCloud, 
	 	'-copy_normals 1',
	 	];
	 	const proc = exec(program,args)

	 	proc.stdout.on('data',
	 		function (data) {
	 			console.log('obj2pcd C++ stdcout: ' + data);
	 		});
	 	proc.stderr.on('data', function (data) {
	        //throw errors
	        console.log('[ERROR] obj2pcd stderr: ' + data);
	    });

	 	proc.on('close', function (code) {
	 		console.log('obj2pcd process exited with code ' + code);
	 		if (code == 0 ){
	 			callback(true);
	 		}else {
	 			callback(false);
	 		}
	 	});

	 };

	/**
	 * transform_point_cloud function
	 * translate and rotate point cloud
	 */
	 var transform_point_cloud = function (filename,tx,ty,tz,rx,ry,rz, callback) {
	 	var program = path_program + "transform_point_cloud_main";
	 	var uploadPath = "uploads/";
	 	var myCloud = uploadPath + filename;

	 	var objectCloud_without_extension = filename.slice(0, -4);
	 	var pcdCloud = objectCloud_without_extension + ".pcd";
	 	pcdCloud = uploadPath + pcdCloud;

	 	const args = [
	 	"-query",myCloud, 
	 	"-output",myCloud, 
	 	tx,
	 	ty,
	 	tz,
	 	rx,
	 	ry,
	 	rz,
	 	];
	 	console.log("Transformation : " + tx + " " + ty + " " + tz + " " + rx + " " + ry + " " + rz + " ");
	 	const proc = exec(program, args,  {maxBuffer: 1024 * 500});

	 	proc.stdout.on('data',
	 		function (data) {
	 			console.log('transform_point_cloud stdcout: ' + data);
	 		});
	 	proc.stderr.on('data', function (data) {
	        //throw errors
	        console.log('[ERROR] transform_point_cloud stderr: ' + data);
	    });

	 	proc.on('close', function (code) {
	 		console.log('Transform_point_cloud process exited with code ' + code);
	 		if (code == 0 ){
	 			callback(true);
	 		}else {
	 			callback(false);
	 		}
	 	});
	 };



	//Export function
	exports.trainDB = trainDB;
	exports.similaritySearchPrediction = similaritySearchPrediction;
	exports.svmPrediction = svmPrediction;
	exports.obj2pcd = obj2pcd;
	exports.transform_point_cloud = transform_point_cloud