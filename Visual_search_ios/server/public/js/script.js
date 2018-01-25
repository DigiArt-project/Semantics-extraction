// common variables
var iBytesUploaded = 0;
var iBytesTotal = 0;
var iPreviousBytesLoaded = 0;
var oTimer = 0;
var sResultFileSize = '';
function secondsToTime(secs) { // we will use this function to convert seconds in normal time format
	var hr = Math.floor(secs / 3600);
	var min = Math.floor((secs - (hr * 3600))/60);
	var sec = Math.floor(secs - (hr * 3600) -  (min * 60));
	if (hr < 10) {hr = "0" + hr; }
	if (min < 10) {min = "0" + min;}
	if (sec < 10) {sec = "0" + sec;}
	if (hr) {hr = "00";}
	return hr + ':' + min + ':' + sec;
};
function bytesToSize(bytes) {
	var sizes = ['Bytes', 'KB', 'MB'];
	if (bytes == 0) return 'n/a';
	var i = parseInt(Math.floor(Math.log(bytes) / Math.log(1024)));
	return (bytes / Math.pow(1024, i)).toFixed(1) + ' ' + sizes[i];
};
function fileSelected() {
	var isOkay = checkFile();
	if (isOkay == false){
		return;
	}else {
	// hide different warnings
	document.getElementById('upload_response').style.display = 'none';
	document.getElementById('error').style.display = 'none';
	document.getElementById('error2').style.display = 'none';
	document.getElementById('abort').style.display = 'none';
	document.getElementById('warnsize').style.display = 'none';
	// get selected file element
	var oFile = document.getElementById('cloud').files[0];

	// get preview element
	var oImage = document.getElementById('preview');
	// prepare HTML5 FileReader
	var oReader = new FileReader();
	oReader.onload = function(e){
	// e.target.result contains the DataURL which we will use as a source of the image
	oImage.src = e.target.result;
	oImage.onload = function () { // binding onload event
	// we are going to display some custom image information here
	sResultFileSize = bytesToSize(oFile.size);

	document.getElementById('fileinfo').style.display = 'block';
	document.getElementById('filename').innerHTML = 'Name: ' + oFile.name;
	document.getElementById('filesize').innerHTML = 'Size: ' + sResultFileSize;
	document.getElementById('filetype').innerHTML = 'Type: ' + oFile.type;
	document.getElementById('filedim').innerHTML = 'Dimension: ' + oImage.naturalWidth + ' x ' + oImage.naturalHeight;
};

}
};
// read selected file as DataURL
oReader.readAsDataURL(oFile);
}
function startUploading() {
	//Check extension
	var file = document.getElementById('cloud').files[0];
	if ( file == null ){
		alert("No file selected !");
		document.getElementById("uploadFile").disabled = false; 
		return;
  // some_variable is either null or undefined
	}

		// cleanup all temp states
		iPreviousBytesLoaded = 0;
		document.getElementById('upload_response').style.display = 'none';
		document.getElementById('error').style.display = 'none';
		document.getElementById('error2').style.display = 'none';
		document.getElementById('abort').style.display = 'none';
		document.getElementById('warnsize').style.display = 'none';
		document.getElementById('progress_percent').innerHTML = '';
		var oProgress = document.getElementById('progress');
		oProgress.style.display = 'block';
		oProgress.style.width = '0px';
		// get form data for POSTing
		//var vFD = document.getElementById('upload_form').getFormData(); // for FF3
		var vFD = new FormData(document.getElementById('upload_form')); 
		// create XMLHttpRequest object, adding few event listeners, and POSTing our data
		var oXHR = new XMLHttpRequest();        
		oXHR.upload.addEventListener('progress', uploadProgress, false);
		oXHR.addEventListener('load', uploadFinish, false);
		oXHR.addEventListener('error', uploadError, false);
		oXHR.addEventListener('abort', uploadAbort, false);
		oXHR.open('POST', 'api/upload/action_upload');
		oXHR.send(vFD);
		// set inner timer
		oTimer = setInterval(doInnerUpdates, 300);

	}
function doInnerUpdates() { // we will use this function to display upload speed
	var iCB = iBytesUploaded;
	var iDiff = iCB - iPreviousBytesLoaded;
// if nothing new loaded - exit
if (iDiff == 0)
	return;
iPreviousBytesLoaded = iCB;
iDiff = iDiff * 2;
var iBytesRem = iBytesTotal - iPreviousBytesLoaded;
var secondsRemaining = iBytesRem / iDiff;
// update speed info
var iSpeed = iDiff.toString() + 'B/s';
if (iDiff > 1024 * 1024) {
	iSpeed = (Math.round(iDiff * 100/(1024*1024))/100).toString() + 'MB/s';
} else if (iDiff > 1024) {
	iSpeed =  (Math.round(iDiff * 100/1024)/100).toString() + 'KB/s';
}
document.getElementById('speed').innerHTML = iSpeed;
document.getElementById('remaining').innerHTML = '| ' + secondsToTime(secondsRemaining);        
}
function uploadProgress(e) { // upload process in progress
	if (e.lengthComputable) {
		iBytesUploaded = e.loaded;
		iBytesTotal = e.total;
		var iPercentComplete = Math.round(e.loaded * 100 / e.total);
		var iBytesTransfered = bytesToSize(iBytesUploaded);
		document.getElementById('progress_percent').innerHTML = iPercentComplete.toString() + '%';
		document.getElementById('progress').style.width = (iPercentComplete * 4).toString() + 'px';
		document.getElementById('b_transfered').innerHTML = iBytesTransfered;
		if (iPercentComplete == 100) {
			var oUploadResponse = document.getElementById('upload_response');
			oUploadResponse.innerHTML = '<h1>Please wait...processing</h1>';
			oUploadResponse.style.display = 'block';
		}
	} else {
		document.getElementById('progress').innerHTML = 'unable to compute';
	}
}
function uploadFinish(e) { // upload successfully finished
	var oUploadResponse = document.getElementById('upload_response');
	oUploadResponse.innerHTML = e.target.responseText;
	oUploadResponse.style.display = 'block';
	document.getElementById('progress_percent').innerHTML = '100%';
	document.getElementById('progress').style.width = '400px';
	document.getElementById('filesize').innerHTML = sResultFileSize;
	document.getElementById('remaining').innerHTML = '| 00:00:00';
	clearInterval(oTimer);
	httpGetAsyncResults('/api/results/',function (result){
		var checkedValueMethod = null; 
		var inputElements = document.getElementsByName('method_search_selected');
		for(var i=0; inputElements[i]; ++i){
			if(inputElements[i].checked){
				checkedValueMethod = inputElements[i].value;
				break;
			}
		}
		console.log("result GET : " + result);
		if (checkedValueMethod == "svm"){
			var string_res =result.split("\n");
			var stringArray_res = new Array();
			for(var i =0; i < string_res.length; i++){
				stringArray_res.push(string_res[i]);
				console.log(string_res[i]);
			}
			oUploadResponse.innerHTML = "Result Prediction : " + stringArray_res[1];

		}
		else {
			oUploadResponse.innerHTML = "Result Similarity Search : ";
			var string_res =result.split("},");
			var stringArray_res = new Array();
			for(var i =0; i < string_res.length; i++){
				stringArray_res.push(string_res[i]);
				console.log(string_res[i]);
				oUploadResponse.innerHTML = oUploadResponse.innerHTML + "<p>" + stringArray_res[i]; "</p>" 
			}
		}

		
		
	});
}
function uploadError(e) { // upload error
	document.getElementById('error2').style.display = 'block';
	clearInterval(oTimer);
}  
function uploadAbort(e) { // upload abort
	document.getElementById('abort').style.display = 'block';
	clearInterval(oTimer);
}

function httpGetAsyncResults(theUrl, callback)
{
	var xmlHttp = new XMLHttpRequest();
	xmlHttp.onreadystatechange = function() { 
		if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
			callback(xmlHttp.responseText);
	}
    xmlHttp.open("GET", theUrl, true); // true for asynchronous 
    xmlHttp.send(null);
}

function checkFile() {
	var fileElement = document.getElementById('uploadFile');
        //var fileElement = document.getElementById("uploadFile");
        var fileExtension = "";
        console.log("fileElement: " + fileElement);
        
        var oFile = document.getElementById('cloud').files[0];
        console.log("ofile : " + oFile);
        console.log("ofile name : " + oFile.name);
        if (oFile.name.lastIndexOf(".") > 0) {
        	fileExtension = oFile.name.substring(oFile.name.lastIndexOf(".") + 1, oFile.name.length);
        	console.log(fileExtension);
        }
        if (fileExtension.toLowerCase() == "pcd" || fileExtension.toLowerCase() == "obj") {
        	document.getElementById("uploadFile").disabled = false; 
        	return true;
        }
        else {
        	alert("You must select a pcd/obj file for cloud upload");
        	document.getElementById("uploadFile").disabled = true; 
        	return false;
        }
    }