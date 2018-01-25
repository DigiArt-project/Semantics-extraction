/**
 * @author Lirone Samoun
 * @brief Initial route.
 */


// Requirements
var express = require('express');
var router = express.Router();
var process = require('../modules/retrieval');
var _ = require('underscore');
var fs = require('fs');
var path = require('path');


//http://localhost:8080/api/results
router.get('/', function(req, res){
  //__dirname The name of the directory that the currently executing script resides in.
  // ./ refer to the directory from which you ran the node command in your terminal window (i.e. your working directory)
  var path ='./results/';
  var recentJSONResults = getMostRecentFileName('./results/');
  var path_to_result = path + recentJSONResults;
  fs.readFile(path_to_result, 'utf8', function (err,data) {
    if (err) {
      return console.log(err);
    }
    console.log("DATA : ",data);
  });
  // Set disposition and send it.
  res.download(path_to_result, function(err){
    if (err) {
      console.log("[NODE JS ERROR] ERROR during the download of results");
      console.log(err);
    } else {
      console.log("[NODE JS INFO] Download completed");
    }
  });
});

// Return only base file name without dir
function getMostRecentFileName(dir) {
    var files = fs.readdirSync(dir);

    // use underscore for max()
    return _.max(files, function (f) {
        var fullpath = path.join(dir, f);

        // ctime = creation time is used
        // replace with mtime for modification time
        return fs.statSync(fullpath).ctime;
    });
}



module.exports = router;