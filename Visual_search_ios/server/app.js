
/**
 * Dependencies
 */
var express = require('express');
var http = require('http');
var util = require('util');
var path = require('path');
var bodyParser = require('body-parser');
var cookieParser = require('cookie-parser');
var multipart = require('connect-multiparty');
var multipartMiddleware = multipart();
var fs = require("fs");



/**
 * Importation of routes
 */

var index = require('./routes/index');
var upload = require('./routes/upload');
var download = require('./routes/download');
var optionsTraining = require('./routes/optionsTraining');

/**
 * Express
 */
var app = express();
var router = express.Router();

app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: true}));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use(function(err, req, res, next) {
  console.error(err.stack);
  res.status(500).send('Something broke!');
});


/**
 * Views
 */
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');


/**
 * Routes
 */

app.use('/', index);
//This app will respond to ip/upload/
app.use('/api/upload', upload);
//This app will respond to ip/download-results/
app.use('/api/results', download);
//This app will respond to ip/utilities/
app.use('/api/utilities', optionsTraining);

app.set('port', process.env.PORT || 8080)
var port = app.get('port');
//var port = 8080;
app.listen( port, function(){ 
  console.log('listening on port '+port); 
});

module.exports = app;