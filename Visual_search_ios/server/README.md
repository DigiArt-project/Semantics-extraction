# Node JS Server side

==Binaries inside the binaries folder are from the core folder. So you need to compile C++ program inside CORE folder and them copy past the binaries compiled into the binaries folder of the server== 

Server side handle by **Node JS**  

In order to make the iOS app work, the server need to be run.

This part concern where the point cloud is send from the Ipad to the server in order to perform the computations on the fly (retrieval using similarity search or svm)  

After having scanned the object, the user need to send it to the server in order to make the computation.

The protocole used is HTTP.

**HTTP POST** for sending data to the server.

**HTTP GET** for getting data from the server.
## Brief explanations of the different folders

* **Module:** contains the code which make the call to the C++ binary for similarity search or SVM
* **public** : HTML pages NodeJS
* **results** : Two json file. One which corresponds to the similarity search result and another one which correspond to SVM result (label)
* **routes** : Differents NodeJS routes. One route called upload.js which take care of the upload part. Another one called download.js which take care of the download part. And finally another one index.js which make the reference to both routes upload and download. It's the interface.
* **uploads:** Contains the uploaded point cloud 
* **view :** Differents .jade view for nodeJS

## How to run the code on your own server 

### Installation of the Node JS module dependencies
In the root, execute the following command :

	npm install

### Ports and IP

Per defaut, the port 8080 is used. Therefore you need to open it.

If you want to use another port, open the file **app.js** in the server folder and change the following line :

	var port = 8080; 
	app.listen( port, function(){ 
  	console.log('listening on port '+port); 
	});

Then open **Utils.hpp** file in iOS app and change port number.
`"http://ip:8080/";`

Change as well the ip with the IP of your server.

### Send to the server the appropriate code

Thank to the `scp` command line, send to the server at the location `/var/www` all the code.

At the end of the process you need to have
	
	/var/www/core
	/var/www/Datasets
	/var/server
	/var/svm
	
For example, assume that my code are in my HOME folder. In order to send to my server called **adressServer** the folder core I will have to execute this following command line  :

`scp -r core/ username@adressServer:/var/www/core/
`

### Dependencies for the server 

The following dependencies are required in order to compile the code :


* BOOST 1.59
* OPEN MP
* PCL 1.8
* OpenCV 2.4
* libSVM 
* HDF5 
* FLANN

LibSVM should be located in `core/3rdparty/`. If it's not the case, you need to download it.

#### Compilation of core C++
In the **core folder**, create a directory called **build** and go inside it.
You can do that by doing the following command line :

`mkdir build && cd build`

Then :

	cmake ..
	make
	
If all the dependecies are okay, the program will compile with succees and you should see lots of binaries programms in the build folder.

#### Compilation of libsvm

Go to `core/3rdparty/libsvm` and do the following command line : `make`

#### Execute Node JS 

In your server, go to `/var/wwww/server` and run the NodeJS programm by doing `node app.js`

If you want to run non-stop the NodeJS server, you can use 
[forever](https://github.com/foreverjs/forever) and do `forever start app.js `

Then you can finally use the iOS App and perform the retrieval process on the cloud


## Running the server 
First Node JS need to be installed.

Then the servan can be runned from the project directory by using the following command line :

`node app.js`  
