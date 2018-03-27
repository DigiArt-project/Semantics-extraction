# Visual Search App version 0.1

## Structure of the repo
=======
This is the code for 3D object classification and retrieval using the structure sensor device.
There are three main part :  

* **iOS App** 
* **Node JS Server**
* **C++ Core algorithm**

## Hierarchy 

There are five importants components :

* **visual_search** —> iOS app for Ipad
* **server** —> server side with nodeJS. Interact with the iOS app and the C++ algorithm
* **Datasets** —> Structure sensor dataset along with the descriptors 
* **training_database** —> the C++ algo (retrieval using similarity search and svm)
* **scripts** —> All the scripts needed for generating files, training and testing

In each folder, you will find instructions.

## How to run the code on your own server 

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

