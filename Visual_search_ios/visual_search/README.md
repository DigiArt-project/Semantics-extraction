# iOS app

## How to build the code

The following dependencies must be satisfied : 

* Xcode 8.1
* SDK 10.2 for iOS
* Structure sensor framework 
* [POD](https://cocoapods.org/)

Then, on the current directory, run the following command line:

`pod install` 

This command line will install the librairies needed and generate the project correctly using these dependencies.

If everything is fine, a green message will appareared on the terminal and a new file will be generated ==*.xcworkspace==

Click on this new file.

Build it using XCode.

Transfer to iPad device.

## Remark

### Connexion 

You must change the IP in the **Utils.h** file to your corresponding IP.

### Add new categories for UI representation

Edit the **categories.json** file in order to add or remove a category.