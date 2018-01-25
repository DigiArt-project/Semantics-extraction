#!/usr/bin/env python3 -W ignore::DeprecationWarning

import os
from os.path import basename
import random
import argparse
from sklearn.preprocessing import StandardScaler,MinMaxScaler,Normalizer

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
with warnings.catch_warnings():
    warnings.filterwarnings("ignore",category=DeprecationWarning)
    import md5, sha



path_alldesc = ""

def transformStringListToNumberList(data,scale):
    results = list(map(float, data))
    if scale :
        results = MinMaxScaler(feature_range=(0,1)).fit_transform(results)

    return results



def concatenate_descriptors(path,option,scaling):
    for root, directories, filenames in os.walk(path):
        for directory in directories:
            directory = os.path.join(root, directory)
            basename_directory = basename(directory)
            if (basename_directory == "descriptors"):
                if (option == 0):
                    path_alldesc = directory+"/alldesc0"
                elif (option == 1):
                    path_alldesc = directory+"/alldesc1"
                elif (option == 2):
                    path_alldesc = directory+"/alldesc2"
                elif (option == 3):
                    path_alldesc = directory+"/alldesc3"
                elif (option == 4):
                    path_alldesc = directory+"/alldesc4"
                elif (option == 5):
                    path_alldesc = directory+"/alldesc5"
                elif (option == 6):
                    path_alldesc = directory+"/alldesc6"

                if not os.path.exists(path_alldesc):
                    print("[INFO] Create ALLDESC directory")
                    os.makedirs(path_alldesc)

                print(directory)
                """
                for filename in filenames: 
                    print(os.path.join(root,filename))
                """
                
            if (option == 0):
                #print("Concatenation ESF and VFH")
                if (("/esf" in directory or "/vfh" in directory) and (not "/esffull" in directory and not "/vfhfull" in directory)):
                    #print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        
                        for filename1 in filenames1:
                           
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfvfh.txt"
                            #print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                               
                        
                                try:
                                    f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                except:
                                    print("Error detected during opening file!")
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                print(new_filename_tosave)
                                try:
                                    f.write(descriptor_number)
                                except:
                                    print("Error detected during writting!")
                                
                    
            elif (option == 1):
                print("Concatenation ESF and GSHOT")
                if ("/esf" in directory or "/gshot" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfgshot.txt"
                            #print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)

                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)

                    
            elif (option == 2):
                print("Concatenation ESF and GRSD")
                if ("/esf" in directory or "/grsd" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfgrsd.txt"
                            print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)

            elif (option == 3):
                print("Concatenation ESF, GRSD and GSHOT")
                if ("/esf" in directory or "/gshot" in directory or "/grsd" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfgshotgrsd.txt"
                            print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)
                    
            elif (option == 4):
                print("Concatenation ESF, GRSD, GSHOT, VFH")
                if ("/esf" in directory or "/gshot" in directory or "/grsd" in directory or "/vfh" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfgshotgrsdvfh.txt"
                            print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)


            elif (option == 5):
                print("Concatenation GRSD and GSHOT")
                if ("/gshot" in directory or "/grsd" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_gshotgrsd.txt"
                            print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)

            elif (option == 6):
                print("Concatenation ESF, VFH and GSHOT")
                if ("/esf" in directory or "/vfh" in directory or "/gshot" in directory):
                    print("DIRECTORY : {} ".format(directory))
                    path = directory
                            
                    for root1, directories1, filenames1 in os.walk(path):
                        for filename1 in filenames1:
                            file = os.path.join(root1, filename1)
                            filename_basename = basename(file)
                            filename_basename = os.path.splitext(filename_basename)[0]
                            new_filename_tosave = path_alldesc + "/" + filename_basename + "_esfvfhghot.txt"
                            print(new_filename_tosave)
                            with open(file) as f:
                                content = f.readlines()
                                # you may also want to remove whitespace characters like `\n` at the end of each line
                                content = [x.strip() for x in content]
                                #Item 11 in the pcd file
                                descriptor = content[11]
                                descriptor_number = descriptor.split(" ")
                                descriptor_number = transformStringListToNumberList(descriptor_number,scaling)
                                descriptor_number = ' '.join(str(e) for e in descriptor_number)
                                if os.path.isfile(new_filename_tosave):
                                    os.remove(new_filename_tosave)

                                f = open(new_filename_tosave,"a") #opens file with name of "test.txt"
                                if (os.stat(new_filename_tosave).st_size != 0):
                                    f.write(" ")
                                f.write(descriptor_number)



def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    print("This programs allows to concatenate some descriptors in one file. It can scale descriptors independently before concatening each one. \n")
    print("Assume you have folders : esf, grsd, vfh, gshot. According to the choice you made, loop over each folder, take each pcd descriptor, scale it and write into another file inside a new folder call alldesc \n")
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--dataset", required=True, help="the path to the descriptors dataset")
    ap.add_argument("-c", "--option_concatenation", required=True, help="0 --> ESF and VFH, 1 --> ESF and GSHOT, 2 -->  ESF and GRSD, 3 -->  ESF, GRSD and GSHOT, 4 --> ESF, GRSD, GSHOT, VFH, 5 --> GRSD and GSHOT, 6 --> ESF, VFH and GShot ")
    ap.add_argument("-s", "--scale", required=False, default =True,type=str2bool,help="Scale descriptors between 0 and 1. Defaut : True")

    args = vars(ap.parse_args())

    dataset_path = args["dataset"]
    option = int(args["option_concatenation"])
    scale = args["scale"]
    print("VALUE SCALE : {}".format(scale))

    concatenate_descriptors(dataset_path,option,scale)


if __name__ == '__main__':
    main()