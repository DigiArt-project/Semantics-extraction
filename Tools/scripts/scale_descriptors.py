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



def transformStringListToNumberList(data,min_range,max_range):
    results = list(map(float, data))
    results = MinMaxScaler(feature_range=(min_range,max_range)).fit_transform(results)

    return results



def scale_descriptors(path_dataset,name_descriptor,min_range,max_range):
    path_alldesc = ""
    for root, directories, filenames in os.walk(path_dataset):
        for directory in directories:
            directory = os.path.join(root, directory)
            basename_directory = basename(directory)
            #if (basename_directory == "descriptors"):
            #        path_alldesc = directory+"/"+name_descriptor
              
            if (name_descriptor in directory):
                path = directory
                            
                for root1, directories1, filenames1 in os.walk(path):
                        
                    for filename1 in filenames1:
                           
                        file = os.path.join(root1, filename1)
                        filename_basename = basename(file)
                        filename_basename = os.path.splitext(filename_basename)[0]
                        path_file = file
                        print("Processing : {} ".format(file))
                        with open(file) as f:
                            content = f.readlines()
                            #you may also want to remove whitespace characters like `\n` at the end of each line
                            content = [x.strip() for x in content]
                            descriptor = content[0]
                            descriptor_number = descriptor.split(" ")
                            
                            descriptor_number = transformStringListToNumberList(descriptor_number,min_range,max_range)
                            descriptor_number = ' '.join(str(e) for e in descriptor_number)
                            #print(descriptor_number)
                            
                        #Remove file and then write a new file with the scaled descriptor
                        os.remove(path_file)
                        try:
                            f = open(path_file,"a") #opens file
                        except:
                            print("Error detected during opening file!")
                            if (os.stat(path_file).st_size != 0):
                                f.write(" ")
                            print(path_file)
                        try:
                            f.write(descriptor_number)
                        except:
                            print("Error detected during writting!")


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    min_range = 0
    max_range = 1
    print("Scale descriptors \n")
    ap = argparse.ArgumentParser()
    ap.add_argument("-dataset", "--dataset", required=True, help="the path to the descriptors dataset")
    ap.add_argument("-descriptor", "--descriptor", required=True, help="Name of the descriptor")
    ap.add_argument("-min", "--min", required=False, default=0,type=int,help="Min range for scaling. Defaut : 0")
    ap.add_argument("-max", "--max", required=False, default=1,type=int,help="Max range for scaling. Defaut : 1")

    args = vars(ap.parse_args())

    dataset_path = args["dataset"]
    name_descriptor = args["descriptor"]

    if args["min"] is not None:
        min_range = args["min"]
    if args["max"] is not None:
        max_range = args["max"]

   

    scale_descriptors(dataset_path,name_descriptor,min_range,max_range)


if __name__ == '__main__':
    main()