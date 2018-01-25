#!/bin/bash
let "configuration_file"
let "svm_file"
let "output"
let "type_descriptor"

#### MAIN ######
printf "====> Perform recognition of the point cloud"

# Display name of the script
echo "Name of the script is : $0"
# Affichage du nombre de paramètres
echo "You passed $# parameters"
# List of parameters (one parameter per argument)
echo "Here the list of the parameter (one parameter per argument) :"
for param in "$@"
do
echo -e "\tParameter : $param"
done

#Configuration file : inside, there is the path to the range file, the path to the dataset_categories and the path to the scale program
configuration_file="config.cfg"

#Check that the configuration file is present
if [ ! -f $configuration_file ]; then
    echo "Configuration file not found!"
    exit 1
fi
#Check the number of arguments. Need 3 : point cloud data, svm model, type descriptor and the output
if [ "$#" -lt 3 ]; then
        printf "\n-->Please precise all the parameter\n"
        printf "$0 cloud svm_model_file type_descriptor(esf/vfh/cvfh/ourcvfh)\n"
else
    if [ -f "$1" ]; then
        cloud_file=$1
        #Check if the point cloud has the correct extension
        if ! [ ${cloud_file: -4} == ".pcd" ] && ! [ ${cloud_file: -4} == ".obj" ]; then
            echo "\n Please check the extension. Only pcd and obj are accepted."
            exit 1
        else
            #Get the parameters
            svm_file=$2
            type_descriptor=$3
            #Get the name of the point cloud (without the path)
            filename_point_cloud=$(basename "$cloud_file")
            filename_no_extension="${filename_point_cloud%.*}"
            #Define the txt file which will contains the label and the category predicted
            result_label_file=$filename_no_extension"_result.label"
            output="-output "$result_label_file
            #Execute the C++ program
            #./svm_prediction_main configuration_file svm_model cloud_pcd_data type_descriptor (esf/vfh/cvfh/ourcvfh) output_result
            ./svm_prediction_main $configuration_file $svm_file $cloud_file $type_descriptor $output
        fi
    fi

fi
