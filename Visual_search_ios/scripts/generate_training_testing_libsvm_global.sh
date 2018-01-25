#!/bin/bash
let "output"
let "type_descriptor"
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


#### MAIN ######
printf "====> generate training file and testing file in libsvm format for SVM training. Assume that the descriptors have been already computed \n"

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
if [ "$#" -lt 4 ];
    then
        printf "\n-->Please precise the path to the datasets which contains the descriptors and the descriptor wanted. Assume that all the scripts are in the same directory and the structure of the descriptors directory is correct \n"
        printf "$0 descriptors_dataset type_descriptor(esf/vfh/cvvfh/ourcvfh) output multiclass (1/0) \n"
else
    directory_output=$3
    if [ ! -d $directory_output ] ; then
        echo "Output directory is not correct - Abort"
        exit 1
    else
        output=$3
    fi

    echo "DIR COMMAND : "$CURRENT_DIR
    #check if the file already exist. If yes, delete it
    directory_descriptors=$1
    if [ ! -d $directory_descriptors ] ; then
        echo "Directory for descriptors is not correct - Abort"
        exit 1
    else
        type_descriptor=$2
    fi

    multiclass=$4


    ## declare an array of ratio
    declare -a array_ratio=(2 3 5)

    #SCRIPT 1 generate training and testing file
        printf "[INFO] Generate training and testing for ratio 2, 3 and 5 for "$type_descriptor"\n"
        ## now loop over the ratio array
        for i in "${array_ratio[@]}"
        do
            echo "Ratio : $i"
            sh $CURRENT_DIR/generate_files_descriptors_training_testing.sh $directory_descriptors $type_descriptor $i $output
        done
        printf "[INFO] Completed. Training and testing file have been generated (ratio2,3,5) for "$type_descriptor"\n"

    #SCRIPT 2 convert training and testing previously created to training and testing in libsvm format
        for i in "${array_ratio[@]}"
        do
            echo "Ratio : $i"
            descriptor_file_training_ratio=$output"/dataset_descriptor_training_"$type_descriptor"_ratio"$i".txt"
    #echo "Descriptor file training : "$descriptor_file_training_ratio
            descriptor_file_training_libsvm_ratio=$output"/dataset_descriptor_training_"$type_descriptor"_libsvm_ratio"$i".txt"

            descriptor_file_testing_ratio=$output"/dataset_descriptor_testing_"$type_descriptor"_ratio"$i".txt"
            descriptor_file_testing_libsvm_ratio=$output"/dataset_descriptor_testing_"$type_descriptor"_libsvm_ratio"$i".txt"

    #Generate training libsvm file
            $CURRENT_DIR/generate_dataset_libsvm_main $descriptor_file_training_ratio $descriptor_file_training_libsvm_ratio $type_descriptor $multiclass
    #Generate testing libsvm file
            $CURRENT_DIR/generate_dataset_libsvm_main $descriptor_file_testing_ratio $descriptor_file_testing_libsvm_ratio $type_descriptor $multiclass
        done
    printf "[INFO] Completed. Training and testing file LIBSVM have been generated (ratio2,3,5) for "$type_descriptor"\n"

fi


