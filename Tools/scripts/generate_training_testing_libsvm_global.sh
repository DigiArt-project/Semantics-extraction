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
if [ "$#" -lt 1 ];
    then
        printf "\n-->Please precise the path to the datasets which contains the descriptors. Assume that all the scripts are in the same directory and the structure of the descriptors directory is correct \n"
        printf "$0 descriptors_dataset\n"
else

    descriptor_dataset=$1;
    #declare -a array_descriptor=('esf' 'vfh' 'ourcvfh' 'cvfh' 'gshot' 'grsd' 'spin' 'egi' 'good' 'sc3D' 'scurv')
    declare -a array_descriptor=('esf' 'vfh' 'ourcvfh' 'cvfh' 'gshot' 'gshotPyramid' 'good' 'sc3D' 'egi' 'spin' 'pointnet' 'grsd')

    ## declare an array of ratio
    #declare -a array_option_concatenation=(0 1 2 3 4 5 6)
    #declare -a array_name_concatenation=('esfvfh' 'esfgshot' 'esfgrsd' 'esfgrsdgshot' 'esfgshotgrsdvfh' 'grsdgshot' 'esfvfhgshot')

    #SCRIPT 1 concatenate descriptors if it's not the case yet
    #    printf "[INFO] Concatenation of descriptors 0 --> ESF and VFH, 1 --> ESF and GSHOT, 2 -->  ESF and GRSD, 3 -->  ESF, GRSD and GSHOT, 4 --> ESF, GRSD, GSHOT, VFH, 5 --> GRSD and GSHOT, 6 -->  ESF, VFH and GSHOT"
        ## now loop over the ratio array
    #    for i in "${array_option_concatenation[@]}"
    #    do
    #        echo "Option : $i"
    #        python3 $CURRENT_DIR/concatenate_descriptor.py -d $directory_descriptors -s True -c $i
    #    done
    #    printf "[INFO] Completed. Concatenation have been done"


    #SCRIPT 2 create list path corresponding to each concatenation descriptor
        for i in "${array_descriptor[@]}"
            do
            echo "Option : $i"
            $CURRENT_DIR/create_list_path_desriptors.sh $descriptor_dataset $i

            $CURRENT_DIR/generate_libsvm_data_main "dataset_descriptor_"$i".txt" "descriptors_"$i".txt" $i 1

        done
    printf "[INFO] Completed. Training file LIBSVM have been generated for descriptors concatenated \n"


    #SCRIPT 2 create list path corresponding to each concatenation descriptor
    #    for i in "${array_option_concatenation[@]}"
    #        do
    #        echo "Option : $i"
    #        $CURRENT_DIR/create_list_path_desriptors.sh $descriptor_dataset "alldesc"$i

    #        $CURRENT_DIR/generate_libsvm_data_main "dataset_descriptor_"${array_name_concatenation[$i]}".txt" "descriptors_"${array_name_concatenation[$i]}".txt" all 1

     #       mv "dataset_descriptor_alldesc"$i".txt" "dataset_descriptor_"${array_name_concatenation[$i]}".txt"

      #  done
    #printf "[INFO] Completed. Training file LIBSVM have been generated for descriptors concatenated \n"

fi


