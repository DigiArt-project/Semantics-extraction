#!/bin/bash

let "total_descriptor = 0"
let "total_categories = 0"
let "dataset_path_root"
let "output"
let "type_descriptor"
let "ratio_count =0"
let "path_to_save"
let "path_to_file_training"
let "path_to_file_testing"
let "numberTraining = 0"
let "numberTesting = 0"
let "count = 0"

# contains(string, substring)
#
# Returns 0 if the specified string contains the specified substring,
# otherwise returns 1.
contains() {
    string="$1"
    substring="$2"
    if test "${string#*$substring}" != "$string"
    then
        return 0    # $substring is in $string
    else
        return 1    # $substring is not in $string
    fi
}

# loop & print a folder recusively,
print_folder_recurse() {
    for i in "$1"/*;do
            #Check if it's a directory
        if [ -d "$i" ];then
            path_dir_string=$i
            #the path is something like database/categorie_i/type_Descriptor. We want to remove the prefixe database in order to keep only the second part, i.e categorie_i/type_descriptor
            categorie=${path_dir_string#$dataset_path_root/}
            #Then we keep only the categorie of the object by taking the first word i.e categorie_i
            if ! [[ $categorie == *"/"* ]]
            then
                if [ ! "$categorie" = "training_esf" ]  && [ ! "$categorie" = "training_vfh" ]  && [ ! "$categorie" = "training_cvfh" ] && [ ! "$categorie" = "training_ourcvfh" ]; then


                    echo "$categorie" >> $path_to_save
                    echo "$categorie" >> $path_to_file_testing
                    echo "$categorie" >> $path_to_file_training
                    echo "Category: $categorie"
                    total_categories=$((total_categories + 1))

                fi
            fi
            print_folder_recurse "$i"
        #Then for each folder, check if the file insite are file
        elif [ -f "$i" ]; then
            if contains "$i" "/$type_descriptor/"; then
                    if contains "$i" ".pcd"; then
                        path_string=$i
                        echo $PWD/"$path_string" >> $path_to_save
                        echo "Found : $path_string"
                        total_descriptor=$((total_descriptor + 1))
                        #For testing, take 1/3 (per defaut)
                        if [ $(($count%$ratio_count)) -eq 0 ]; then
                            numberTesting=$((numberTesting + 1))
                            echo $PWD/"$path_string" >> $path_to_file_testing

                        else
                            numberTraining=$((numberTraining + 1))
                            echo $PWD/"$path_string" >> $path_to_file_training

                        fi
                    fi
            fi

        fi
        count=$((count+1))
    done
}

#### MAIN ######
printf "====> Create a txt file which contains the path to every descriptors i. The results dataset must follow this structure : \nDataset/Categorie_i/type_descriptor with type_descriptor = [ vfh, esf, CVFH, OURCVFH] <==== \n ./script dataset_path extension (optionel parameter). Extension needed for descriptors need to be pcd  \n"

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

if [ "$#" -lt 3 ];
then
    printf "\n-->Please precise the path to the results which contains the descriptors and the descriptor wanted \n"
    printf "$0 dataset_path esf/vfh/cvvfh/ourcvfh ratio(nb training/testing. Defaut : 3) output\n"
    exit 1
else
    #check if the file already exist. If yes, delete it
    if [ -f $file ] ; then
        rm $file
    fi

    # try get path from param
    path=""
    if [ -d "$1" ]; then
        path=$1;
        dataset_path_root=$1
        type_descriptor=$2
        if [ "$#" -eq 3 ]; then
            ratio_count=3;
        else
            ratio_count=$3;
        fi
    if [ -d "$4" ]; then
        output=$4
    else
        printf "\n The output must be a directory \n"
        exit 1
    fi

    file="dataset_descriptor_"
    file+=$type_descriptor".txt";
    path_to_file_training=$output"/dataset_descriptor_training_"
    path_to_file_training+=$type_descriptor"_ratio"$ratio_count".txt";
    path_to_file_testing=$output"/dataset_descriptor_testing_"
    path_to_file_testing+=$type_descriptor"_ratio"$ratio_count".txt";
    path_to_save=$output"/"$file
    print_folder_recurse $path $type_descriptor $ratio_count

    echo "Path to save $path_to_save"

    echo "Number Training file: ${numberTraining}"
    echo "Number Testing file: ${numberTesting}"
    echo "Total descriptor: ${total_descriptor}"
    fi
fi
