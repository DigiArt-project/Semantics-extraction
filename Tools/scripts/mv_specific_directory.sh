#!/bin/bash

let "dataset_path_root"
let "dataset_path_root_output"
let "name_category"
let "count = 0"
let "count_category = 0"

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
            name_category=$categorie
            #echo "Path $path_dir_string"

            if contains "$name_category" "/$2" ; then
                input_without_first_part=${path_dir_string#*/}
                path_output_to_save="$dataset_path_root_output/$input_without_first_part"
                echo "Going to save $path_dir_string to $path_output_to_save"
                cp -Rf "$path_dir_string/" "$path_output_to_save/"
            fi
            #Then we keep only the categorie of the object by taking the first word i.e categorie_i
            print_folder_recurse "$i" "$2"
            count_category=$((count_category+1))
         fi
        count=$((count+1))
    done
}

#### MAIN ######
printf "====> Move specific folder and its file to another same parent directory"

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
    printf "\n-->This script allows to move specifc subfolder with a specific parent directory to another same parent directory. For example you have a folder named descriptor which "
    printf "contain desc1 and desc2. It will move desc1 or desc2 to the folder descriptor of the other directory"
    printf "\n \n-->Please precise  \n"
    printf "$0 dataset_path_input specfic_folder_to_move dataset_path_output\n"
else

    # try get path from param
    path_dataset_input=""
    if [ -d "$1" ]; then
        path_dataset_input=$1;
        folder_to_move=$2
        dataset_path_root=$1
        dataset_path_root_output=$3
        print_folder_recurse $path_dataset_input $folder_to_move $dataset_path_output

    else 
        echo "Not valid directory"
    fi
fi
