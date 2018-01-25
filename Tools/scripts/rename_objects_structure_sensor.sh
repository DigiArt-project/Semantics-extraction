#!/bin/bash

let "type_descriptor"
let "dataset_path_root"
let "name_category"
let "count = 0"
let "count_category = 0"
let "cloud_extension"

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
        #echo "$i"
        if [ -d "$i" ];then
            path_dir_string=$i
            #the path is something like database/categorie_i/type_Descriptor. We want to remove the prefixe database in order to keep only the second part, i.e categorie_i/type_descriptor
            categorie=${path_dir_string#$dataset_path_root/}
            name_category=$categorie
            #Then we keep only the categorie of the object by taking the first word i.e categorie_i
            print_folder_recurse "$i"
            count_category=$((count_category+1))
        #Then for each folder, check if the file insite are file
        elif [ -f "$i" ]; then
                    if contains "$i" "."$cloud_extension ; then
                        
                        extension_filename="${i##*.}"
                        filename="${i%.*}"
                        parent_dir="$(dirname "$i")"
                        grand_parent_dir="$(dirname "$parent_dir")"
                        filename_parent_dir="$(basename $parent_dir)"
                        echo "parent_dir : $parent_dir"
                        echo "filename : $filename"
                        value=$(cut -d/ -f2 <<<"${i}")
                        

                        new_name_object="$filename_parent_dir.$extension_filename"
                        echo "$new_name_object"
                        echo "Path dir string : $path_dir_string"

                        #Rename
                        mv "$i" "${path_dir_string#}/$new_name_object"
                        #Move
                        mv "${path_dir_string#}/$new_name_object" "$grand_parent_dir"
                        #Delete folder
                        rm -rf "$parent_dir"

                        path_string=$i
                    fi

        fi
        count=$((count+1))
    done
}

#### MAIN ######
printf "====> Rename files inside folder"

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

if [ "$#" -lt 2 ];
then
    printf "\n-->This script allows to organize better the structure sensor dataset. You assume that the dataset follows this organization : dataset/object/object_1/model.ply "
    printf "\n The programs change to this structure : dataset/object/object_1.ply "
    printf "\n \n-->Please precise the path to the structure sensor dataset where you want to rename files \n"
    printf "$0 dataset_path extension\n"
else

    # try get path from param
    path=""
    if [ -d "$1" ]; then
        path=$1;
        cloud_extension=$2
        dataset_path_root=$1
        print_folder_recurse $path

    fi
fi
