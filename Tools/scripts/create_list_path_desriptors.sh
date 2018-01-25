#!/bin/bash

let "total_descriptor = 0"
let "total_categories = 0"
let "dataset_path_root"
let "type_descriptor"
let "path_to_save"
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
                if [ ! "$categorie" = "training_esf" ]  && [ ! "$categorie" = "training_gshot" ] && [ ! "$categorie" = "training_gshotPyramid" ] &&[ ! "$categorie" = "training_vfh" ]  && [ ! "$categorie" = "training_grsd" ]  && [ ! "$categorie" = "training_cvfh" ] && [ ! "$categorie" = "training_ourcvfh" ]; then
                    echo "$categorie" >> $path_to_save
                    #echo "$categorie" >> descriptor_testing.txt
                    #echo "$categorie" >> descriptor_training.txt
                    echo "Category: $categorie"
                    total_categories=$((total_categories + 1))

                fi
            fi
            print_folder_recurse "$i"
        #Then for each folder, check if the file insite are file
        elif [ -f "$i" ]; then
            if contains "$i" "/$type_descriptor/" && contains "$i" "/$compute_on_full/"; then

                    if contains "$i" ".pcd" || contains "$i" ".txt"; then
                        echo "$i"
                        path_string=$i
                        #echo $PWD/"$path_string" >> $path_to_save
                        echo $PWD/"$path_string" >> $path_to_save
                        echo "Found : $path_string"
                        total_descriptor=$((total_descriptor + 1))
                    fi
            fi

        fi
        count=$((count+1))
    done
}

#### MAIN ######
printf "====> Create a txt file which contains the path to every descriptors. \n The results dataset must follow this structure : \n Dataset/Categorie_i/type_descriptor with type_descriptor = [ vfh, esf, CVFH, OURCVFH, GRSD] <==== \n ./script dataset_path extension (optionel parameter). Extension needed for descriptors need to be pcd  \n \n Example : ./create_list_path_descriptors.sh folder_descriptors grsd"

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
    printf "\n==> Please precise the path to the results which contains the descriptors and the descriptor wanted \n"
    printf "$0 dataset_path esf/vfh/cvvfh/ourcvfh/grsd/gshot/gshotPyramid/spin/good/scurv/sc3D/egigshotfull/esffull/grsdfull/pointnet/pointnet_modelnet/alldesc[i] full/views\n \n"
    printf "all means that you want to concatenate some descriptors \n"
else
    #check if the file already exist. If yes, delete it
    if [ -f $file ] ; then
        rm $file
    fi

    # try get path from param
    path=""
    if [ -d "$1" ]; then
        path=$1;
        if contains "$path" "../"; then
            echo "$path" | cut -c 4-
        fi
        dataset_path_root=$1
        type_descriptor=$2
        compute_on_full=$3

        file="dataset_descriptor_"
        #if contains "$type_descriptor" "alldesc"; then
        #    if [ "$type_descriptor" == "alldesc0" ]; then
        #        name_concatenation="esfvfh"
        #    elif [ "$type_descriptor" == "alldesc1" ]; then
        #       name_concatenation="esfgshot"
        #    elif [ "$type_descriptor" == "alldesc2" ]; then
        #        name_concatenation="esfgrsd"
        #    elif [ "$type_descriptor" == "alldesc3" ]; then
        #        name_concatenation="esfgrsdgshot"
        #    elif [ "$type_descriptor" == "alldesc4" ]; then
        #        name_concatenation="esfgshotgrsdvfh"
        #    elif [ "$type_descriptor" == "alldesc5" ]; then
        #        name_concatenation="grsdgshot"
        #    elif [ "$type_descriptor" == "alldesc6" ]; then
        #        name_concatenation="esfvfhgshot"
        #    fi

           
        #else 
        #    name_concatenation=$type_descriptor

        #fi
        #file+=$name_concatenation".txt";
        name_concatenation=$type_descriptor
        file+=$name_concatenation".txt";

        path_to_save=$file
        print_folder_recurse $path $type_descriptor $compute_on_full


    echo "Total descriptor: ${total_descriptor}"
    fi
fi
