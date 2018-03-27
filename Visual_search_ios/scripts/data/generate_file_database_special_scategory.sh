#!/bin/bash

#Loop over categories
let "total_files = 0"
let "total_categories = 0"
let "dataset_path_root"
let "numberTraining = 0"
let "numberTesting = 0"
let "count = 0"
file=dataset_categories.txt

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
            #the path is somthing like database/categorie_i/object_i. We want to remove the prefixe database in order to keep only the second part, i.e categorie_i/object_i
            categorie=${path_dir_string#$dataset_path_root/}
            #Then we keep only the categorie of the object by taking the first word i.e categorie_i
            if ! [[ $categorie == *"/"* ]]
            then
                echo "$categorie" >> $file
                echo "$categorie" >> dataset_testing.txt
                echo "$categorie" >> dataset_training.txt
                echo "Category: $categorie"
                total_categories=$((total_categories + 1))
            fi
            print_folder_recurse "$i" $2
        #Then for each folder, check if the file insite are file
        elif [ -f "$i" ]; then
            #Depend of what kind of data we want to search. PCD file or PLY file
            if [ ! -z $2 ]; then
                extension=$2
            else
                #Per defaut
                extension="ply"
            fi
             if [[ $i == *.$extension ]]; then
                path_string=$i
                file_string=${path_string#$dataset_path_root/}
              if  contains "$categorie" "camera"; then
                echo "$file_string" >> $file
                echo "Found : $file_string"
               fi
                total_files=$((total_files + 1))
                #For testing, take 1/3
                if [ $(($count%3)) -eq 0 ]; then
                    numberTesting=$((numberTesting + 1))
                    echo "$file_string" >> dataset_testing.txt
                else
                    numberTraining=$((numberTraining + 1))
                    echo "$file_string" >> dataset_training.txt

                fi

             fi
        fi
        count=$((count+1))
    done
}

#### MAIN ######
printf "====> Create a txt file which contains the path to every model in the dataset. The dataset must follow this structure : \nDataset/Categorie_1/Object_1 , Dataset/Categorie_1/Object_i , Dataset/Categorie_i/Object_i <==== \n ./script dataset_path extension (an option). Extension is either ply or pcd. Per defaut it's pcd \n"

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
    printf "\n-->Please precise the database path \n"
    printf "$0 dataset_path \n"
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

        #If the parameter is present
        extension=$2
        print_folder_recurse $path $extension



    echo "Number Training file: ${numberTraining}"
    echo "Number Testing file: ${numberTesting}"
    echo "Total file: ${total_files}"
    echo "Total Categories: ${total_categories}"
    fi
fi
