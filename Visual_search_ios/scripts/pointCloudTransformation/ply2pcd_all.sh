#!/bin/bash

#
#Take a dataset as an input and for every ply point cloud found convert it to pcd \n"
#

let "total_file_process = 0"

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
        if [ -d "$i" ];then
#echo "dir: $i"
            print_folder_recurse "$i"
        elif [ -f "$i" ]; then
            if contains "$i" ".ply" &&  ! contains "$i" "mtl" ]; then

                filename_ext=$(basename "$i")
                extension="${filename##*.}"
                filename="${filename_ext%.*}"
                newFilePCD=$filename".pcd"
                path=${i%$filename_ext}
                newFilePCD=$path$newFilePCD;
                format="format -1"
                #echo "$i" | sed -e "s/$filename_ext$//"
                echo "path : $newFilePCD"
                ./pcl_ply2pcd $format $i $newFilePCD
                total_file_process=$((total_file_process + 1))
            fi
        fi
    done
}

#### MAIN ######
printf "====> From a dataset, convert all ply file to pcd file"

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
        printf "\n-->Please precise the path to the datasets which contains the cloud \n"
        printf "$0 dataset \n"
else
    if [ -d "$1" ]; then
    dataset_path=$1
    print_folder_recurse $dataset_path

    echo "Total file process: ${total_file_process}"
    fi

fi


