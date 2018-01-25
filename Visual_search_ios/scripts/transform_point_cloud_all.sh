#!/bin/bash

#
#Take a dataset as an input and for every point cloud inside it (pcd,obj,ply) do a translation/rotation according what the user chose
# $0 dataset type_cloud(pcd, obj, ply) tx ty tz rx ry rz\n"
#

let "tx"
let "ty"
let "tz"
let "rx"
let "ry"
let "rz"
let "type_descriptor"
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
            if contains "$i" ".$type_descriptor"  &&  ! contains "$i" "mtl" ]; then
                echo "file: $i"
                query="-query "$i
                output="-output "$i
                ./transform_point_cloud_main $query $output $tx $ty $tz $rx $ry $rz
                total_file_process=$((total_file_process + 1))
            fi
        fi
    done
}

#### MAIN ######
printf "====> From a dataset, make a rotation and/or translation of all points cloud inside this dataset. Extension accepted : pcd, ply, obj"

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

if [ "$#" -lt 8 ];
    then
        printf "\n-->Please precise the path to the datasets which contains the cloud and the modification you want to do  \n"
        printf "$0 dataset type_cloud(pcd, obj, ply) tx ty tz rx ry rz\n"
else
    if [ -d "$1" ]; then
    dataset_path=$1
    type_descriptor=$2
    tx=$3
    ty=$4
    tz=$5
    rx=$6
    ry=$7
    rz=$8
    print_folder_recurse $dataset_path

    echo "Total file process: ${total_file_process}"
    fi

fi


