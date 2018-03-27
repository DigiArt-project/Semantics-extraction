#!/bin/bash

#
#Take a dataset as an input and for every pcd point cloud convert to ply\n"
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
            if contains "$i" ".pcd" &&  ! contains "$i" "mtl" ]; then

                filename_ext=$(basename "$i")
                extension="${filename##*.}"
                filename="${filename_ext%.*}"
                newFilePLY=$filename".ply"
                path=${i%$filename_ext}
                newFilePLY=$path$newFilePLY;
                format="format -1"
                #echo "$i" | sed -e "s/$filename_ext$//"
                echo "path : $newFilePLY"
                ./pcl_pcd2ply $format $i $newFilePLY
                total_file_process=$((total_file_process + 1))
            fi
        fi
    done
}

#### MAIN ######
printf "====> From a dataset, convert all pcd file to ply file"

# Display name of the script
echo "Name's script is : $0"
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


