#!/bin/bash

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




rm train_views.txt
rm train_full.txt
while read line
do
	if contains "$line" "view" ]; then
		echo $line >> train_views.txt
	else 
		echo $line >> train_full.txt
	fi
done < train.txt

rm test_views.txt
rm test_full.txt
while read line
do
	if contains "$line" "view" ]; then
		echo $line >> test_views.txt
	else 
		echo $line >> test_full.txt
	fi
done < test.txt
