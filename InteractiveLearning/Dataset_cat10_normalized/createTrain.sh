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



rm train.txt
rm test.txt
numberLines=$(cat list.txt | wc -l)
cat list.txt | shuf -n ${numberLines} --output tmp.txt
xFull=0
xViews=0
while read line
do
	if contains "$line" "view" ]; then
		if [ `echo "$xViews" | bc` -lt 6 ]; then 
			echo $line >> train.txt
		else 
			echo $line >> test.txt
		fi
		xViews=`echo "$(($xViews+1)) % 10" | bc`
	else
		if [ `echo "$xFull" | bc` -lt 6 ]; then 
			echo $line >> train.txt
		else 
			echo $line >> test.txt
		fi
		xFull=`echo "$(($xFull+1)) % 10" | bc`
	fi
done < tmp.txt

rm tmp.txt
