#!/bin/bash


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


#### #. Generate category file
echo "[1/4] Create category file"
search_dir=.
if [ -f categories.txt ]; then
    rm categories.txt
fi

for category in "$search_dir"/*/
	do
	#Remove the last slash
	category=${category%/}
	#Remove the first slash
	category=${category#./}
	if ! contains "$category" "training_"; then
		echo "$category"
		echo "$category" >> categories.txt
	fi

done

#### 2. Generate list of objects
echo "[2/4] Generate list of objects from categories"
search_dir=.
if [ -f list.txt ]; then
    rm list.txt
fi
for category in "$search_dir"/*/
do
	for type in "$category"*/
	do
		for entry in "$type"*.ply
		do echo "$entry" >> list.txt
	  	done
	done
done


#### 3. Create train and test file
echo "[3/4] Create train and test file"
if [ -f train.txt ]; then
    rm train.txt
fi
if [ -f test.txt ]; then
    rm test.txt
fi
numberLines=$(cat list.txt | wc -l)
cat list.txt | gshuf -n ${numberLines} --output tmp.txt
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

#### 4. Split full and views
echo "[4/4] Split full and partial views"
if [ -f train_views.txt ]; then
    rm train_views.txt
fi
if [ -f train_full.txt ]; then
    rm train_full.txt
fi
while read line
do
	if contains "$line" "view" ]; then
		echo $line >> train_views.txt
	else 
		echo $line >> train_full.txt
	fi
done < train.txt

if [ -f test_views.txt ]; then
    rm test_views.txt
fi
if [ -f test_views.txt ]; then
    rm test_views.txt
fi
while read line
do
	if contains "$line" "view" ]; then
		echo $line >> test_views.txt
	else 
		echo $line >> test_full.txt
	fi
done < test.txt

echo "Process done."




