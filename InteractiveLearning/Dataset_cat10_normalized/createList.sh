#!/bin/bash

search_dir=.

rm list.txt
for category in "$search_dir"/*/
do
	for type in "$category"*/
	do
		for entry in "$type"*.ply
		do echo "$entry" >> list.txt
	  	done
	done
done