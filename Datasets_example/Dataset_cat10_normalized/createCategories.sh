#!/bin/bash

search_dir=.
rm categories.txt
for category in "$search_dir"/*/
do echo "$category" >> categories.txt
done