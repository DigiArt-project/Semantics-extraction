#!/usr/bin/env python3
import shutil
import os
import sys

#Usage : python3 move_pcd.py structure_sensor_dataset/ Model.pcd

dr = sys.argv[1]
file_to_search = sys.argv[2]

for root, dirs, files in os.walk(dr):
    for file in files:
        if file == file_to_search:
            spl = root.split("/"); newname = spl[-1]; sup = ("/").join(spl[:-1])
            shutil.move(root+"/"+file, sup+"/"+newname+".pcd"); shutil.rmtree(root)