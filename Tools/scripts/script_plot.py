import pylab
import matplotlib.pyplot as plt
import csv
import os
from os.path import basename
import numpy as np

x = []
y = []

dataset = "./"
number_limit = 5
'''
filename = "desc_box_9_0.txt"

for line in open(filename, 'r'):
  values = [float(s) for s in line.split()]


plt.plot(values)
plt.show()
'''

count = 0 
fig = plt.figure(figsize=(15, 8))
for root1, directories1, filenames1 in os.walk(dataset):
    for filename1 in filenames1:
        file = os.path.join(root1, filename1)
        if count < number_limit:
            filename = file
            filename_basename = basename(file)
            filename_basename = os.path.splitext(filename_basename)[0]
            print(filename_basename)
            for line in open(filename, 'r'):
                values = [float(s) for s in line.split()]

            plt.plot(values, lw = 1, c=np.random.random(3), label=filename_basename)


        count = count + 1
plt.legend(loc="lower left")
plt.show()
