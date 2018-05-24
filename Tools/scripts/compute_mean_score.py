import sys
import os
import argparse
import numpy as np

## Given a txt file wich contain an array of score, compute the mean score

def main(file):
    print("Main")
    count = 0
    with open(file) as f:
        content = f.readlines()
        for line in content:
            line = line.strip()
            if count > 0:
                scores = line.split()     
                size = len(scores)
                total = 0
                for score in scores:
                    total = total + float(score)
                average = total/size
                print("Average Score : {}".format(average))

            count = count + 1
            #print("\n")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-file", "--file", required=True, help="File which contains the scores")
    args = vars(ap.parse_args())

    file = args["file"]
    main(file)