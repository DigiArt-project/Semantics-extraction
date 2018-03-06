import sys
import os
import argparse
import numpy as np
from operator import itemgetter
from collections import Counter

nb_to_process = 19816
#nb_to_process = 10

nb_class = 10
max_k = 20

def load_testfile(filepath, associate_to_label = True):
    test_file_category = list()
    with open(filepath) as f:
        content = f.readlines()
        count = 0
        for line in content:
            line = line.strip()
            category = line.split("/")[1]
            if associate_to_label:
                category_label = 0
                if (category == "bathtub"):
                    category_label = 0
                elif (category == "bed"):
                    category_label = 1
                elif (category == "chair"):
                    category_label = 2
                elif (category == "desk"):
                    category_label = 3
                elif (category == "dresser"):
                    category_label = 4
                elif (category == "monitor"):
                    category_label = 5
                elif (category == "night_stand"):
                    category_label = 6
                elif (category == "sofa"):
                    category_label = 7
                elif (category == "table"):
                    category_label = 8
                elif (category == "toilet"):
                    category_label = 9
                test_file_category.append(category_label)
            else :
                test_file_category.append(category)

    return test_file_category

def count_objects_per_category(list_objects_category, number_category):
    #print(list_objects_category[number_category])
    list_count = Counter(elem[2] for elem in list_objects_category[number_category])

    print(list_count)

def load_category(filepath):
    categories_label = list()
    with open(filepath) as f:
        content = f.readlines()
        count = 0
        for line in content:
            line = line.strip()
            tuple_content = (line.split("/")[1], count)
            categories_label.append(tuple_content)
            

            count = count + 1

    return categories_label


def query_category_score(lists_all_categories_score,category_number):
    nb_category_to_nbtab = category_number
    current_category_scores = lists_all_categories_score[nb_category_to_nbtab]
    #print("Category {}, list = {}".format(category_number,current_category_scores))
    sorted_by_top_results = sorted(current_category_scores, key=lambda tup: tup[0], reverse=True)

    result_precision_pourcent = compute_precision_k(sorted_by_top_results,category_number,max_k)
    print("{} %".format(result_precision_pourcent))

    return result_precision_pourcent



def compute_precision_k(sorted_top_results,category_number,max_k):
    rel_k = 0
    max_k_to_nbarray = max_k - 1
    list_count = Counter(elem[1] for elem in sorted_top_results[:max_k])
    number_items_from_category = list_count[category_number]
    
    #print('Number results in top {} for category {} = {}'.format(max_k,category_number,number_items_from_category))
    #print(sorted_top_results)
    #print(sorted_top_results[max_k_to_nbarray])

    result_label = sorted_top_results[max_k_to_nbarray][1]
    #print("Last rank --> Result label = {} | True label = {}".format(result_label,category_number) )
    if not result_label == category_number:
        rel_k = 0
    else :
        rel_k = 1

    #print("Rek k({}) = {}".format(max_k,rel_k))
    result_precision = number_items_from_category / max_k
    result_precision_pourcent = result_precision * 100

    return result_precision,result_precision_pourcent, rel_k


def main(file,test_file):
    count = 0
    test_file_category = load_testfile(test_file,True)
    test_file_category = test_file_category[:nb_to_process]

    #List which contain the labels
    list_label = list()
    #List which contains lists of each category
    lists_all_categories_score = []
    n = nb_class
    for i in range(n):
        lists_all_categories_score.append([])
    #print(lists)
    count = 0
    with open(file) as f:
        content = f.readlines()
        for line in content[:nb_to_process]:
            line = line.strip()
            scores = line.split()   
            #print(scores)
            
            list_label.append(float(scores[nb_class]))
            #Fill each list 
            for idx,current_score in enumerate(scores):

                if not idx == nb_class:
                    #tuple_score_label = list(zip(lists_all_categories_score[0],list_label))
                    tuple_score_label = (float(current_score),list_label[count],test_file_category[count])
                    #lists_all_categories_score[idx].append(float(current_score))
                    lists_all_categories_score[idx].append(tuple_score_label)

            
            
            #print(count)
            count = count + 1

    #Query part
    print("\n")
    #count_objects_per_category(lists_all_categories_score, 2)
    #exit()
    nb_category = 6
    current_category_scores = lists_all_categories_score[nb_category]
    #print("Category {}, list = {}".format(category_number,current_category_scores))
    sorted_by_top_results = sorted(current_category_scores, key=lambda tup: tup[0], reverse=True)
    #print(sorted_by_top_results)


    #For all categories
    MeanAveragePrecision = 0
    max_k = 1900
    for nb_cat in range(0,10):
        current_category_scores = lists_all_categories_score[nb_cat]
        sorted_by_top_results = sorted(current_category_scores, key=lambda tup: tup[0], reverse=True)
        AveragePrecision = 0
        number_relevant_results = 0
        current_k = 0
        for i in range(max_k):
            current_k = i + 1
            precision,result_precision_pourcent,rel_k = compute_precision_k(sorted_by_top_results,nb_cat,current_k)
            #print("P({}) = {} | Rel({}) = {}".format(current_k,precision,current_k,rel_k))
            if rel_k == 1:
                number_relevant_results = number_relevant_results + 1
            AveragePrecision = AveragePrecision + (precision * rel_k)
            #print("Average Precision not divided : {}".format(AveragePrecision))
        if not number_relevant_results == 0:
            AveragePrecision = AveragePrecision/number_relevant_results   
        else :
            AveragePrecision = 0
        
        MeanAveragePrecision = MeanAveragePrecision + AveragePrecision
        print("\nNumber relevant document: {}".format(number_relevant_results))
        print("\nAverage Precision  = {} %".format(AveragePrecision*100))
        

    MeanAveragePrecision = MeanAveragePrecision/nb_class

    print("\nMean Average Precision  = {} %".format(MeanAveragePrecision*100))
    
    

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-file", "--file", required=True, help="File which contains the scores")
    ap.add_argument("-test", "--test", required=True, help="File which contains the test set")
    args = vars(ap.parse_args())

    file = args["file"]
    test_file = args["test"]
    main(file,test_file)

