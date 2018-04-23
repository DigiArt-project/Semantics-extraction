#!/usr/bin/env python3 -W ignore::DeprecationWarning

####### IMPORT MODULES #######

import os
import time
import warnings
import argparse
import math

import copy
import itertools
import numpy as np
import logging

#Interactive learning
from base.dataset import Dataset
from models import *
from query_strategies import *
from labelers import IdealLabeler

#Skelearn
import sklearn as skl
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import precision_score,f1_score,recall_score,precision_recall_curve,average_precision_score,classification_report,confusion_matrix
from sklearn import svm, datasets


#Matplotlib
import matplotlib.pyplot as plt
from matplotlib.image import AxesImage
import matplotlib.gridspec as gridspec
import matplotlib.ticker as plticker
#Activate on the server or when using multi thread
#matplotlib.use('Agg')
import utils

####### GLOBAL VARIABLE #######
PATH_DATASET = "/Users/lironesamoun/digiArt/Datasets/Dataset_structuresensor_normalized/"
TRAINING_FILE_FULL = PATH_DATASET + "train_full.txt"
TESTING_FILE_FULL =  PATH_DATASET + "test_full.txt"
TRAINING_FILE_VIEWS =  PATH_DATASET +"train_views.txt"
TESTING_FILE_VIEWS =  PATH_DATASET +"test_views.txt"
#Perform interative learning on full or views object
COMPUTE_FULL = "false"
#Array of descriptors we want to test
DESCRIPTORS = ['esf','vfh']
#Location of the dataset directory where the txt file are located. The txt represents the list of all the descriptors and path to descriptors
ROOT_DATA = "data/"
NAME_DATASET = 'structuresensor_views'
#Where to save the graphics
OUTPUT_GRAPHICALS_RESULTS = "results_graphics/"
OUTPUT_JSON_RESULT_SIMILARITYSEARCH = "data/results_similarity.json"
#Log file
LOG_FILE = "interactive_learning.log"

#Category label to change for option similarity search interactive learning
CATEGORY_LABEL = 3
#Positive example ranking for display i.e the number of samples ranked from positive to negative, to display to the user 
NBRE_DATA_SELECTION = 20 #defaut 48
#Number of uncertain examples close to the margin to choose 
NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY = 30
#Among the uncertain choosen, this is the number of uncertains to display to the users
NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY_DISPLAY = 7
#Number of example the most uncertain display to the user for selection after diversity
NBRE_DATA_FINAL_SELECTION = 7
# the percentage of samples in the dataset that will be randomly selected and assigned to the test set
TEST_SIZE = 0.35
#SVM parameters
C = 100 #100
GAM = 0.0078125 #0.001
KERNEL_SVM = "rbf" #rbf, linear, poly, sigmoid
DIVERSIFICATION_LAMBDA = 0.5

#Defaut number of k neighbors when using similarity search
K = 20
#Leaf resolution
LEAF_RESOLUTION = 0.01
#Defaut descriptor to use for Interactive Leaning
DESCRIPTOR_INTERACTIVELEARNING = "esf"
#Defaut descriptor to use for Similarity search
DESCRIPTOR_SIMILARITYSEARCH = "esf"
#Number samples to select at the first step (if number = 6, will take the first three and the last three)
NUMBER_TO_ANNOTATE_FIRST_STEP = 6
#Number uncertain samples to select at the second step (if = 5, will take the first 5 uncertains)
NUMBER_TO_ANNOTATE_SECOND_STEP = 5
#Number iterations max of interactive learning step i.e, number of steps the user want to select uncertain sample
NB_ITERATIONS_MAX = 20
#Number of experiment to run of each interactive learning session. 1 is okay, more will take more time
NB_REPETITION_EXPERIMENT = 1
#Number of top result for pourcentage computation
NB_MAX_POSITIF_DISPLAY = 10
#The number of objects we select randomly for each category inside the dataset
NUMBER_OF_OBJECTS_TO_SELECT_RANDOMLY = 3
#Either select full objects or views object for selecting randomly
SELECT_FULL_OBJECT = False

QUERY_CLOUD = ""
OBJECTSLIST_PATH_FILE = ""
DESCRIPTORSLIST_PATH_FILE = ""

_ITEM_CLICK_CALLBACK= set()
_ITEM_ALL_LABEL_DATA = list()

USE_NORMALIZATION = True
PLOT_SCORE = True
PLOT_CNF_MATRIX = False
PLOT_CLF_REPORT = False
PLOT_PR_CURVE = False

"""
Handle the on click event, when the user select example on the screen
"""
def onpick(event):
    if isinstance(event.artist, AxesImage):
        artist = event.artist
        im = artist
        A = im.get_array()
    else :
        id_extract = [int(s) for s in  event.artist.get_text().split() if s.isdigit()]
        print('[INFO] ID selected:', id_extract[0])
        logging.info('[INFO] ID selected: %s  ', id_extract[0])
        _ITEM_CLICK_CALLBACK.add(id_extract[0])


"""
Display the first data to anotate, for the similarity search interactive learning
"""
def display_data_init_similarity_search_classic(list_idx, fully_labeled_train_dataset, display_color = True):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1,figsize=(11, 10))
    fig.canvas.set_window_title('Results')
    # gridspec inside gridspec
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    outer_grid = gridspec.GridSpec(8, 6, wspace=1, hspace=0.8)
    count = 0
    for idx,id_value in enumerate(list_idx):
        lb = idealLabels.label(X[id_value])
        ax = plt.Subplot(fig, outer_grid[count])    
        #filename_test = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],1)
        #print(filename_test)
        #exit()
        filename = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],10)
        ax.text(0.1, 0.5, str(filename),size=7)
        title = "ID {}".format(id_value)
        if display_color:
            if (lb == 1):
                ax.set_title(title,style='italic', picker=True,  bbox={'facecolor':'limegreen', 'pad':3}, size = 13)
            else :
                ax.set_title(title,style='italic', picker=True, bbox={'facecolor':'salmon', 'pad':3}, size = 13)
        else :
            ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)

        count = count + 1

    fig.canvas.mpl_connect('pick_event', onpick)
    
    plt.show(block=False)



"""
After the first step, display data :
- The most uncertaine one i.e close to the margins
- The positif one i.e the most certains one corresponding to the positif label
"""
def display_selected_data(dataset, objectslist_path,id_to_display,id_close_margins):
    plt.close(plt.figure(1))
    plt.close(plt.figure(3))
    fig = plt.figure(1,figsize=(9, 7))
    fig1 = plt.figure(3,figsize=(9, 7))
    fig.canvas.set_window_title('Results')
    fig1.canvas.set_window_title('Most uncertain + diversity example')
    labeled_entry_ids, X_pool_labeled = zip(*dataset.get_labeled_entries())
    
    outer_grid = gridspec.GridSpec(5, 6, wspace=1, hspace=0.5)
    outer_grid1 = gridspec.GridSpec(5, 6, wspace=1, hspace=0.5)
    #The most certain exemples
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        filename = utils.get_filename_descriptor_from_path(objectslist_path[ask_id],10)

        ax = plt.Subplot(fig, outer_grid[i])
        ax.text(0.1, 0.5, str(filename),size=7)
        title = "ID {}".format(ask_id)
        ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)

    #The most uncertains one
    for i in range(len(id_close_margins)):
        ask_id = id_close_margins[i]
        if not ask_id in np.array(labeled_entry_ids):
            filename = utils.get_filename_descriptor_from_path(objectslist_path[ask_id],10)
            ax = plt.Subplot(fig1, outer_grid1[i])
    
            ax.text(0.1, 0.5, str(filename),size=7)
            title = "ID {}".format(ask_id)
            ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
            ax.set_xticks([])
            ax.set_yticks([])
            fig1.add_subplot(ax)

    
    fig.canvas.mpl_connect('pick_event', onpick)
    fig1.canvas.mpl_connect('pick_event', onpick)
    plt.show(block=False)



"""
In this version, we have the ground truth available meaning that we display in green the positive label and in red the negative label
After the first step, display data :
- The most uncertaine one i.e close to the margins
- The positif one i.e the most certains one corresponding to the positif label
"""
def display_selected_data_similarity_search_withGT(fully_labeled_train_dataset,train_dataset, path_list_train_views,id_to_display,id_close_margins,display_color = True,save_figures = True):
    plt.close(plt.figure(1))
    plt.close(plt.figure(3))

    fig = plt.figure(1,figsize=(9, 7))
    fig1 = plt.figure(3,figsize=(9, 7))
    fig.canvas.set_window_title('Results')
    fig1.canvas.set_window_title('Most uncertain + diversity example')
    labeled_entry_ids, X_pool_labeled = zip(*train_dataset.get_labeled_entries())
    X, y = zip(*train_dataset.data)
    nb_total_positif_annotated = y.count(1)
    nb_total_negatif_annotated = y.count(-1)

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)

    count_positif = 0
    count_negatif = 0
    
    outer_grid = gridspec.GridSpec(5, 6, wspace = 1, hspace=1)
    outer_grid1 = gridspec.GridSpec(5, 6, wspace=1, hspace=1)
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        ax = plt.Subplot(fig, outer_grid[i])

        filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],10)

        lb = idealLabels.label(X[ask_id])

        ax.text(0.1, 0.4, str(filename),size=10)
        title = "ID {}".format(ask_id)
        if display_color:
            if (lb == 1):
                ax.set_title(title,style='italic', picker=True,  bbox={'facecolor':'limegreen', 'pad':3}, size = 13)
            else :
                ax.set_title(title,style='italic', picker=True, bbox={'facecolor':'salmon', 'pad':3}, size = 13)
        else :
            if (lb == 1):
                count_positif = count_positif + 1
            else : 
                count_negatif = count_negatif + 1
            ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)
    title = "Positives samples where the distance is far from the boundary "
    fig.suptitle(title, fontsize=8, fontweight='bold')


    for i in range(len(id_close_margins)):
        ask_id = id_close_margins[i]
        if not ask_id in np.array(labeled_entry_ids):
            ax = plt.Subplot(fig1, outer_grid1[i])
       
            filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],10)

            lb = idealLabels.label(X[ask_id])
            ax.text(0.1, 0.4, str(filename),
                            size=8)
            title = "ID {}".format(ask_id)
            if display_color:
                if (lb == 1):
                    ax.set_title(title,style='italic', picker=True,  bbox={'facecolor':'limegreen', 'pad':3}, size = 13)
                else :
                    ax.set_title(title,style='italic', picker=True, bbox={'facecolor':'salmon', 'pad':3}, size = 13)
            else :
                ax.set_title(title, picker=True, bbox=dict(facecolor='blue'), size = 13)
            ax.set_xticks([])
            ax.set_yticks([])
            fig1.add_subplot(ax)
    title = "Samples where the distance is close to the decision boundary (most uncertain one + diversity) "
    fig1.suptitle(title, fontsize=8, fontweight='bold')
    
    fig.canvas.mpl_connect('pick_event', onpick)

    if (save_figures):
        fig.savefig('results.png', bbox_inches='tight')

    pourcent_positif_display = (count_positif / NBRE_DATA_SELECTION) * 100
    pourcent_negatif_display = (count_negatif / NBRE_DATA_SELECTION) * 100

    fig1.canvas.mpl_connect('pick_event', onpick)
    
    plt.show(block=False)

    return pourcent_positif_display, pourcent_negatif_display



"""
Ask user to select positive label
Return list of ID selected
"""
def select_positive_example_onclick(id_unlabeled_selected):
    banner = "Clicks ID to label as POSITIVE, then when you finish, write ok \n"
    str_input = input(banner)
    while (str_input != "ok"):
        str_input = input(banner)
        print("If you have finished, write ok")

    positif_label = list()
    for i in range(len(_item_click_callback)):
        positif_label.append(1)
    
    positive_example = list(zip(_item_click_callback, positif_label))
    #Remove items selected in the list
    _item_click_callback.clear()

    return positive_example

"""
Ask user to select negative label
Return list of ID selected
"""
def select_negative_example_onclick(id_unlabeled_selected):
    banner = "Clicks ID to label as NEGATIVE then when you finish, write ok \n"
    str_input = input(banner)
    while (str_input != "ok"):
        str_input = input(banner)
        print("If you have finished, write ok")

    negatif_label = list()
    for i in range(len(_item_click_callback)):
        negatif_label.append(-1)
    
    negative_example = list(zip(_item_click_callback, negatif_label))
    #Remove items selected in the list
    _item_click_callback.clear()
    
    return negative_example


"""
Ask user to select positive label
Return list of ID selected
"""
def select_positive_example_onclick(id_unlabeled_selected):
    banner = "Clicks ID to label as POSITIVE, then when you finish, write ok \n"
    str_input = input(banner)
    while (str_input != "ok"):
        str_input = input(banner)
        print("If you have finished, write ok")

    positif_label = list()
    for i in range(len(_ITEM_CLICK_CALLBACK)):
        positif_label.append(1)
    
    positive_example = list(zip(_ITEM_CLICK_CALLBACK, positif_label))
    #Remove items selected in the list
    _ITEM_CLICK_CALLBACK.clear()

    return positive_example

"""
Ask user to select negative label
Return list of ID selected
"""
def select_negative_example_onclick(id_unlabeled_selected):
    banner = "Clicks ID to label as NEGATIVE then when you finish, write ok \n"
    str_input = input(banner)
    while (str_input != "ok"):
        str_input = input(banner)
        print("If you have finished, write ok")

    negatif_label = list()
    for i in range(len(_ITEM_CLICK_CALLBACK)):
        negatif_label.append(-1)
    
    negative_example = list(zip(_ITEM_CLICK_CALLBACK, negatif_label))
    #Remove items selected in the list
    _ITEM_CLICK_CALLBACK.clear()
    
    return negative_example


"""
    Ask user to annotate positive and negative label
    Return list of positive and negatives label
    """
def annotate_data(whole_dataset, id_to_display):
    id_selected_positif = select_positive_example_onclick(id_to_display)
    id_selected_negatif = select_negative_example_onclick(id_to_display)
    
    list1 = list()
    list2 = list()
    for id_l,label in id_selected_positif:
        logging.debug('ID : %s corresponds to label : %s  ', id_l, label)
        list1.append((id_l,label))
        whole_dataset.update(int(id_l), label)
        _ITEM_ALL_LABEL_DATA.append((id_l,label))
    for id_l,label in id_selected_negatif:
        logging.debug('ID : %s corresponds to label : %s  ', id_l, label)
        list2.append((id_l,label))
        whole_dataset.update(int(id_l), label)
        _ITEM_ALL_LABEL_DATA.append((id_l,label))
    total_annotated_id_list = list1 + list2

    logging.debug('[INFO] ====== All label data %s  ', _ITEM_ALL_LABEL_DATA)
    logging.debug('Update Nbr unlabeled: %s  ', whole_dataset.len_unlabeled())
    logging.debug('Update Nbr labeled: %s  ', whole_dataset.len_labeled())

    return total_annotated_id_list



"""
Ask the user to annotate positive and negative label
Return list of positive and negatives label
"""
def annotate_data_similarity(fully_labeled_train_dataset,whole_dataset_unlabeled, id_to_display):
    id_selected_positif = select_positive_example_onclick(id_to_display)
    id_selected_negatif = select_negative_example_onclick(id_to_display)
    
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, y = zip(*fully_labeled_train_dataset.data)
    
    while (len(id_selected_negatif) == 0):
        print("Select one random negatif")
        id_random = random.randint(0, len(X)-1)
        feature_random = X[id_random]
        lb = idealLabels.label(feature_random)
        if (lb == - 1):
            id_selected_negatif.append((id_random,lb))
            _ITEM_ALL_LABEL_DATA.append((id_random,lb))
    
    while (len(id_selected_positif) == 0):
        print("Select one random positif")
        id_random = random.randint(0, len(X)-1)
        feature_random = X[id_random]
                
        lb = idealLabels.label(feature_random)
        if (lb == 1):
            count_annotate_positif = count_annotate_positif + 1
            id_selected_positif.append((id_random,lb))
            _ITEM_ALL_LABEL_DATA.append((id_random,lb))
    
    list1 = list()
    list2 = list()
    for id_l,label in id_selected_positif:
        logging.debug('ID : %s corresponds to label : %s  ', id_l, label)
        list1.append((id_l,label))
        whole_dataset_unlabeled.update(int(id_l), label)
        _ITEM_ALL_LABEL_DATA.append((id_l,label))
    for id_l,label in id_selected_negatif:
        logging.debug('ID : %s corresponds to label : %s  ', id_l, label)
        list2.append((id_l,label))
        whole_dataset_unlabeled.update(int(id_l), label)
        _ITEM_ALL_LABEL_DATA.append((id_l,label))
    total_annotated_id_list = list1 + list2


    return total_annotated_id_list


"""
Replicate the predict function of scikit :  clf.predict(X)
Give the prediction value of a given point 
"""
def predict(params, sv, nv, a, b, cs, x):
    ''' params = model parameters
        sv = support vectors
        nv = # of support vectors per class
        a  = dual coefficients
        b  = intercepts 
        cs = list of class names
        X  = feature to predict       
    '''
    decision = decision_function(params, sv, nv, a, b, x)
    votes = [(i if decision[p] > 0 else j) for p,(i,j) in enumerate((i,j) 
                                           for i in range(len(cs))
                                           for j in range(i+1,len(cs)))]

    return cs[max(set(votes), key=votes.count)]

"""
Replicate the predict function of scikit :  clf.predict(X)
Give the prediction value of a given vector of points
"""
def predict_vector(params, sv, nv, a, b, cs, x):
    ''' params = model parameters
        sv = support vectors
        nv = # of support vectors per class
        a  = dual coefficients
        b  = intercepts 
        cs = list of class names
        X  = feature to predict       
    '''
    result_prediction= list()
    for idx, value in enumerate(X):
        decision = utils.decision_function_point(params, sv, nv, a, b, value)
        votes = [(i if decision[p] < 0 else j) for p,(i,j) in enumerate((i,j) 
                                           for i in range(len(cs))
                                           for j in range(i+1,len(cs)))]
        result_prediction.append(cs[max(set(votes), key=votes.count)])

    return np.array(result_prediction)


"""
Given a model (only SVC here), return the parameters of the model
"""
def getparameters_from_models(model):
    if isinstance(model,sklearn.svm.classes.SVC):
        # Get parameters from model
        _params = model.get_params()
        #Support vectors.
        _sv = model.support_vectors
        #Number of support vectors for each class.
        _nv = model.n_support_
        #Coefficients of the support vector in the decision function
        _a = model.dual_coef_
        #Constants in decision function.
        _b = model._intercept_
        #List of class name
        _cs = model.classes_
        
        return _params, _sv, _a, _b, _cs

    else :
        logging.error('[ERROR] getparameters_from_models ==> instance is not SVC')


"""
Count the number of positive label and negative label displayed to the user
"""
def compute_pourcentage_results(fully_labeled_train_dataset, train_dataset, id_to_display, number_to_consider = 20):

    if (number_to_consider > len(id_to_display)):
        number_to_consider = len(id_to_display) - 1

    ideal_labels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count_positif = 0
    count_negatif = 0
    count_total = 0
    #Count positve and negative display to the user
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        lb = ideal_labels.label(X[ask_id])
        if (count_total < number_to_consider):
            if (lb == 1):
                count_positif = count_positif + 1
                count_total = count_total + 1
            else :
                count_negatif = count_negatif + 1
                count_total = count_total + 1

    #Count total positif and negatif in the train dataset labeled
    X, y = zip(*train_dataset.data)
    nb_total_positif_annotated = y.count(1)
    nb_total_negatif_annotated = y.count(-1)

    logging.debug("Number positif : %s Number negatif %s | Number to consider %s ", count_positif, count_negatif, number_to_consider)
    if (number_to_consider != 0):
        pourcent_positif_display = (count_positif/number_to_consider) * 100
        pourcent_negatif_display = (count_negatif/number_to_consider) * 100
    else :
        pourcent_positif_display = 0
        pourcent_negatif_display = 0

    return pourcent_positif_display,pourcent_negatif_display


"""
Based on the decision function, select the most uncertain data i.e the closest value to zero. 
So the most uncertains are the one which their distance to the boundary are close to 0
"""
def select_most_uncertains_samples_from_decisionfunction(dataset, model_learning, m, correction_to_apply = 0):
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    features_all = np.array(features_all)
    idx_unlabeled_data, x_pool_unlabeled = zip(*dataset.get_unlabeled_entries())
    decision_function = model_learning.decision_function(features_all)

    decision_function = [x - correction_to_apply for x in decision_function]
    
    indices_rank_decision_function_positif, distance_rank_decision_function_positif = utils.sort_ascending_order(decision_function,True)
    logging.debug("Decision function before take closest 0 {}" .format(decision_function))
    indices_rank_closest_zero = utils.take_closest_to_zero(decision_function)
    logging.debug("indices_rank_closest_zero {}" .format(indices_rank_closest_zero))
    idx_example_positif_selection = np.take(idx_all,indices_rank_closest_zero)[:m]
    idx_example_positif_selection1 = indices_rank_closest_zero[:m]

    
    return idx_example_positif_selection

"""
Based on the probabilities, select the most uncertain data i.e the smallest probabilities.
So the most uncertains are the one which their probabilities are close to 0
"""
def select_most_uncertains_samples_from_probabilities(dataset, probabilities, m, return_score=False):
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    unlabeled_entry_ids, X_pool = zip(*dataset.get_unlabeled_entries())
    dvalue = copy.copy(probabilities)
    score = -np.max(dvalue, axis=1)
    ask_id = np.argmax(score)
    ask_multiple_id = (np.array(score).argsort()[::-1])[:m]
    
    list_result =  np.take(idx_all,ask_multiple_id)
    
    if return_score:
        return list_result, \
                   list(zip(idx_all, score))
    else:
        return list_result

"""
Step 0 Interactive Learning
Feedback
Compute the rank based on positive and negatives samples selected by the user
"""
def feedback(whole_dataset, current_rank, model_learning, annotated_data_selection):
    #Have the score of the selected example
    logging.info('[INFO] Previous Rank : %s  ', current_rank)

    h = 0
    new_rank = 0

    idx_unlabeled_data, x_pool_unlabeled = zip(*whole_dataset.get_unlabeled_entries())
    X_pool_labeled, labels = zip(*whole_dataset.get_labeled_entries())
    decision_function_unlabeled = model_learning.decision_function(x_pool_unlabeled)

    label_all,features_all = zip(*[(idx, entry[0]) for idx, entry in enumerate(whole_dataset.data)])
    features_all = np.array(features_all)

    decision_function_all = model_learning.decision_function(features_all)
    if (USE_NORMALIZATION):
        decision_function_all = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function_all)

    for idx,selection in enumerate(annotated_data_selection):
        logging.debug('=> h = %s  ', h)

        ask_id = int(selection[0])
        label = int(selection[1])

        logging.debug('Label %s for ID %s  ', label, ask_id)

        X = features_all[ask_id]
       
        decision_function_X = decision_function_all[ask_id]

        h_current = label - decision_function_X
        
        h = h + h_current

    new_rank = current_rank + h

    logging.info('[INFO]New rank = %s  ', new_rank)

    return int(round(new_rank))

"""
Step 1 Interactive Learning
Classification
"""
def training_data(training_dataset, model):
    model.fit(*(training_dataset.format_sklearn()))

"""
Step 2 Interactive Learning
Correction
"""
def correction(dataset, model, rank):
    label_all,features_all = zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    features_all = np.array(features_all)

    idx_unlabeled_data, x_pool_unlabeled = zip(*dataset.get_unlabeled_entries())
    decision_function_unlabeled = model.decision_function(x_pool_unlabeled)
    decision_func_all_data = model.decision_function(features_all)

    if (USE_NORMALIZATION):
        decision_function_unlabeled = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function_unlabeled)
        decision_func_all_data = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_func_all_data)

    #Sort decision function
    indices_rank_decision_function_positif_unlabeled, distance_rank_decision_function_positif_unlabeled = utils.sort_descending_order(decision_func_all_data)

    try:
        v= indices_rank_decision_function_positif_unlabeled[rank]
    except IndexError:
        rank = 0

    number_to_remove = distance_rank_decision_function_positif_unlabeled[rank]

    decision_function_unlabeled = [x + number_to_remove for x in decision_function_unlabeled]

    return decision_function_unlabeled,number_to_remove

"""
Step 3 Interactive Learning
Preselection
"""
def preselection(dataset, decision_function, number):
    abs_decision_function = [math.fabs(number) for number in decision_function]
    tmp = copy.copy(abs_decision_function)
    sorted_indice = (np.array(abs_decision_function).argsort()[::-1])[:number]
    sorted_values = sorted(tmp,reverse = True)[:number]
    
    return sorted_indice, sorted_values

"""
Step 4 Interactive Learning : Selection
Consider the subset of images close to the boundary and then use a criterion related to Average precision to compute the cost g
"""
def selectioner(dataset, model_learning, kernel, y_train_gt, decision_function, pre_selection_data):
    feature_labeled,labeled = zip(*dataset.get_labeled_entries())
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])

    g = []
    distance_frontiere_abs= [math.fabs(number) for number in decision_function]
    decision_function_all= model_learning.decision_function(features_all)
    distance_frontiere_abs_all= [math.fabs(number) for number in decision_function_all]

    #Loop over all the uncertain example which have been chose
    for idx, value_id in enumerate(pre_selection_data):
       
        X = features_all[value_id]
        prediction= model_learning.predict(X.reshape(1, -1))

        if prediction[0] == -1 :
            prediction[0] = 1
            
        y_value_gt= [y_train_gt[value_id]]
        
        abs_decision_function_X=distance_frontiere_abs_all[value_id]
        #decision_function_X = model_learning.decision_function(X.reshape(1, -1))
        #abs_decision_function_X = math.fabs(decision_function_X)

        g_current=abs_decision_function_X * (1 - precision_score(y_value_gt,prediction))
        g.insert(idx,g_current)

    logging.debug('Final G : %s  ', len(g))

    return g

"""
Step 5 Interactive Learning : Diversity
https://www.aaai.org/Papers/ICML/2003/ICML03-011.pdf
"""
def diversifier(dataset, param_adjust, batch_number, g, params, pre_selection_data):
    kernel = params['kernel']
    gamma = params['gamma']
    selection= list()
    idx_all, features_all= zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    labeled_data= np.array(_ITEM_ALL_LABEL_DATA)

    for q in range(0, batch_number):
        result = list()
        for i, value_id_i in enumerate(pre_selection_data):
            similarities_i_j= list()
            feature_selected_i= features_all[value_id_i]
            g_current_selected_i= g[i]

            for j, value_id_j in enumerate(selection):
                feature_selected_j = features_all[value_id_j]
                #Compute angle diversity
                similarity_measure= utils.angle_kernel(kernel, gamma, feature_selected_i, feature_selected_j)
                similarities_i_j.append(similarity_measure)

            if similarities_i_j:
                max_similarity_measure_i_j= max(similarities_i_j)
            else :
                max_similarity_measure_i_j= 0

            
            result_diversity= param_adjust * math.fabs(g_current_selected_i) + (1 - param_adjust) * max_similarity_measure_i_j
            result.append(result_diversity)
            
        indice_select= np.argmin(result)
        id_selected= pre_selection_data[indice_select]
        #Remove the concerned id
        #del pre_selection_data[select]
        selection.append(id_selected)
        pre_selection_data= np.delete(pre_selection_data, indice_select)

    logging.debug('[INFO] Final selection id : %s  ', selection)
    
    return selection

"""
At the beginning we need to match the results given by the similarity search with the dataset of unlabeled data.
So from the similarity search, (from the json) we get an array of categories 
Then in the fully labeled dataset, we make a loop, extract the path of each object, extract the category and try to match with the categories in the array
At the end we have a list of ID which will be the first results to display to the user in order to annotate
"""
def select_data_similaritySearch(fully_labeled_train_dataset, categorie_array_results):
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count = 0
    list_idx_result = list()
    for idx1,path1 in enumerate(categorie_array_results[:25]):
        for idx2, path2 in enumerate(fully_labeled_train_dataset.data):
            filename = os.path.basename(path2[0][1])

            #The initial name is desc_category.pcd. We keep only the part after desc_
            filename = filename.split('_',1)[1]
            #Remove extension
            filename = os.path.splitext(filename)[0]

    
            if (path1 == filename):
                
                if idx2 not in list_idx_result:
                    list_idx_result.append(idx2)

                    lb = idealLabels.label(X[idx2])
                    count = count + 1


    return list_idx_result

def select_data_similaritySearch_byindex(fully_labeled_train_dataset,categorie_array_results,index_in,index_out):
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count = 0
    list_idx_result = list()

    range_categorie_array_results = categorie_array_results[index_in:index_out]

    for idx1,path1 in enumerate(range_categorie_array_results):

        for idx2, path2 in enumerate(fully_labeled_train_dataset.data):
            filename = os.path.basename(path2[0][1])
            #The initial name is desc_category.pcd. We keep only the part after desc_
            filename = filename.split('_',1)[1]
            #Remove the extension
            filename = os.path.splitext(filename)[0]
            #Check if there is additional data that we don't need . Ex donut3_0_esfgshotgrsdvfh , we want to remove _esfgshotgrsdvfh
            if (filename.count('_') == 2):
                filename = filename.rsplit('_',1)[0]
            if (filename == path1):
                if idx2 not in list_idx_result:
                    list_idx_result.append(idx2)

                    lb = idealLabels.label(X[idx2])
                    count = count + 1

    return list_idx_result



def compute_scores_SVM(X_test, y_test, y_pred):
    y_true = y_test
    ### F1 SCORE ###
    #The f1-score gives you the harmonic mean of precision and recall. 
    #The scores corresponding to every class will tell you the accuracy of the classifier 
    #in classifying the data points in that particular class compared to all other classes.
    #The support is the number of samples of the true response that lie in that class.
    f1score = f1_score(y_true, y_pred, average='weighted')  

    ### Average Precision SCORE ###
    AP= average_precision_score(y_true, y_pred)

    ### Precision recall curve ###
    precision_curve, recall_curve, thresholds = precision_recall_curve(y_true, y_pred)
    recall = recall_score(y_true, y_pred, average='weighted')
    precision = precision_score(y_true, y_pred, average='weighted')

    ### Classification report ###
    report_classification= classification_report(y_true, y_pred)

    ### Confusion matrix ###
    cnf_matrix = confusion_matrix(y_test, y_pred)
    np.set_printoptions(precision=2)


    logging.info("==> F1 score : %s", f1score)
    logging.info("==> Precision : %s", precision)
    logging.info("==> Recall : %s", recall)
    logging.info("==> Average Precision score : %s", AP)
    logging.info("==> Classification report : %s", report_classification)
    logging.info("==> Cconfusion_matrix : %s", cnf_matrix)


    return f1score, AP, precision, recall, precision_curve, recall_curve, report_classification, cnf_matrix


"""
Main function for interactive learning with similarity search
"""
def similarity_search_interactive_learning(params):
    global COMPUTE_FULL, OBJECTSLIST_PATH_FILE, DESCRIPTORSLIST_PATH_FILE, OUTPUT_GRAPHICALS_RESULTS, CATEGORY_LABEL

    descriptor_interactive_learning, result_jsonfile, save_figure = params

    categorie_array_results = utils.extract_names_objects_from_result_json(result_jsonfile)

    #Get the file which contain the path to every objects of the dataset
    listObjectsFromDataset= ROOT_DATA + NAME_DATASET + "/dataset_descriptor_" + descriptor_interactive_learning + ".txt"
    #Get the file which contain the path to every descriptors of the dataset
    listDescriptorsFromDataset = ROOT_DATA + NAME_DATASET + "/descriptors_"+ descriptor_interactive_learning + ".txt"
    OBJECTSLIST_PATH_FILE = listObjectsFromDataset
    DESCRIPTORSLIST_PATH_FILE = listDescriptorsFromDataset

    output_graphicals_result = "results_graphics" + NAME_DATASET + "/"
    OBJECTSLIST_PATH_FILE = listObjectsFromDataset
    DESCRIPTORSLIST_PATH_FILE = listDescriptorsFromDataset

    display_color_label = True

    #score of SVM for displaying data
    score_array, error_array, rank_array = [], [], []

    #Interval matplotlib
    reg_interval_y = plticker.MultipleLocator(base=10.0) # this locator puts ticks at regular intervals
    reg_interval_x = plticker.MultipleLocator(base=1.0) # this locator puts ticks at regular intervals

    nb_iterations = 0

    dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), DESCRIPTORSLIST_PATH_FILE)


    if COMPUTE_FULL == "true":
        fully_dataset_labeled_binary, train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views, path_list_test_views = \
        utils.split_train_test_from_training_testing_data_similaritySearch_GT(
            dataset_filepath, OBJECTSLIST_PATH_FILE, TRAINING_FILE_FULL,TESTING_FILE_FULL,
            categorie_array_results,CATEGORY_LABEL,COMPUTE_FULL
            )
            
    else :
        fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views, path_list_test_views = \
        utils.split_train_test_from_training_testing_data_similaritySearch_GT(
            dataset_filepath, OBJECTSLIST_PATH_FILE, TRAINING_FILE_VIEWS,TESTING_FILE_VIEWS,
            categorie_array_results,CATEGORY_LABEL, COMPUTE_FULL)

    
    y_test_binary = [entry[1] for idx, entry in enumerate(test_dataset_binary_labeled.data)]

    unlabeled_entry_list = list(train_dataset_unlabeled.get_unlabeled_entries())

    #Initialization rank
    rank = len(unlabeled_entry_list)/2
    print("\nItération : {} ".format(nb_iterations))

    list_idx_result = select_data_similaritySearch(train_dataset_binary_labeled, categorie_array_results)
    if (len(list_idx_result) == 0):
        print("[ERROR] List IDX empty ")
        logging.error("[ERROR] List IDX empty ")
        exit()
    
    display_data_init_similarity_search_classic(list_idx_result, train_dataset_binary_labeled, display_color_label)

    
    #Annotate first data
    id_unlabeled_display_first = [id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:NBRE_DATA_SELECTION])]

    total_annotated_id_list = annotate_data_similarity(train_dataset_binary_labeled, train_dataset_unlabeled, id_unlabeled_display_first)
    
    ################ FIRST TRAINING ###################
    model_learning = svm.SVC(kernel=KERNEL_SVM, C = C, gamma = GAM, class_weight='balanced', probability=True)
    training_data(train_dataset_unlabeled, model_learning)

    
    #Get parameters from the svm model
    _params = model_learning.get_params()
    _sv = model_learning.support_vectors_
    _nv = model_learning.n_support_
    _a  = model_learning.dual_coef_
    _b  = model_learning._intercept_
    _cs = model_learning.classes_

    #Get the id and features from unlabeled data
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
    #Get the id and features from all the data
    idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])

    #Get the probabilities result and decision function of all the data
    probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
    decision_function = model_learning.decision_function(X_pool_unlabeled)

    X_test, y = zip(*test_dataset_binary_labeled.get_labeled_entries())
    X_test =  np.array(X_test)
    y_pred = model_learning.predict(X_test)

    f1score, AP, precision, recall, precision_curve,recall_curve,report_classification, cnf_matrix = compute_scores_SVM(X_test, y_test_binary, y_pred)

    print("Precision = {}".format(precision))
    print("Recall = {}".format(recall))
    print("F1-Score = {}".format(f1score))

    
    ################ GRAPHIC ###################
    #Add the score to the chart
    score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
    error_array = np.append(error_array, 1 - score)
    score_array = np.append(score_array,score)
    score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
    score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
    title = "Score Success : " + str(score_ok)  + " Score Error : " + str(score_error) + " \n Rank : " + str(rank)
    rank_array = np.append(rank_array, rank)

    if (PLOT_SCORE):
        plt.figure(2)
        query_num = np.arange(0, 1)
        fig = plt.figure(2,figsize=(7, 8))
        ax = fig.add_subplot(2, 1, 1)
        p1, = ax.plot(query_num, error_array, 'r', label='Error')
        ax.set_xlabel('Number of steps')
        ax.set_ylabel('Error')
        ax.xaxis.set_major_locator(reg_interval_x)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
                   shadow=True, ncol=5)


        ay = fig.add_subplot(2, 1, 2)
        p2, = ay.plot(query_num, score_array, 'g', label='Score')
        ay.set_xlabel('Number of steps ')
        ay.set_ylabel('Score')
        ay.xaxis.set_major_locator(reg_interval_x)
        ay.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
                   shadow=True, ncol=5)
   
    
    if PLOT_PR_CURVE:
        # Plot Precision-Recall curve
        plt.figure(4)
        plt.plot(recall, precision, lw=2, color='navy',label='Precision-Recall curve')
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.ylim([0.0, 1.05])
        plt.xlim([0.0, 1.0])
        plt.title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
        plt.legend(loc="lower left")

    if PLOT_CNF_MATRIX:
        # Plot non-normalized confusion matrix
        plt.figure(5)
        class_names = ['Positif','Negatif']
        plotting.plot_confusion_matrix(cnf_matrix, classes=class_names, title='Confusion matrix, without normalization')

    
    if PLOT_CLF_REPORT:
        plotting.plot_classification_report(report_classification)


    plt.show(block = False)
 
    if (USE_NORMALIZATION):
        decision_function = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)

    
    #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
    indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function, False)
    idx_example_positif_selection = np.take(idx_unlabeled_data, indices_ranking_descending_order)[:NBRE_DATA_SELECTION]

    ################ SELECTION UNCERTAINTY ###################
    #ids_example_close_margin,_ = select_most_uncertains_samples_from_probabilities(train_dataset, probabilities_samples_unlabeled,nbre_data_margins_selection_uncertainty,True)
    ids_example_close_margin = select_most_uncertains_samples_from_decisionfunction(train_dataset_unlabeled, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY)
    total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))
    
    display_selected_data_similarity_search_withGT(
        train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,
        idx_example_positif_selection, ids_example_close_margin[:NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY_DISPLAY],
        display_color_label, save_figure)

    ###########=====>>>> SECOND STEP UPDATE  ###########
    total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
    idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
    decision_function = model_learning.decision_function(X_pool_unlabeled)
    decision_function_custom = utils.decision_function_vector(_params, _sv, _nv, _a, _b, X_pool_unlabeled)

    i = 0
    while nb_iterations < NB_ITERATIONS_MAX :
        nb_iterations = nb_iterations + 1
        print("\n=====>[INFO] Itération : ({}/{}) ".format(nb_iterations, NB_ITERATIONS_MAX))

        ################ 1. FEEDBACK ###################
        rank = feedback(train_dataset_unlabeled, rank,model_learning, total_annotated_id_list)
        training_data(train_dataset_unlabeled, model_learning)

        # Get parameters from model
        _params = model_learning.get_params()
        _sv = model_learning.support_vectors_
        _nv = model_learning.n_support_
        _a  = model_learning.dual_coef_
        _b  = model_learning._intercept_
        _cs = model_learning.classes_


        X_test, y = zip(*test_dataset_binary_labeled.get_labeled_entries())
        X_test =  np.array(X_test)
        y_pred = model_learning.predict(X_test)
        f1score, AP, precision, recall,precision_curve,recall_curve, report_classification, cnf_matrix = compute_scores_SVM(X_test, y_test_binary, y_pred)

        print("Score = {}".format(precision))
        print("Precision = {}".format(precision))
        print("Recall = {}".format(recall))
        print("F1-Score = {}".format(f1score))


        # Add score to the chart
        score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
        error_array = np.append(error_array, 1 - score)
        score_array = np.append(score_array, score)
        score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
        score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100

        rank_array = np.append(rank_array, rank)

        query_num = np.arange(0, i + 2)
        if (PLOT_SCORE):
            plt.figure(2)
            ax.set_xlim((0, i + 1))
            ax.set_ylim((0, max(error_array) + 0.2))
            ay.set_xlim((0, i + 1))
            ay.set_ylim((0, max(score_array) + 0.2))
            ax.xaxis.set_major_locator(reg_interval_x)
            ay.xaxis.set_major_locator(reg_interval_x)
            
            p1.set_xdata(query_num)
            p1.set_ydata(error_array)
            p2.set_xdata(query_num)
            p2.set_ydata(score_array)

        
        if PLOT_PR_CURVE:
            # Plot Precision-Recall curve
            plt.figure(4)
            plt.plot(recall, precision, lw=2, color='navy', label='Precision-Recall curve')
            plt.xlabel('Recall')
            plt.ylabel('Precision')
            plt.ylim([0.0, 1.05])
            plt.xlim([0.0, 1.0])
            plt.title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
            plt.legend(loc="lower left")

        if PLOT_CNF_MATRIX:
            # Plot non-normalized confusion matrix
            plt.figure(5)
            class_names = ['Positif','Negatif']
            plotting.plot_confusion_matrix(cnf_matrix, classes=class_names, title='Confusion matrix, without normalization')

    
        if PLOT_CLF_REPORT:
            plot_classification_report(report_classification)

        plt.show(block = False)

        
        idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
        idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
        probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
        decision_function = model_learning.decision_function(X_pool_unlabeled)

        indices_rank_decision_function_positif_without_correction, _ = utils.sort_descending_order(decision_function, True)
        idx_example_positif_selection_without_correction = np.take(idx_unlabeled_data,indices_rank_decision_function_positif_without_correction)[:NBRE_DATA_SELECTION]

        ################ 2. CORRECTION STEP ###################
        decision_function, number_to_remove = correction(train_dataset_unlabeled, model_learning, rank)
        indices_rank_new_decision_function_positif, distance_rank_decision_function_positif = utils.sort_descending_order(decision_function, True)
        idx_example_positif_selection = np.take(idx_unlabeled_data,indices_rank_new_decision_function_positif)[:NBRE_DATA_SELECTION]

        ################ 3. PRESELECTION STEP ###################
        ids_example_close_margin_correction = select_most_uncertains_samples_from_decisionfunction(train_dataset_unlabeled, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY,number_to_remove)
        ids_example_close_margin_without_correction = select_most_uncertains_samples_from_decisionfunction(train_dataset_unlabeled, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY)

        ################ 4. SELECTION STEP ###################
        kernel_ = 0
        cost_g = selectioner(train_dataset_unlabeled, model_learning, kernel_, y_train_binary_GT, decision_function, ids_example_close_margin_correction)


        ################ 5. DIVERSIFIER STEP ###################
        id_selection_diversification = diversifier(train_dataset_unlabeled, DIVERSIFICATION_LAMBDA, NBRE_DATA_FINAL_SELECTION, cost_g, _params, ids_example_close_margin_correction)

        save_figure = False

        display_selected_data_similarity_search_withGT(
            train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views, 
            idx_example_positif_selection,id_selection_diversification, display_color_label, save_figure)

        total_id_annotation = list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
        total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)

        i = i + 1
    
    
def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')



def main():
    print('The scikit-learn version is {}.'.format(skl.__version__))
    if not (skl.__version__ == '0.18'):
        print("You need yo use Scikit-learn 0.18")
        exit()

    global DESCRIPTOR_INTERACTIVELEARNING, DESCRIPTOR_SIMILARITYSEARCH, K, NB_ITERATIONS_MAX, LEAF_RESOLUTION, COMPUTE_FULL, OUTPUT_GRAPHICALS_RESULTS, NAME_DATASET
    #Ignore deprecation warninf from python
    warnings.filterwarnings("ignore", category=DeprecationWarning) 

    #Log 
    logging.basicConfig(filename=LOG_FILE, filemode='a', level=logging.DEBUG, format='%(asctime)s - %(levelname)s | %(message)s', datefmt='%H:%M:%S',)

    """
    The option we want to use for running this program
    1 --> Run interactive learning 3D with similarity search. The user makes a query, a similarity search is performed and then interactive learning is perform manually
    Need to change the variable view_path_file and dataset_descriptor at the top of the program
    2 -->  Run interactive learning 3D with similarity search AUTOMATICALLY by submitting a specific point clous query. Then a similarity search is performed and interactive learning as well
    3 --> Run interactive learning 3D with similarity search AUTOMATICALLY. For each dataset and descriptor, a similarity search is performed and interactive learning as well. Take long time
    """
    class option_interactiveLearning:
        similaritySearch_manuel = 1
        similaritySearch_automatic_oneobject = 2
        similaritySearch_automatic_alltesting = 3

    #Choose here the option you want
    current_option = option_interactiveLearning.similaritySearch_manuel

    if not os.path.isfile("cloudRetrieval_main"):
        print("\n [ERROR] Cloud Retrieval main binary does not exist - Please check the path")
        logging.error("Cloud Retrieval main binary does not exist - Please check the path")
        exit(1)

    ap = argparse.ArgumentParser()
    ap.add_argument("-query", "--query", required=True, help="The query point cloud (in PCD)")
    ap.add_argument("-trained", "--trained", required=True, help="The trained dataset where the index of similarity search is located")
    ap.add_argument("-descriptorSimilarity", "--descriptorSimilarity", required=False, help="Descriptor for similarity Search. Per default : esf")
    ap.add_argument("-descriptorInteractive", "--descriptorInteractive", required=False, help="Descriptor for interactive learning. Per default : esf")
    ap.add_argument("-output", "--output", required=False, help="where to save the json results file. Per default : results.json")
    ap.add_argument("-leaf_resolution", "--resolution", required=False, help="For cloud resolution invariance,. Per defaut : 0.01")
    ap.add_argument("-k", "--k", required=False, help="number of results to find. Per defaut : 10")
    ap.add_argument("-nbiterations", "--iterations", required=False, help="Number of iterations for automatic labelling. Per defaut : 10")
    args = vars(ap.parse_args())

    descriptor_similaritySearch = ''
    descriptor_interactivelearning = ''
    nb_iterations_max = ''
    k = ''
    leaf_resolution = ''
    output_json_result_similaritysearch = ''
    query_cloud = args["query"]
    trained_dataset = args["trained"]
    if args["descriptorSimilarity"] is not None:
        descriptor_similaritySearch = args["descriptorSimilarity"]
    else :
        descriptor_similaritySearch = DESCRIPTOR_SIMILARITYSEARCH
    if args["descriptorInteractive"] is not None:
        descriptor_interactiveLearning = args["descriptorInteractive"]
    else :
        descriptor_interactivelearning = DESCRIPTOR_INTERACTIVELEARNING
    if args["output"] is not None:
        output_json_result_similaritysearch = args["output"]
    else :
        output_json_result_similaritysearch = OUTPUT_JSON_RESULT_SIMILARITYSEARCH
    if args["resolution"] is not None:
        leaf_resolution = args["resolution"]
    else :
        leaf_resolution = LEAF_RESOLUTION
    if args["k"] is not None:
        k = args["k"]
    else :
        k = K
    if args["iterations"] is not None:
        nb_iterations_max = args["iterations"]
    else:
        nb_iterations_max = NB_ITERATIONS_MAX


    #Run interactive learning with similarity search
    if current_option == option_interactiveLearning.similaritySearch_manuel:
       
        cmd = './cloudRetrieval_main -query ' + query_cloud + ' -trained ' + trained_dataset + ' -descriptor ' + descriptor_similaritySearch + ' -k ' + str(k) + ' -leaf_resolution ' + str(leaf_resolution) + ' -output ' + output_json_result_similaritysearch + ' -compute_full ' + COMPUTE_FULL 
        os.system(cmd) # returns the exit status

        save_figure = False
        params = (descriptor_interactivelearning,OUTPUT_JSON_RESULT_SIMILARITYSEARCH,save_figure)
        similarity_search_interactive_learning(params)

    #Run automatic interactive learning with similarity search and by sumbittming one point cloud
    elif current_option == option_interactiveLearning.similaritySearch_automatic_oneobject:
        params = (descriptor_interactiveLearning,output_json_result,number_to_annotate_first_step,number_to_annotate_second_step,nb_iterations_max,repetition_experiment)
        #automatic_interactiveLearning(params,True)

    elif current_option == option_interactiveLearning.similaritySearch_automatic_alltesting:
        nb_iterations_max = 20
        num_cores = 2 #num_cores = mp.cpu_count() 
        use_multiprocessing = True
        
        pool = mp.Pool(processes=num_cores) 
        path_dataset = os.path.abspath(os.path.join(training_file_full, os.pardir))
        print("Number to select randomly : {}".format(number_object_to_select_randomly))
            
        #list_of_random_objects,category_list_name_array,result_list_numberObjectsPerClass =utils.get_all_objects_from_dataset(path_dataset,select_full_object,"ply")
        #exit()
        list_of_random_objects,category_list_name_array,result_list_numberObjectsPerClass = utils.get_random_objects_from_dataset(path_dataset,number_object_to_select_randomly,select_full_object,"ply")


        if (len(list_of_random_objects) == 0):
            logging.error("List of objects is empty")
            print("[ERROR] List of objects is empty")
        if (use_multiprocessing):
            params = (list_of_random_objects, path_dataset, name_dataset,result_list_numberObjectsPerClass)
            func = partial(automatic_interactiveLearning_objectlist, params)

            results_accuracy_score,results_positif_display = zip(*pool.map(func, descriptors))
    

if __name__ == '__main__':
    main()


