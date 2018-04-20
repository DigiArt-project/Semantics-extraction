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
#Location of the dataset directory where the txt file are located. The txt represents the list of all the descriptors and path to descriptors
ROOT_DATA = "data/"
NAME_DATASET = 'pottery_views'
#Where to save the graphics
OUTPUT_GRAPHICALS_RESULTS = "results_graphics/"
#Log file
LOG_FILE = "interactive_learning.log"

#Category label to change for option similarity search interactive learning
CATEGORY_LABEL = 6
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
#Defaut descriptor to use for Interactive Leaning
DESCRIPTOR_INTERACTIVELEARNING = "esf"

OBJECTSLIST_PATH_FILE = ""
DESCRIPTORSLIST_PATH_FILE = ""

_ITEM_CLICK_CALLBACK= set()
_ITEM_ALL_LABEL_DATA = list()

USE_NORMALIZATION = True


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
Display the first data to anotate for the classic interactive learning
"""
def display_data_init(dataset, objectslist_path, number_to_display):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1, figsize=(11, 10))
    fig.canvas.set_window_title('Results')
    outer_grid = gridspec.GridSpec(8, 6, wspace=1, hspace=0.8)
    for i in range(number_to_display):        
        ax = plt.Subplot(fig, outer_grid[i])
        filename = utils.get_filename_descriptor_from_path(objectslist_path[i],10)
        
        #feature = objectslist_path[i].decode("utf-8").split("/")[7]
        ax.text(0.1, 0.5, str(filename),
                size=7)
        title = "ID {}".format(dataset[i][0])
        ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)
        
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
Compute RBF or linear Kernel using two points
"""
def simple_kernel(kernel,gamma,x1,x2,coef0 = 0, degree = 3):
    if kernel == 'linear':
        #sv = support vectors
        return np.dot(x1, x2)
    elif kernel == 'rbf':
        return math.exp(-gamma * np.dot(x1 - x2, x1 - x2))
    elif kernel == 'sigmoid':
        return [np.tanh(gamma * np.dot(x1, x2) + coef0 )]
    elif kernel == 'sigmoid':
        return [np.tanh(gamma * np.dot(x1, x2) + coef0 )]
    elif kernel == 'poly':
        return [math.pow((gamma * np.dot(x1, x2) + coef0), degree )]


"""
Compute RBF or linear Kernel using vector
"""
def kernel(params, sv, X,coef0 = 0, degree = 3 ):
    if params['kernel'] == 'linear':
        #sv = support vectors
        return [np.dot(vi, X) for vi in sv]
    elif params['kernel'] == 'rbf':
        gamma = params['gamma']
        return [math.exp(-gamma * np.dot(vi - X, vi - X)) for vi in sv]
    elif params['kernel'] == 'sigmoid':
        gamma = params['gamma']
        return [np.tanh(gamma * np.dot(vi, X) + coef0 ) for vi in sv]
    elif params['kernel'] == 'poly':
        gamma = params['gamma']
        return [math.pow((gamma * np.dot(vi, X) + coef0), degree ) for vi in sv ]

"""
Compute diveristy with the angle kernel
Brinker : https://www.aaai.org/Papers/ICML/2003/ICML03-011.pdf
"""
def angle_kernel(kernel, gamma, x1,x2):
    simple_kernel_i_j = simple_kernel(kernel,gamma,x1,x2)
    simple_kernel_i_i = simple_kernel(kernel,gamma,x1,x1)
    simple_kernel_j_j = simple_kernel(kernel,gamma,x2,x2)

    if (isinstance(simple_kernel_i_j, list)):
        simple_kernel_i_j = simple_kernel_i_j[0]
        simple_kernel_i_i = simple_kernel_i_i[0]
        simple_kernel_j_j = simple_kernel_j_j[0]

    abs_kernel = math.fabs(simple_kernel_i_j)

    angle_kernel_result = (abs_kernel/ math.sqrt(simple_kernel_i_i * simple_kernel_j_j) )

    return angle_kernel_result


"""
Replicate the decision function of scikit : clf.decision_function(X)
Give the decision function of a specific point X
"""
def decision_function_point(params, sv, nv, a, b, X):
    # calculate the kernels
    k= kernel(params, sv, X)

    # define the start and end index for support vectors for each class
    start= [sum(nv[:i]) for i in range(len(nv))]
    end= [start[i] + nv[i] for i in range(len(nv))]

    # calculate: sum(a_p * k(x_p, x)) between every 2 classes
    c= [ sum(a[ i ][p] * k[p] for p in range(start[j], end[j])) +
          sum(a[j-1][p] * k[p] for p in range(start[i], end[i]))
                for i in range(len(nv)) for j in range(i+1,len(nv))]

    #add the intercept
    return [sum(x) for x in zip(c, b)]

"""
Replicate the decision function of scikit : clf.decision_function(X)
Give the decision function of a given Vector X (which contains many points)
"""
def decision_function_vector(params, sv, nv, a, b, X):
    decision_function = []
    # define the start and end index for support vectors for each class
    start = [sum(nv[:i]) for i in range(len(nv))]
    end = [start[i] + nv[i] for i in range(len(nv))]

    k = []
    for idx, value in enumerate(X):
        k_current = kernel(params, sv, value)
        k.insert(idx,k_current)

        c = [ sum(a[i][p] * k_current[p] for p in range(start[j], end[j])) + sum(a[j-1][p] * k_current[p] for p in range(start[i], end[i]))
                for i in range(len(nv)) for j in range(i+1,len(nv))]

        for x in zip(c, b):
            som = sum(x)
        
        decision_function.insert(idx,som)
    
    return np.array(decision_function)

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
        decision = decision_function_point(params, sv, nv, a, b, value)
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
count the number of positive label and negative label displayed to the user
"""
def compute_pourcentage_results(fully_labeled_train_dataset, train_dataset, id_to_display, number_to_consider = 20):

    if (number_to_consider > len(id_to_display)):
        number_to_consider = len(id_to_display) - 1

    ideal_labels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count_positif = 0
    count_negatif = 0
    count_total = 0
    #Count posiitve and negative display to the user
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

    logging.debug("Compute pourcentage result. Count positif : %s Count negatif %s | number to consider %s ", count_positif, count_negatif, number_to_consider)
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
def select_queries_uncertainty_decision_function(dataset, model_learning, m, correction_to_apply = 0):
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    features_all = np.array(features_all)
    idx_unlabeled_data, x_pool_unlabeled = zip(*dataset.get_unlabeled_entries())
    decision_function = model_learning.decision_function(features_all)

    if correction_to_apply != 0 :
        decision_function = [x - correction_to_apply for x in decision_function]

    
    indices_rank_decision_function_positif, distance_rank_decision_function_positif = utils.sort_ascending_order(decision_function,True)
    #print("Decision function before take closest 0 {}" .format(decision_function))
    indices_rank_closest_zero = utils.take_closest_to_zero(decision_function)
    #print("indices_rank_closest_zero {}" .format(indices_rank_closest_zero))
    idx_example_positif_selection = np.take(idx_all,indices_rank_closest_zero)[:m]
    idx_example_positif_selection1 = indices_rank_closest_zero[:m]

    
    return idx_example_positif_selection

"""
Based on the probabilities, Select the most uncertain data the smallest probabilities.
 So the most uncertains are the one which their probabilities are close to 0
"""
def select_queries_uncertainty_sampling(dataset, probabilities, m, return_score=False):
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
Step 1 Interactive Learning
Feedback
Compute the rank based on positive and negatives samples selected by the user
"""
def feedback(whole_dataset, current_rank, model_learning, annotated_data_selection):
    #Have the score of the selected example
    logging.debug('Previous Rank : %s  ', current_rank)

    h = 0
    new_rank = 0

    idx_unlabeled_data, x_pool_unlabeled = zip(*whole_dataset.get_unlabeled_entries())
    X_pool_labeled, labels = zip(*whole_dataset.get_labeled_entries())
    decision_function_unlabeled = model_learning.decision_function(x_pool_unlabeled)

    label_all,features_all = zip(*[(idx, entry[0]) for idx, entry in enumerate(whole_dataset.data)])
    features_all = np.array(features_all)

    decision_function_all = model_learning.decision_function(features_all)
    decision_function_all = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function_all)
    #print("[INFO]Decision function all feature : {} ".format(decision_function_all))

    decision_func_tmp_unlabel = copy.copy(decision_function_unlabeled)

    for idx,selection in enumerate(annotated_data_selection):
        logging.debug('=> h = %s  ', h)

        ask_id = int(selection[0])
        label = int(selection[1])

        logging.debug('Label %s for ID %s  ', label, ask_id)

        X = features_all[ask_id]
       
        decision_function_X = decision_function_all[ask_id]

        h_current = label - decision_function_X
        
        h = h + h_current
    #print("h final =  {} ".format(h))
    new_rank = current_rank + h

    #print("New  rank : {}".format(int(round(new_rank))))
    logging.debug('New rank = %s  ', new_rank)

    return int(round(new_rank))

"""
Step 2 Interactive Learning
Classification
"""
def training_data(training_dataset, model):
    model.fit(*(training_dataset.format_sklearn()))

"""
Step 3 Interactive Learning
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
Step 4 Interactive Learning
Preselection
"""
def preselection(dataset, decision_function, m):
    abs_decision_function = [math.fabs(number) for number in decision_function]
    tmp = copy.copy(abs_decision_function)
    sorted_indice = (np.array(abs_decision_function).argsort()[::-1])[:m]
    sorted_values = sorted(tmp,reverse = True)[:m]
    
    return sorted_indice, sorted_values

"""
Step 5 Interactive Learning
Selection
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
       
        X= features_all[value_id]
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
Step 5 Interactive Learning
Diversity
https://www.aaai.org/Papers/ICML/2003/ICML03-011.pdf
"""
def diversifier(dataset, param_adjust, batch_number, g, params, pre_selection_data):
    kernel= params['kernel']
    gamma= params['gamma']
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
                similarity_measure= angle_kernel(kernel, gamma, feature_selected_i, feature_selected_j)
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



def compute_scores_SVM(X_test, y_test, y_pred, model_learning, plot_PR_curve=False, plot_cnf_matrix=False, plot_clf_report=False, save_figure=False):
    y_true=y_test
    ### F1 SCORE ###
    #The f1-score gives you the harmonic mean of precision and recall. 
    #The scores corresponding to every class will tell you the accuracy of the classifier 
    #in classifying the data points in that particular class compared to all other classes.
    #The support is the number of samples of the true response that lie in that class.
    f1score = f1_score(y_true, y_pred, average='weighted')  

    ### Average Precision SCORE ###
    AP= average_precision_score(y_true, y_pred)

    ### Precision recall curve ###
    precision_curve, recall_curve, thresholds= precision_recall_curve(y_true, y_pred)
    recall= recall_score(y_true, y_pred, average='weighted')
    precision= precision_score(y_true, y_pred, average='weighted')

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
Interactive learning without similarity search. Basic test
"""
def basic_interactive_learning_start(params):
    global OBJECTSLIST_PATH_FILE
    global DESCRIPTORSLIST_PATH_FILE

    DESCRIPTOR_INTERACTIVELEARNING = params
    #Get the file which contain the path to every objects of the dataset
    listObjectsFromDataset= ROOT_DATA + NAME_DATASET + "/dataset_descriptor_" + DESCRIPTOR_INTERACTIVELEARNING + ".txt"
    #Get the file which contain the path to every descriptors of the dataset
    listDescriptorsFromDataset= ROOT_DATA + NAME_DATASET + "/descriptors_"+ DESCRIPTOR_INTERACTIVELEARNING + ".txt"
    OBJECTSLIST_PATH_FILE= listObjectsFromDataset
    DESCRIPTORSLIST_PATH_FILE= listDescriptorsFromDataset
    
    
    E_in, E_out, rank_array = [], [], []
    nb_iterations = 1
    

    dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), DESCRIPTORSLIST_PATH_FILE)
    fully_dataset,train_dataset, X_test,y_test, y_train_gt, fully_labeled_train_dataset, path_list_train_views, path_list_test_views = utils.split_train_test_from_libsvm_data(dataset_filepath, OBJECTSLIST_PATH_FILE,  TEST_SIZE)
    unlabeled_entry_list = list(train_dataset.get_unlabeled_entries())


    #Initialization rank
    rank = len(unlabeled_entry_list)/2

    print("[INFO] Initial rank : {} ".format(rank))
    print("\n=====> Itération : {} ".format(nb_iterations))
    logging.info('\n=====> Itération : %s  ', nb_iterations)


    ################ FIRST PLOT ###################
    start = time.time()
    display_data_init(unlabeled_entry_list, path_list_train_views, NBRE_DATA_SELECTION)
    stop = time.time()
    print("Displya data init - time : {} s".format(stop - start))
    test_dataset,X_test_new,y_test_new,CATEGORY_LABEL = utils.get_testDataset_binary(X_test, y_test, OBJECTSLIST_PATH_FILE)

    y_train_binary_gt = utils.get_train_label_binary(y_train_gt, CATEGORY_LABEL)
    #Annotate first data
    id_unlabeled_display_first= [id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:NBRE_DATA_SELECTION])]
    total_annotated_id_list= annotate_data(train_dataset, id_unlabeled_display_first)
    
    ################ FIRST TRAINING ###################
    model_learning = svm.SVC(kernel=KERNEL_SVM,C=C, gamma = GAM, class_weight = 'balanced', probability = True)

    training_data(train_dataset, model_learning)


    #Get parameters from the svm model
    _params = model_learning.get_params()
    _sv = model_learning.support_vectors_
    _nv = model_learning.n_support_
    _a = model_learning.dual_coef_
    _b = model_learning._intercept_
    _cs = model_learning.classes_

    #Get the id and features from unlabeled data
    idx_unlabeled_data, x_pool_unlabeled = zip(*train_dataset.get_unlabeled_entries())
    #Get the id and features from all the data
    idx_all_data, x_pool_all= zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset.data)])

    #Get the probabilities result and decision function of all the data
    probabilities_samples_unlabeled = model_learning.predict_proba(x_pool_unlabeled)
    ##HAS BEEN CHANGED
    decision_function= model_learning.decision_function(x_pool_unlabeled)

    ################ GRAPHIC ###################
    #Add the score to the chart
    score = model_learning.score(*(test_dataset.format_sklearn()))
    E_out = np.append(E_out, 1 - score)
    E_in = np.append(E_in,score)
    score_ok = model_learning.score(*(test_dataset.format_sklearn()))*100
    score_error = (1 - model_learning.score(*(test_dataset.format_sklearn())))*100
    #title = "Score Success : " + str(score_ok)  + " Score Error : " + str(score_error) + " \n Rank : " + str(rank)
    rank_array=np.append(rank_array, rank)

    query_num=np.arange(0, 1)
    fig=plt.figure(figsize=(7, 8))
    ax=fig.add_subplot(2, 1, 1)
    p1,= ax.plot(query_num, E_out, 'r', label='Error')
    ax.set_xlabel('Number of steps')
    ax.set_ylabel('Error')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
               shadow=True, ncol=5)

    ay = fig.add_subplot(2, 1, 2)
    p2, = ay.plot(query_num, E_in, 'g', label='Score')
    ay.set_xlabel('Number of steps ')
    ay.set_ylabel('Score')
    ay.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
               shadow=True, ncol=5)
  
    plt.show(block=False)
 
    if (USE_NORMALIZATION):
        decision_function=MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)


    #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
    indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function,False)
    idx_example_positif_selection=np.take(idx_unlabeled_data, indices_ranking_descending_order)[:NBRE_DATA_SELECTION]

    ################ SELECTION UNCERTAINTY ###################
    #ids_example_close_margin,_ = select_queries_uncertainty_sampling(train_dataset, probabilities_samples_unlabeled,NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY,True)
    start=time.time()
    ids_example_close_margin=select_queries_uncertainty_decision_function(train_dataset, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY)
    stop=time.time()
    print("Select queries uncertainty decision - time : {} s".format(stop - start))
    total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))
    display_selected_data(train_dataset, path_list_train_views, idx_example_positif_selection, ids_example_close_margin[:NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY_DISPLAY])

    ###########=====>>>> SECOND STEP UPDATE  ###########
    total_annotated_id_list = annotate_data(train_dataset,total_id_annotation)
    idx_unlabeled_data,x_pool_unlabeled = zip(*train_dataset.get_unlabeled_entries())
    decision_function=model_learning.decision_function(x_pool_unlabeled)
    decision_function_custom=decision_function_vector(params, _sv, _nv, _a, _b, x_pool_unlabeled)

    decision_func = copy.copy(decision_function)
    i = 0
    while True :
        nb_iterations = nb_iterations + 1
        print("\n=====>[INFO] Itération : {} ".format(nb_iterations))
        startStep2=time.time()

        ################ 1. FEEDBACK ###################
        rank=feedback(train_dataset, rank, model_learning, total_annotated_id_list)
 
        training_data(train_dataset, model_learning)

        # Get parameters from model
        params=model_learning.get_params()
        _sv = model_learning.support_vectors_
        _nv = model_learning.n_support_
        _a = model_learning.dual_coef_
        _b = model_learning._intercept_
        _cs = model_learning.classes_

        # Add score to the chart
        score=model_learning.score(*(test_dataset.format_sklearn()))
        E_out=np.append(E_out, 1 - score)
        E_in=np.append(E_in, score)
        score_ok=model_learning.score(*(test_dataset.format_sklearn()))*100
        score_error=(1 - model_learning.score(*(test_dataset.format_sklearn())))*100
        rank_array=np.append(rank_array, rank)

        ax.set_xlim((0, i + 1))
        ax.set_ylim((0, max(E_out) + 0.2))
        ay.set_xlim((0, i + 1))
        ay.set_ylim((0, max(E_in) + 0.2))

        query_num=np.arange(0, i + 2)

        p1.set_xdata(query_num)
        p1.set_ydata(E_out)
        p2.set_xdata(query_num)
        p2.set_ydata(E_in)


        plt.show(block=False)

        idx_unlabeled_data,x_pool_unlabeled=zip(*train_dataset.get_unlabeled_entries())
        idx_all_data,x_pool_all=zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset.data)])
        probabilities_samples_unlabeled=model_learning.predict_proba(x_pool_unlabeled)
        decision_function=model_learning.decision_function(x_pool_unlabeled)

        indices_rank_decision_function_positif_without_correction,_= utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection_without_correction=np.take(idx_unlabeled_data, indices_rank_decision_function_positif_without_correction)[:NBRE_DATA_SELECTION]

        ################ 2. CORRECTION ###################
        decision_function, number_to_remove=correction(train_dataset, model_learning, rank)
        indices_rank_new_decision_function_positif, distance_rank_decision_function_positif=utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection=np.take(idx_unlabeled_data, indices_rank_new_decision_function_positif)[:NBRE_DATA_SELECTION]

        ################ 3. PRESELECTION ###################
        ids_example_close_margin_correction=select_queries_uncertainty_decision_function(train_dataset, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY, number_to_remove)
        ids_example_close_margin_without_correction=select_queries_uncertainty_decision_function(train_dataset, model_learning, NBRE_DATA_MARGINS_SELECTION_UNCERTAINTY)

        ################ 4. SELECTION ###################
        kernel_=0
        cost_g=selectioner(train_dataset, model_learning,kernel_, y_train_binary_gt, decision_function, ids_example_close_margin_correction)


        ################ 5. DIVERSIFIER ###################
        param_adjust=0.5
        id_selection_diversification=diversifier(train_dataset, param_adjust, NBRE_DATA_FINAL_SELECTION, cost_g,params, ids_example_close_margin_correction)


        stopStep2=time.time()
        print("Computation time : {} s".format(stopStep2 - startStep2 ))
        display_selected_data(train_dataset,path_list_train_views, idx_example_positif_selection, id_selection_diversification)
        total_id_annotation=list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
        total_annotated_id_list=annotate_data(train_dataset, total_id_annotation)

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

    global DESCRIPTOR_INTERACTIVELEARNING, OUTPUT_GRAPHICALS_RESULTS,NAME_DATASET
    #Ignore deprecation warninf from python
    warnings.filterwarnings("ignore", category=DeprecationWarning) 


    #Log 
    logging.basicConfig(filename=LOG_FILE, filemode='a', level=logging.DEBUG, format='%(asctime)s - %(levelname)s | %(message)s', datefmt='%H:%M:%S',)

    ap=argparse.ArgumentParser()
    ap.add_argument("-descriptorInteractive", "--descriptorInteractive", required=False, help="Descriptor for interactive learning. Per default : esf")
    args = vars(ap.parse_args())

    if args["descriptorInteractive"] is not None:
        DESCRIPTOR_INTERACTIVELEARNING = args["descriptorInteractive"]
        
    params = (DESCRIPTOR_INTERACTIVELEARNING)
    basic_interactive_learning_start(params)

    

if __name__ == '__main__':
    main()


