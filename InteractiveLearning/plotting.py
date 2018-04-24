import numpy as np
import random
import utils 
import matplotlib.pyplot as plt
from importing_modules import *
from collections import OrderedDict


reg_interval_y = plticker.MultipleLocator(base=10.0) # this locator puts ticks at regular intervals
reg_interval_x = plticker.MultipleLocator(base=1.0) # this locator puts ticks at regular intervals

def plot_precision_recall(recall, precision, AP, save_figure=False, name_figure="Precision_Recall_Curve.png"):
    # Plot Precision-Recall curve
    fig = plt.figure(figsize=(6, 6))
    plt.plot(recall, precision, lw=3, color='navy', label='Precision-Recall curve')
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.ylim([0.0, 1.05])
    plt.xlim([0.0, 1.0])
    plt.title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
    plt.legend(loc="lower left")
    if save_figure:
        fig.savefig(name_figure, bbox_inches='tight')

def plot_curves(query_num, figure_data, figure_colour, figure_label, figure_subtitle, main_title="Curves result", save_figure=False, name_figure="result.png" ):
    nb_figures = len(figure_data)
    fig = plt.figure(figsize=(6, 6))
    plt.suptitle(main_title, fontsize=8, fontweight='bold')
    for i in range(nb_figures):
        ax = fig.add_subplot(nb_figures, 1, i+1)
        ax.plot(query_num, figure_data[i], figure_colour[i], label=figure_label[i],lw = 3)
        ax.set_title(figure_subtitle[i],fontsize=7,fontweight='bold')
        lgd_ax = ax.legend(bbox_to_anchor=(1.05, 1),loc=2, borderaxespad=0.,fancybox=True, shadow=True)
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Pourcent %")
        ax.set_ylim((0, 100 + 1))
        ax.yaxis.set_major_locator(reg_interval_y)
        ax.xaxis.set_major_locator(reg_interval_x)
        
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)

    if (save_figure):
        fig.savefig(name_figure, bbox_inches='tight')

def plot_curves_2(
    query_num, figure_data, figure_colour,
    figure_label, figure_subtitle, main_title,
    name_figure, name_dataset, name_category,
    name_descriptor, output_graphicals_result,
    save_figure = False ):
    nb_figures = len(figure_data)
    fig = plt.figure(figsize=(11, 8))
    for i in range(nb_figures):
        ax = fig.add_subplot(nb_figures, 1, i+1)
        plt.suptitle(main_title, fontsize=8, fontweight='bold')
        #ax = f1.add_subplot(111)
        ax.plot(query_num, figure_data[i], figure_colour[i], label=figure_label[i], lw = 3)
        ax.set_title(figure_subtitle[i],fontsize=7,fontweight='bold')
        lgd_ax = ax.legend(bbox_to_anchor=(1.05, 1),loc=2, borderaxespad=0.,fancybox=True, shadow=True)
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Pourcent %")
        ax.set_ylim((0, 100 + 1))
        ax.yaxis.set_major_locator(reg_interval_y)
        ax.xaxis.set_major_locator(reg_interval_x)
        plt.tight_layout()
        # Add space at top
        plt.subplots_adjust(top=0.83)
        if (save_figure):
            title_figure = name_dataset + "_" + name_category + "_" + name_descriptor + "_" + name_figure[i] + ".png"
            paht_to_save = output_graphicals_result + title_figure
            fig.savefig(paht_to_save, bbox_inches='tight')
            #Save points
            title_points = name_dataset + "_" + name_category + "_" + name_descriptor + "_mean_" + name_figure[i] + ".txt"
            paht_to_save = output_graphicals_result + title_points
            utils.save_list_points(figure_data[i],paht_to_save)
        
def plot_curves_final(
    query_num, figure_data, figure_colour, figure_label,
    figure_subtitle, main_title, name_figure, name_dataset,
    name_descriptor, output_graphicals_result, save_figure = False ):
    nb_figures = len(figure_data)
    fig = plt.figure(figsize=(11, 8))
    for i in range(nb_figures):
        ax = fig.add_subplot(nb_figures, 1, i+1)
        #ax = plt.subplot(2, 1, i+1)
        ax.plot(query_num, figure_data[i], figure_colour[i], label=figure_label[i],lw = 4)
        ax.set_title(figure_subtitle[i],fontsize=7,fontweight='bold')
        lgd_ax = ax.legend(bbox_to_anchor=(1.05, 1),loc=2, borderaxespad=0.,fancybox=True, shadow=True)
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Pourcent %")
        ax.set_ylim((0, 100 + 1))
        ax.yaxis.set_major_locator(reg_interval_y)
        ax.xaxis.set_major_locator(reg_interval_x)
        if (save_figure):
            #Save points
            title_points = name_dataset + "_" + name_descriptor + "_mean_"+ name_figure[i] +".txt"
            path_to_save = output_graphicals_result + title_points
            utils.save_list_points(figure_data[i],path_to_save)

        
    plt.tight_layout()
    # Add space at top
    plt.subplots_adjust(top=0.83)


    if (save_figure):
        title_figure = name_dataset + "_" + name_descriptor + ".png"
        paht_to_save = output_graphicals_result + title_figure
        fig.savefig(paht_to_save, bbox_inches='tight')


def plot_curves_NN_FT_ST(
    query_num,NN_score, FT_score, ST_score,
    main_title, name_dataset, name_category,
    name_descriptor, output_graphicals_result
    save_figure=False):

    fig2 = plt.figure(figsize=(11, 8))
    plt.suptitle(main_title, fontsize=14, fontweight='bold')
    ax = fig2.add_subplot(3, 1, 1)
    p1, = ax.plot(query_num, NN_score, lw=3, color='g')
    #ax.set_xlabel('Number of steps')
    ax.set_ylabel('Score')
    ax.set_title("NN over iterations")
    ax.set_ylim([0.0, 101.5])
    ax.xaxis.set_major_locator(reg_interval_x)
    #ax.set_title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(mean_average_precision))
    lgd_ax = ax.legend(loc="lower left")

    ay = fig2.add_subplot(3, 1, 2)
    p2, = ay.plot(query_num, FT_score, 'g', label='FT', lw = 3)
    ay.set_ylim([0.0, 101.5])
    ay.xaxis.set_major_locator(reg_interval_x)
    ay.set_title("FT over iterations")
    #ay.set_xlabel('Number of steps ')
    ay.set_ylabel('Score')
    lgd_ay = ay.legend(loc="lower left")

    az = fig2.add_subplot(3, 1, 3)
    p3, = az.plot(query_num, ST_score, 'g', label='ST', lw = 3)
    az.set_ylim([0.0, 101.5])
    az.xaxis.set_major_locator(reg_interval_x)
    az.set_title("ST over iterations")
    #ay.set_xlabel('Number of steps ')
    az.set_ylabel('Score')
    title_figure = name_dataset + "_" + name_category + "_" + name_descriptor + "_NN_FT_ST.png"
    if save_figure:
        paht_to_save = output_graphicals_result + title_figure
        fig2.savefig(paht_to_save, bbox_inches='tight')
        

def plot_curves_precision_recall(
    query_num, recall_array, precision_array, ecall_curve,
    precision_curve, AP, save_figure=False,
    name_figure = "Result_curve_precision_recall.png"):

    # Plot Precision-Recall curve
    fig1 = plt.figure(figsize=(10,7))
    ax = fig1.add_subplot(3, 1, 1)
    p1, = ax.plot(recall_curve, precision_curve, lw=3, color='navy',label='Recall curve')
    ax.set_xlabel('Recall')
    ax.set_ylabel('Precision')
    ax.set_ylim([0.0, 1.05])
    ax.set_xlim([0.0, 1.0])
    ax.set_title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
    lgd_ax = ax.legend(loc="lower left")
        

    ay = fig1.add_subplot(3, 1, 2)
    p2, = ay.plot(query_num, precision_array, 'g', label='Precision',lw = 3)
    ay.set_ylim([0.0, 1.05])
    ay.xaxis.set_major_locator(reg_interval_x)
    ay.set_xlabel('Number of steps ')
    ay.set_ylabel('Precision')
    lgd_ay = ay.legend(loc="lower left")

        
    az = fig1.add_subplot(3, 1, 3)
    p3, = az.plot(query_num, recall_array, 'c', label='Recall', lw = 3)
    az.set_ylim([0.0, 1.05])
    az.xaxis.set_major_locator(reg_interval_x)
    az.set_xlabel('Number of steps ')
    az.set_ylabel('Recall')
    lgd_az = az.legend(loc="lower left")

    if (save_figure):
        fig1.savefig(name_figure, bbox_inches='tight')
        

def plot_f1_precision_recall_final(
    query_num, f1_score, precision_score, recall_score,
    recall_curve, precision_curve, average_precision,
    name_dataset, name_category, name_descriptor,
    output_graphicals_result, title, save_figure = False):

    fig = plt.figure(figsize=(11, 8))
    plt.suptitle(title, fontsize=16, fontweight='bold')

    ax = fig.add_subplot(2, 2, 1)
    p1, = ax.plot(query_num, recall_score, lw=3, color='navy',label='Recall')
    #ax.set_xlabel('Number of steps')
    ax.set_ylabel('Recall')
    ax.set_title("Recall over iterations")
    ax.set_ylim([0.0, 1.05])
    ax.xaxis.set_major_locator(reg_interval_x)
    #ax.set_title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(mean_average_precision))
    lgd_ax = ax.legend(loc="lower left")

    ay = fig.add_subplot(2, 2, 2)
    p2, = ay.plot(query_num, precision_score, 'g', label='Precision',lw = 3)
    ay.set_ylim([0.0, 1.05])
    ay.xaxis.set_major_locator(reg_interval_x)
    ay.set_title("Precision over iterations")
    #ay.set_xlabel('Number of steps ')
    ay.set_ylabel('Precision')
    lgd_ay = ay.legend(loc="lower left")
    
    az = fig.add_subplot(2, 2, 3)
    p3, = az.plot(query_num,f1_score, 'c', label='F1 Score',lw = 3)
    #az.set_xlabel('Number of steps ')
    az.set_title("F1 score over iterations")
    az.set_ylabel('F1 score')
    az.xaxis.set_major_locator(reg_interval_x)
    lgd_az = az.legend(loc="lower left")


    aw = fig.add_subplot(2, 2, 4)
    p3, = aw.plot(recall_curve,precision_curve, lw=3, color='navy',label='Precision Recall curve')
    aw.set_xlabel('Recall')
    aw.set_ylabel('Precision')
    aw.set_ylim([0.0, 1.05])
    aw.set_xlim([0.0, 1.0])
    aw.set_title('Average Precision Score AUC={0:0.2f}'.format(average_precision))
    lgd_aw = aw.legend(loc="lower left")

    if (save_figure):
        title_figure = name_dataset + "_" + name_descriptor + "mean_average_precision.png"
        paht_to_save = output_graphicals_result + title_figure
        fig.savefig(paht_to_save, bbox_inches='tight')

    plt.tight_layout()
    # Add space at top
    plt.subplots_adjust(top=0.83)

def plot_f1_precision_recall(
    query_num, f1_score, precision_score, recall_score, recall_curve,
    precision_curve, average_precision, name_dataset, name_category,
    name_descriptor, output_graphicals_result, title, save_figure = False):
    fig2 = plt.figure(figsize=(11, 8))
    plt.suptitle(title, fontsize=16, fontweight='bold')

    ax = fig2.add_subplot(2, 2, 1)
    p1, = ax.plot(query_num, recall_score, lw=4, color='navy',label='Recall')
    #ax.set_xlabel('Number of steps')
    ax.set_ylabel('Recall')
    ax.set_title("Recall over iterations")
    ax.set_ylim([0.0, 1.05])
    ax.xaxis.set_major_locator(reg_interval_x)
    #ax.set_title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(mean_average_precision))
    lgd_ax = ax.legend(loc="lower left")

    ay = fig2.add_subplot(2, 2, 2)
    p2, = ay.plot(query_num, precision_score, 'g', label='Precision', lw = 4)
    ay.set_ylim([0.0, 1.05])
    ay.xaxis.set_major_locator(reg_interval_x)
    ay.set_title("Precision over iterations")
    #ay.set_xlabel('Number of steps ')
    ay.set_ylabel('Precision')
    lgd_ay = ay.legend(loc="lower left")
    
    az = fig2.add_subplot(2, 2, 3)
    p3, = az.plot(query_num,f1_score, 'c', label='F1 Score')
    #az.set_xlabel('Number of steps ')
    az.set_title("F1 score over iterations")
    az.set_ylabel('F1 score')
    az.xaxis.set_major_locator(reg_interval_x)
    lgd_az = az.legend(loc="lower left")

    aw = fig2.add_subplot(2, 2, 4)
    p3, = aw.plot(recall_curve,precision_curve, lw=4, color='navy',label='Precision Recall curve')
    aw.set_xlabel('Recall')
    aw.set_ylabel('Precision')
    aw.set_ylim([0.0, 1.05])
    aw.set_xlim([0.0, 1.0])
    aw.set_title('Average Precision Score AUC={0:0.2f}'.format(average_precision))
    lgd_aw = aw.legend(loc="lower left")
     
    if (save_figure):
        title_figure = name_dataset + "_" + name_category + "_" + name_descriptor + "_averageprecision.png"
        paht_to_save = output_graphicals_result + title_figure
        fig2.savefig(paht_to_save, bbox_inches='tight')
       

def get_cmap(n, name='Set1'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name,n)


def plot_bar(descriptors_array,categories_array,score_array,title):
    len_category = len(categories_array)
    bar_width = 0.05
    opacity = 1.0
    fig, ax = plt.subplots(figsize = (13,13))


    cmap = get_cmap(len(descriptors_array))
    labeldic = {idx:descriptors_array[idx] for idx in range(0,len(descriptors_array)) }

    score_array = np.asarray(score_array)
    index = np.arange(len_category)
    #index = np.arange(score_array.shape[0])

    #a = np.c_[test1,test2]
    maxi = np.max(score_array, axis=0)
    maxi = [ '%.2f' % round(value,1) for value in maxi]

    l = ["{}\n{}".format(labeldic[j],i) for i,j in zip(maxi, np.argmax(score_array, axis=0))]


    for i in range(len(descriptors_array)):
        plt.bar(index-0.2+bar_width*i, score_array[i,:], width=bar_width,alpha=opacity,label = labeldic[i],color=cmap(i))

    for i in range(score_array.shape[1]): 
        plt.annotate(l[i], xy=(i,maxi[i]), xytext=(0,20), size=12,bbox=dict(boxstyle="round4", fc="w"),
                     textcoords="offset points", ha="center",va="center",color='blue', fontweight='bold')
    plt.margins(y=0.2)

    plt.xlabel('Categories')
    plt.ylabel('Scores')
    plt.title(title)
    plt.xticks(index + bar_width, categories_array)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return fig

def plot_points(descriptors_array, categories_array, score_array, title):
    len_category = len(categories_array)
    bar_width = 0.05
    opacity = 1.0
    fig, ax = plt.subplots(figsize = (13,13))

    cmap = get_cmap(len(descriptors_array))
    labeldic = {idx:descriptors_array[idx] for idx in range(0,len(descriptors_array)) }

    score_array = np.asarray(score_array)
    index = np.arange(len_category)
    #index = np.arange(score_array.shape[0])

    #a = np.c_[test1,test2]
    maxi = np.max(score_array, axis=0)
    maxi = [ '%.2f' % round(value,1) for value in maxi]

    l = ["{}\n{}".format(labeldic[j],i) for i,j in zip(maxi, np.argmax(score_array, axis=0))]

    for i in range(len(descriptors_array)):
        plt.plot(score_array[i,:],alpha=opacity,label = labeldic[i],color=cmap(i), lw = 4 )

    for i in range(score_array.shape[1]): 
        plt.annotate(l[i], xy=(i,maxi[i]), xytext=(0,60), size=10,bbox=dict(boxstyle="round4", fc="w"),
                     textcoords="offset points", ha="center",va="center",color='blue', fontweight='bold')
    plt.margins(y=0.2)
    plt.xlabel('Categories')
    plt.ylabel('Scores')
    plt.title(title)
    plt.xticks(index + bar_width, categories_array)
    plt.legend()
    plt.tight_layout()
    plt.show(block = True)

    return fig



"""
Visualize high dimensional data
"""
def visualisation_high_dimensional(X, y, model_learning):
    db = DBPlot(model_learning,PCA(n_components=2),acceptance_threshold=0.03, n_decision_boundary_keypoints=10, n_generated_testpoints_per_keypoint=15)
    X_train = np.array(X)
    y_train = np.array(y)
    db.fit(X_train, y_train)
    db.plot(plt, generate_testpoints=False,background_resolution = 100,scatter_size_scale = 0.4)  # set generate_testpoints=False to speed up plotting
    db.plot().show()



def plot_confusion_matrix(cm, classes,normalize=False,title='Confusion matrix',cmap=plt.cm.Blues, save_figure = True):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    plt.clf()
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        print("Normalized confusion matrix")
        #else:
        #print('Confusion matrix, without normalization')

    #print(cm)

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, cm[i, j],
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    #plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')

    return plt
        


def plot_learning_curve(estimator, title, X, y, ylim=None, cv=None,n_jobs=1, train_sizes=np.linspace(.1, 1.0, 5)):
    """
    Generate a simple plot of the test and training learning curve.

    Parameters
    ----------
    estimator : object type that implements the "fit" and "predict" methods
        An object of that type which is cloned for each validation.

    title : string
        Title for the chart.

    X : array-like, shape (n_samples, n_features)
        Training vector, where n_samples is the number of samples and
        n_features is the number of features.

    y : array-like, shape (n_samples) or (n_samples, n_features), optional
        Target relative to X for classification or regression;
        None for unsupervised learning.

    ylim : tuple, shape (ymin, ymax), optional
        Defines minimum and maximum yvalues plotted.

    cv : int, cross-validation generator or an iterable, optional
        Determines the cross-validation splitting strategy.
        Possible inputs for cv are:
          - None, to use the default 3-fold cross-validation,
          - integer, to specify the number of folds.
          - An object to be used as a cross-validation generator.
          - An iterable yielding train/test splits.

        For integer/None inputs, if ``y`` is binary or multiclass,
        :class:`StratifiedKFold` used. If the estimator is not a classifier
        or if ``y`` is neither binary nor multiclass, :class:`KFold` is used.

        Refer :ref:`User Guide <cross_validation>` for the various
        cross-validators that can be used here.

    n_jobs : integer, optional
        Number of jobs to run in parallel (default 1).
    """
    plt.figure()
    plt.title(title)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.xlabel("Training examples")
    plt.ylabel("Score")
    train_sizes, train_scores, test_scores = learning_curve(
        estimator, X, y, cv=cv, n_jobs=n_jobs, train_sizes=train_sizes)
    train_scores_mean = np.mean(train_scores, axis=1)
    train_scores_std = np.std(train_scores, axis=1)
    test_scores_mean = np.mean(test_scores, axis=1)
    test_scores_std = np.std(test_scores, axis=1)
    plt.grid()

    plt.fill_between(train_sizes, train_scores_mean - train_scores_std,
                     train_scores_mean + train_scores_std, alpha=0.1,
                     color="r")
    plt.fill_between(train_sizes, test_scores_mean - test_scores_std,
                     test_scores_mean + test_scores_std, alpha=0.1, color="g")
    plt.plot(train_sizes, train_scores_mean, 'o-', color="r",
             label="Training score")
    plt.plot(train_sizes, test_scores_mean, 'o-', color="g",
             label="Cross-validation score")

    plt.legend(loc="best")
    return plt



def show_values(pc, fmt="%.2f", **kw):
    '''
    Heatmap with text in each cell with matplotlib's pyplot
    Source: https://stackoverflow.com/a/25074150/395857 
    By HYRY
    '''
    #from itertools import izip
    pc.update_scalarmappable()
    ax = pc.get_axes()
    for p, color, value in zip(pc.get_paths(), pc.get_facecolors(), pc.get_array()):
        x, y = p.vertices[:-2, :].mean(0)
        if np.all(color[:3] > 0.5):
            color = (0.0, 0.0, 0.0)
        else:
            color = (1.0, 1.0, 1.0)
        ax.text(x, y, fmt % value, ha="center", va="center", color=color, **kw)


def cm2inch(*tupl):
    '''
    Specify figure size in centimeter in matplotlib
    Source: https://stackoverflow.com/a/22787457/395857
    By gns-ank
    '''
    inch = 2.54
    if type(tupl[0]) == tuple:
        return tuple(i/inch for i in tupl[0])
    else:
        return tuple(i/inch for i in tupl)


def heatmap(AUC, title, xlabel, ylabel, xticklabels, yticklabels, figure_width=40, figure_height=20, correct_orientation=False, cmap='RdBu'):
    '''
    Inspired by:
    - https://stackoverflow.com/a/16124677/395857 
    - https://stackoverflow.com/a/25074150/395857
    '''

    # Plot it out
    fig, ax = plt.subplots()    
    #c = ax.pcolor(AUC, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap='RdBu', vmin=0.0, vmax=1.0)
    c = ax.pcolor(AUC, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap=cmap)

    # put the major ticks at the middle of each cell
    ax.set_yticks(np.arange(AUC.shape[0]) + 0.5, minor=False)
    ax.set_xticks(np.arange(AUC.shape[1]) + 0.5, minor=False)

    # set tick labels
    #ax.set_xticklabels(np.arange(1,AUC.shape[1]+1), minor=False)
    ax.set_xticklabels(xticklabels, minor=False)
    ax.set_yticklabels(yticklabels, minor=False)

    # set title and x/y labels
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)      

    # Remove last blank column
    plt.xlim( (0, AUC.shape[1]) )

    # Turn off all the ticks
    ax = plt.gca()    
    for t in ax.xaxis.get_major_ticks():
        t.tick1On = False
        t.tick2On = False
    for t in ax.yaxis.get_major_ticks():
        t.tick1On = False
        t.tick2On = False

    # Add color bar
    plt.colorbar(c)

    # Add text in each cell 
    show_values(c)

    # Proper orientation (origin at the top left instead of bottom left)
    if correct_orientation:
        ax.invert_yaxis()
        ax.xaxis.tick_top()       

    # resize 
    fig = plt.gcf()
    fig.set_size_inches(cm2inch(figure_width, figure_height))


def plot_classification_report(classification_report, title='Classification report ', cmap='RdBu'):
    '''
    Plot scikit-learn classification report.
    Extension based on https://stackoverflow.com/a/31689645/395857 
    '''
    lines = classification_report.split('\n')

    classes = []
    plotMat = []
    support = []
    class_names = []
    for line in lines[2 : (len(lines) - 2)]:
        t = line.strip().split()
        if len(t) < 2: continue
        classes.append(t[0])
        v = [float(x) for x in t[1: len(t) - 1]]
        support.append(int(t[-1]))
        class_names.append(t[0])
        print(v)
        plotMat.append(v)

    print('plotMat: {0}'.format(plotMat))
    print('support: {0}'.format(support))

    xlabel = 'Metrics'
    ylabel = 'Classes'
    xticklabels = ['Precision', 'Recall', 'F1-score']
    yticklabels = ['{0} ({1})'.format(class_names[idx], sup) for idx, sup  in enumerate(support)]
    figure_width = 25
    figure_height = len(class_names) + 7
    correct_orientation = False
    heatmap(np.array(plotMat), title, xlabel, ylabel, xticklabels, yticklabels, figure_width, figure_height, correct_orientation, cmap=cmap)



if __name__ == '__main__':
    #descriptors = ['test1','test2']
    #descriptors = ['esf','vfh','cvfh','ourcvfh','grsd', 'gshot','spin','egi','good','scurv','sc3D']
    descriptors = ['esf','vfh','cvfh']
    len_category = 10
    score_array = list()
    for i in range(len(descriptors)):
        test_mean = [random.uniform(0,1) for _ in range (len_category)]
        score_array.append(test_mean)

    category_list_name_array = list()
    for i in range(len_category):
        current_category = "Cat " + str(i+1)
        category_list_name_array.append(current_category)
    fig = plot_points(descriptors,category_list_name_array,score_array,"title")
    #plot_bar(descriptors,category_list_name_array,score_array, multiple = True)
 

 '''
def plot_bar(descriptors_array,categories_array,score_array, multiple = True,save = True):
    
    len_category = len(categories_array)

    # create plot
    fig, ax = plt.subplots(figsize = (9,9))
    index = np.arange(len_category)
    if multiple:
        bar_width = 0.05
    else :
        bar_width = 1.5
    opacity = 1.0

    cmap = get_cmap(len(descriptors_array))

    if multiple:
        count = 0
        for i in range(len(descriptors_array)):
            count = count + 1
            current_label = descriptors_array[i]
            rects = plt.bar(index-0.2+bar_width*i, score_array[i], bar_width,
                             alpha=opacity,
                             color=np.random.rand(3,1),
                             label=current_label )
            
            max_score_current = max(score_array[i])
            print("Max Score : {}".format(max_score_current))

    
    else :
        count = 0
        opacity = 1
        for i in range(len(descriptors_array)):

            count = count + 1
            current_label = descriptors_array[i]
            rects = plt.bar(index+bar_width, score_array[i], width=0.5*bar_width,
                             alpha=opacity,
                             color=cmap(i),
                             label=current_label
                             )
            opacity = opacity - 0.05

    plt.xlabel('Categories')
    plt.ylabel('Scores')
    plt.title('Scores by Categories')
    plt.xticks(index + bar_width, categories_array)
    plt.legend()        
     
    plt.tight_layout()
    plt.show()

    return fig
'''
