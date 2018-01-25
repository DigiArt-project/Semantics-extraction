#UI
from tkinter import * 
from PIL import Image, ImageTk
import tkinter.filedialog

#Matplotlib
import matplotlib
#Activate on the server
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.image import AxesImage
import matplotlib.gridspec as gridspec
import matplotlib.ticker as plticker

import utils 


"""
Display the first data to anotate for the classic interactive learning
"""
def display_data_init(dataset, path_list_train_views, number_to_display):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1,figsize=(9, 8))
    fig.canvas.set_window_title('Positive side example')
    # gridspec inside gridspec
    
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)
    for i in range(number_to_display):        
        ax = plt.Subplot(fig, outer_grid[i])
        #print(path_list_train_views[i])
        filename = utils.get_filename_descriptor_from_path(path_list_train_views[i],9)
        #print(filename)
        
        #feature = path_list_train_views[i].decode("utf-8").split("/")[7]
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
Display the first data to anotate, for the similarity search interactive learning
"""
def display_data_init_similarity_search_classic(list_idx,fully_labeled_train_dataset,display_color = True):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1,figsize=(7, 8))
    fig.canvas.set_window_title('Positive side example')
    # gridspec inside gridspec
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)

    count = 0
    for idx,id_value in enumerate(list_idx):
        lb = idealLabels.label(X[id_value])
        ax = plt.Subplot(fig, outer_grid[count])    

        filename = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],9)

        ax.text(0.1, 0.5, str(filename),size=7)
        title = "ID {}".format(id_value)
        if display_color:
            if (lb == 1):
                ax.set_title(title, picker=True, bbox=dict(facecolor='green'))
            else :
                ax.set_title(title, picker=True, bbox=dict(facecolor='red'))
        else :
            ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)

        count = count + 1

    fig.canvas.mpl_connect('pick_event', onpick)
    
    plt.show(block=False)




"""
Display the first data to anotate, for the automatic similarity search interactive learning 
From a list of ID, display the data to the user in order to do the first annotation
Thank to the groundtruth, if an object is positive, it will be displayed in green. Otherwise in red 
"""
def automatic_display_data_init_similarity_search(list_idx,fully_labeled_train_dataset,display_color = True):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1,figsize=(7, 8))
    fig.canvas.set_window_title('Positive side example')
    # gridspec inside gridspec
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)
    if (len(list_idx) > 64 ):
        list_idx = list_idx[:64]

    count = 0
    for idx,id_value in enumerate(list_idx):
        lb = idealLabels.label(X[id_value])
        ax = plt.Subplot(fig, outer_grid[count])    

        filename = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],9)

        ax.text(0.1, 0.5, str(filename),size=7)
        title = "ID {}".format(id_value)
        if display_color:
            if (lb == 1):
                ax.set_title(title, picker=True, bbox=dict(facecolor='green'))
            else :
                ax.set_title(title, picker=True, bbox=dict(facecolor='red'))
        else :
            ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)

        count = count + 1

    fig.canvas.mpl_connect('pick_event', onpick)
    
    plt.show(block=True)


"""
After the first step, display data :
- The most uncertaine one i.e close to the margins
- The positif one i.e the most certains one corresponding to the positif label
"""
def display_selected_data(dataset, path_list_train_views,id_to_display,id_close_margins):
    plt.close(plt.figure(1))
    plt.close(plt.figure(3))
    fig = plt.figure(1,figsize=(7, 8))
    fig1 = plt.figure(3,figsize=(7, 5))
    fig.canvas.set_window_title('Positive side example')
    fig1.canvas.set_window_title('Most uncertain + diversity example')
    labeled_entry_ids, X_pool_labeled = zip(*dataset.get_labeled_entries())
    # gridspec inside gridspec
    
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)
    outer_grid1 = gridspec.GridSpec(4, 8, wspace=1, hspace=0.5)
    #The most certain one
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        ax = plt.Subplot(fig, outer_grid[i])

        filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

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
            ax = plt.Subplot(fig1, outer_grid1[i])
       
            filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

            ax.text(0.1, 0.5, str(filename),
                            size=7)
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
    fig = plt.figure(1,figsize=(7, 8))
    fig1 = plt.figure(3,figsize=(7, 5))
    fig.canvas.set_window_title('Positive side example')
    fig1.canvas.set_window_title('Most uncertain + diversity example')
    labeled_entry_ids, X_pool_labeled = zip(*train_dataset.get_labeled_entries())
    X, y = zip(*train_dataset.data)
    nb_total_positif_annotated = y.count(1)
    nb_total_negatif_annotated = y.count(-1)

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    # gridspec inside gridspec

    count_positif = 0
    count_negatif = 0
    
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)
    outer_grid1 = gridspec.GridSpec(4, 8, wspace=1, hspace=0.5)
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        ax = plt.Subplot(fig, outer_grid[i])

        filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

        lb = idealLabels.label(X[ask_id])

        ax.text(0.1, 0.5, str(filename),size=7)
        title = "ID {}".format(ask_id)
        if display_color:
            if (lb == 1):
                ax.set_title(title, picker=True, bbox=dict(facecolor='green'))
                count_positif = count_positif + 1
            else :
                ax.set_title(title, picker=True, bbox=dict(facecolor='red'))
                count_negatif = count_negatif + 1
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

    
    #print(np.array(labeled_entry_ids))
    for i in range(len(id_close_margins)):
        ask_id = id_close_margins[i]
        if not ask_id in np.array(labeled_entry_ids):
            ax = plt.Subplot(fig1, outer_grid1[i])
       
            filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

            lb = idealLabels.label(X[ask_id])
            ax.text(0.1, 0.5, str(filename),
                            size=7)
            title = "ID {}".format(ask_id)
            if display_color:
                if (lb == 1):
                    ax.set_title(title, picker=True, bbox=dict(facecolor='green'))
                else :
                    ax.set_title(title, picker=True, bbox=dict(facecolor='red'))
            else :
                ax.set_title(title, picker=True, bbox=dict(facecolor='blue'))
            ax.set_xticks([])
            ax.set_yticks([])
            fig1.add_subplot(ax)
    title = "Samples where the distance is close to the decision boundary (most uncertain one + diversity) "
    fig1.suptitle(title, fontsize=8, fontweight='bold')

    
    fig.canvas.mpl_connect('pick_event', onpick)

    if (save_figures):
        fig.savefig('results.png', bbox_inches='tight')

    pourcent_positif_display = (count_positif/nbre_data_selection) * 100
    pourcent_negatif_display = (count_negatif/nbre_data_selection) * 100

    fig1.canvas.mpl_connect('pick_event', onpick)
    plt.show(block=False)

    return pourcent_positif_display,pourcent_negatif_display



