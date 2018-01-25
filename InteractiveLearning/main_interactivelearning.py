#!/usr/bin/env python3 -W ignore::DeprecationWarning
"""
The script helps guide the users to quickly understand how to use
libact by going through a simple active learning task with clear
descriptions.
"""


#Matplotlib
import matplotlib
#Activate on the server or when using multi thread
matplotlib.use('Agg')


import utils 
from importing_modules import *
import plotting



###### GLOBAL VARIABLE #######

path_dataset = "/Users/lironesamoun/digiArt/Datasets/Dataset_cat10_normalized/"
training_file_full = path_dataset + "train_full.txt"
testing_file_full =  path_dataset + "test_full.txt"
training_file_views =  path_dataset +"train_views.txt"
testing_file_views =  path_dataset +"test_views.txt"
#Perform interative learning on full or views object
compute_full = "false"
#Array of descriptors we want to test
descriptors = ['esf','pointnet']
name_dataset = 'cat10_views'
#Location of the dataset directory where the txt file are located. The txt represents the list of all the descriptors and path to descriptors
root_data = "data/"
#View path file
view_path_file = "data/cat10_views/dataset_descriptor_esf.txt"
#descriptor path file
dataset_descriptor = "data/cat10_views/descriptors_esf.txt"


#Where to save the graphics
output_graphicals_result = "results_graphics/"
#Log file
log_file = "interactive_learning_log.txt"

#Category label to change for option automatic interactive learning
category_label = 3
#Positive example ranking for display i.e the number of samples ranked from positive to negative, to display to the user 
nbre_data_selection = 20 #defaut 48
#Number of uncertain examples close to the margin to choose 
nbre_data_margins_selection_uncertainty = 30
#Among the uncertain choosen, this is the number of uncertains to display to the users
nbre_data_margins_selection_uncertainty_display = 7
#Number of example the most uncertain display to the user for selection after diversity
nbre_data_final_selection = 7
# the percentage of samples in the dataset that will be randomly selected and assigned to the test set
test_size = 0.35
#SVM parameters
C = 100 
gam = 0.001
kernel_svm = "rbf" #rbf, linear, poly, sigmoid

    
#Defaut number of k neighbors when using similarity search
k = 20
#Leaf resolution
leaf_resolution = 0.01
#Defaut descriptor to use for similarity search (only VFH, ESF, CVFH and OURCVFH are available) 
descriptor_similaritySearch = "esf"
#Number samples to select at the first step (if number = 6, will take the first three and the last three)
number_to_annotate_first_step = 6
#Number uncertain samples to select at the second step (if = 5, will take the first 5 uncertains)
number_to_annotate_second_step = 5
#Number iterations max of interactive learning step i.e, number of steps the user want to select uncertain sample
nb_iterations_max = 20
#Number of experiment to run of each interactive learning session. 1 is okay, more will take more time
repetition_experiment = 1
#Number of top result for pourcentage computation
nb_max_positif_display = 10
#The number of objects we select randomly for each category inside the dataset
number_object_to_select_randomly = 3
#Either select full objects or views object for selecting randomly
select_full_object = True

query_cloud = ""


_item_click_callback= set()
_item_all_label_data = list()

use_norm = True


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
        print('id selected:', id_extract[0])
        _item_click_callback.add(id_extract[0])


"""
Display the first data to anotate for the classic interactive learning
"""
def display_data_init(dataset, path_list_train_views, number_to_display):
    if 'fig' in locals():
        plt.close(fig)
    fig = plt.figure(1,figsize=(9, 8))
    fig.canvas.set_window_title('Results')
    # gridspec inside gridspec
    outer_grid = gridspec.GridSpec(8, 8, wspace=1, hspace=0.5)
    for i in range(number_to_display):        
        ax = plt.Subplot(fig, outer_grid[i])
        filename = utils.get_filename_descriptor_from_path(path_list_train_views[i],9)
        
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

        filename = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],9)

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
Display the first data to anotate, for the automatic similarity search interactive learning 
From a list of ID, display the data to the user in order to do the first annotation
Thank to the groundtruth, if an object is positive, it will be displayed in green. Otherwise in red 
"""
def automatic_display_data_init_similarity_search(list_idx,fully_labeled_train_dataset,display_color = True):
    if 'fig' in locals():
        plt.close(fig)
    #fig = plt.figure(1,figsize=(11, 7)) defaut
    fig = plt.figure(1,figsize=(9, 7))
    fig.canvas.set_window_title('Results')
    # gridspec inside gridspec
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    #outer_grid = gridspec.GridSpec(8, 6, wspace=1, hspace=0.8)
    outer_grid = gridspec.GridSpec(5, 6, wspace=1, hspace=0.8)
    if (len(list_idx) > 48 ):
        list_idx = list_idx[:48]
    count = 0
    for idx,id_value in enumerate(list_idx):
        lb = idealLabels.label(X[id_value])
        ax = plt.Subplot(fig, outer_grid[count])    

        filename = utils.get_filename_descriptor_from_path(fully_labeled_train_dataset.data[id_value][0][1],9)

        ax.text(0.1, 0.4, str(filename),size=8)
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
    
    plt.show(block=True)


"""
Optimize with dictionnary
"""
def split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file,testing_file,categorie_array_results,category_label):

    global compute_full

    print("[INFO] Dataset filepath : {}".format(dataset_filepath))
    logging.debug('Dataset filepath : %s', dataset_filepath)
    X, y = import_libsvm_sparse(dataset_filepath).format_sklearn()
    X = MinMaxScaler().fit_transform(X)

    test = ""

    with open(view_path_file, "r") as ins:
        array = np.chararray(len(X),itemsize="1500")
        test_list = list()
        i = 0
        for line in ins:
            #print(line)
            if "ply" in line or "pcd" in line or "txt" in line: 
                array[i]=str(line.strip())
                test_list.append(str(line.strip()))
                test = line.strip()
                i = i + 1

    descripteur_and_path_order = OrderedDict(zip(test_list, X))
    X_dataset_with_path = list(zip(X,  test_list))

    fully_dataset = Dataset(X_dataset_with_path, y)


    y_binary = [1 if y_v == int(category_label) else -1 for y_v in y]
    number_positif_label = y_binary.count(1)
    number_negatif_label = y_binary.count(-1)
    print("[INFO]Number positif from libsvm data : {}".format(number_positif_label))
    print("[INFO] Number negatif from libsvm data : {}".format(number_negatif_label))

    fully_dataset_labeled_binary = Dataset(X_dataset_with_path, y_binary)

    X_train = list()
    X_test = list()
    y_train = list()
    y_test = list()


    X_dataset_with_path_dict=OrderedDict()
    for idx,value in enumerate(X_dataset_with_path):
        base1=os.path.basename(value[1])
        name_X_desc = os.path.splitext(base1)[0]

        #for desc_name take only name
        if not compute_full == "true":
                name_X_desc = name_X_desc.replace('view_','')
        name_X_desc = name_X_desc.replace('desc_','')
        print(name_X_desc)
        X_dataset_with_path_dict[name_X_desc] = X_dataset_with_path[idx][0]

    values = list(X_dataset_with_path_dict.values())
    keys = list(X_dataset_with_path_dict.keys())

    #training file
    with open(training_file, "r") as trfile:
        for line in trfile:
            base=os.path.basename(line)
            name_train = os.path.splitext(base)[0]
            if not compute_full == "true":
                name_train = name_train.replace('view_','')

            if (name_train in X_dataset_with_path_dict):

               
                index = list(X_dataset_with_path_dict.keys()).index(name_train)
                X_value = X_dataset_with_path[index]
                label = y[index]
                X_train.append(X_value)
                y_train.append(label)

    X_train = np.asarray(X_train)
    y_train = np.asarray(y_train)

    #testing file
    with open(testing_file, "r") as trfile:
        for line in trfile:
            base=os.path.basename(line)
            name_test = os.path.splitext(base)[0]
            if not compute_full == "true":
                name_test = name_test.replace('view_','')

            if (name_test in X_dataset_with_path_dict):

                
                index = list(X_dataset_with_path_dict.keys()).index(name_test)
                #X_value = X_dataset_with_path_dict[name_test]
                X_value = X_dataset_with_path[index]
                label = y[index]
                X_test.append(X_value)
                y_test.append(label)

    print(len(X_dataset_with_path))
    print(len(X_train))
    print(len(y_train))
    print(len(X_test))
    print(len(y_test))


    y_train_binary = [1 if y == int(category_label) else -1 for y in y_train]
    y_train_binary = np.array(y_train_binary)
    number_positif_label_train = np.count_nonzero(y_train_binary == 1)
    number_negatif_label_train = np.count_nonzero(y_train_binary == -1)


    train_dataset_binary_labeled = Dataset(X_train, y_train_binary)


    path_list_train_views = list()
    feature_train = list()
    for i in X_train:
        path_list_train_views.append(i[1])
        feature_train.append(i[0])

    path_list_test_views = list()
    feature_test = list()
    for i in X_test:
        path_list_test_views.append(i[1])
        feature_test.append(i[0])

    feature_test = np.array(feature_test)

    y_test_binary = [1 if y == int(category_label) else -1 for y in y_test]
    y_test_binary = np.array(y_test_binary)
    test_dataset_binary_labeled = Dataset(feature_test, y_test_binary)

    y_train_unlabeled = np.concatenate([y_train[:0], [None] * (len(y_train))])
    train_dataset_unlabeled = Dataset(feature_train, y_train_unlabeled)
    
    #HAs been changed
    #fully_labeled_trn_ds = Dataset(X_train, y_train)
    test_dataset_GT = Dataset(feature_test, y_test)

    #return unlabeled train dataset, test dataset, y label train, labeled train dataset, path to views training, path to views testing
    return fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary,test_dataset_binary_labeled, path_list_train_views,path_list_test_views


"""
From a file of descriptors and another file which contains its path to views, create the database
The difference with the other function is the fact that we get the category label and therefore the groundtruth. We change the multicalss problem to a binary one where
there is one positif label and the others are all set to negatif
At the end we have 
- the fully_dataset_labeled_binary object : contains features X + corresponding path, and its label binary (+1 or -1)
- train_dataset_unlabeled object : contains the train part Features X + label unknown
- test_dataset_binary_labeled : features X test + label y test binary
- y_train_binary : label from the train set binary
- train_dataset_binary_labeled object : fully dataset features X + label y binary
"""
def split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size_split,categorie_array_results,category_label):

    logging.debug('Dataset filepath : %s', dataset_filepath)
    X, y = import_libsvm_sparse(dataset_filepath).format_sklearn()
    X = MinMaxScaler().fit_transform(X)

    test = ""
    with open(view_path_file, "r") as ins:
        array = np.chararray(len(X),itemsize="1500")
        test_list = list()
        i = 0
        for line in ins:
            if "ply" in line or "pcd" in line or "txt" in line: 
                array[i]=str(line.strip())
                test_list.append(str(line.strip()))
                test = line.strip()
                i = i + 1

    descripteur_and_path_order = OrderedDict(zip(test_list, X))
    X_dataset_with_path = list(zip(X,  test_list))

    fully_dataset = Dataset(X_dataset_with_path, y)


    y_binary = [1 if y_v == int(category_label) else -1 for y_v in y]
    number_positif_label = y_binary.count(1)
    number_negatif_label = y_binary.count(-1)
    print("[INFO]Number positif from libsvm data : {}".format(number_positif_label))
    print("[INFO]Number negatif from libsvm data : {}".format(number_negatif_label))

    fully_dataset_labeled_binary = Dataset(X_dataset_with_path, y_binary)

    X_train,X_test,y_train,y_test = utils.train_test_split_according_to_categories(X_dataset_with_path, y,categorie_array_results,test_size_split)

    y_train_binary = [1 if y == int(category_label) else -1 for y in y_train]
    y_train_binary = np.array(y_train_binary)
    number_positif_label_train = np.count_nonzero(y_train_binary == 1)
    number_negatif_label_train = np.count_nonzero(y_train_binary == -1)

    while (number_positif_label_train == 0 or number_negatif_label_train == 0 ):
        X_train,X_test,y_train,y_test = utils.train_test_split_according_to_categories(X_dataset_with_path, y,categorie_array_results)
        y_train_binary = [1 if y == int(category_label) else -1 for y in y_train]
        y_train_binary = np.array(y_train_binary)
        number_positif_label_train = np.count_nonzero(y_train_binary == 1)
        number_negatif_label_train = np.count_nonzero(y_train_binary == -1)


    train_dataset_binary_labeled = Dataset(X_train, y_train_binary)


    path_list_train_views = list()
    feature_train = list()
    for i in X_train:
        path_list_train_views.append(i[1])
        feature_train.append(i[0])

    path_list_test_views = list()
    feature_test = list()
    for i in X_test:
        path_list_test_views.append(i[1])
        feature_test.append(i[0])

    feature_test = np.array(feature_test)

    y_test_binary = [1 if y == int(category_label) else -1 for y in y_test]
    y_test_binary = np.array(y_test_binary)
    test_dataset_binary_labeled = Dataset(feature_test, y_test_binary)

    y_train_unlabeled = np.concatenate([y_train[:0], [None] * (len(y_train))])
    train_dataset_unlabeled = Dataset(feature_train, y_train_unlabeled)
    
    #HAs been changed
    test_dataset_GT = Dataset(feature_test, y_test)

    return fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary,test_dataset_binary_labeled, path_list_train_views,path_list_test_views


"""
After the first step, display data :
- The most uncertaine one i.e close to the margins
- The positif one i.e the most certains one corresponding to the positif label
"""
def display_selected_data(dataset, path_list_train_views,id_to_display,id_close_margins):
    plt.close(plt.figure(1))
    plt.close(plt.figure(3))
    fig = plt.figure(1,figsize=(9, 8))
    fig1 = plt.figure(3,figsize=(7, 5))
    fig.canvas.set_window_title('Results')
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
    #fig = plt.figure(1,figsize=(11, 10))
    #fig1 = plt.figure(3,figsize=(11, 5))
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
    # gridspec inside gridspec

    count_positif = 0
    count_negatif = 0
    
    #outer_grid = gridspec.GridSpec(9, 6, wspace = 1, hspace=1)
    #outer_grid1 = gridspec.GridSpec(4, 6, wspace=1, hspace=1)
    outer_grid = gridspec.GridSpec(5, 6, wspace = 1, hspace=1)
    outer_grid1 = gridspec.GridSpec(5, 6, wspace=1, hspace=1)
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        ax = plt.Subplot(fig, outer_grid[i])

        filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

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
       
            filename = utils.get_filename_descriptor_from_path(path_list_train_views[ask_id],9)

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

    pourcent_positif_display = (count_positif/nbre_data_selection) * 100
    pourcent_negatif_display = (count_negatif/nbre_data_selection) * 100

    fig1.canvas.mpl_connect('pick_event', onpick)
    plt.show(block=False)

    return pourcent_positif_display,pourcent_negatif_display



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
        _item_all_label_data.append((id_l,label))
    for id_l,label in id_selected_negatif:
        logging.debug('ID : %s corresponds to label : %s  ', id_l, label)
        list2.append((id_l,label))
        whole_dataset.update(int(id_l), label)
        _item_all_label_data.append((id_l,label))
    total_annotated_id_list = list1 + list2

    logging.debug('[INFO] ====== All label data %s  ', _item_all_label_data)
    logging.debug('Update Nbr unlabeled: %s  ', whole_dataset.len_unlabeled())
    logging.debug('Update Nbr labeled: %s  ', whole_dataset.len_labeled())

    return total_annotated_id_list

"""
Utility function. Given A dataset object, print informations of the dataset
"""
def check_info_database(database):
    unlabeled_entry_ids, X_pool_unlabeled = zip(*database.get_unlabeled_entries())
    if database.len_labeled() > 0 :
        labeled_entry_ids, X_pool_labeled = zip(*database.get_labeled_entries())
        logging.debug('Entries labled data: %s  ', labeled_entry_ids)
    entry_id,all_xpool = zip(*[(idx, entry[0]) for idx, entry in enumerate(database.data)])
    logging.debug('Entries unlabled data : %s  ', unlabeled_entry_ids)
    logging.debug('Nbr unlabeled : %s  ', database.len_unlabeled())
    logging.debug('Nbr labeled : %s  ', database.len_labeled())



"""
Discounted cumulative gain (DCG) at rank K.
"""
def dcg_score(y_true, y_score, k=5):
    """Discounted cumulative gain (DCG) at rank K.
    Parameters
    ----------
    y_true : array, shape = [n_samples]
        Ground truth (true relevance labels).
    y_score : array, shape = [n_samples]
        Predicted scores.
    k : int
        Rank.
    Returns
    -------
    score : float
    References
    ----------
    .. [1] `Wikipedia entry for the Discounted Cumulative Gain
           <https://en.wikipedia.org/wiki/Discounted_cumulative_gain>`_
    """
    y_score = [[0.15, 0.55, 0.20], [0.7, 0.2, 0.1], [0.06, 0.04, 0.9]]
    order = np.argsort(y_score)[::-1]
    y_true = np.take(y_true, order[:k])

    gain = 2 ** y_true - 1

    discounts = np.log2(np.arange(len(y_true)) + 2)
    return np.sum(gain / discounts)

"""
Normalized discounted cumulative gain (NDCG) at rank K.
"""
def ndcg_score(y_true, y_score, k=5):
    
    y_score, y_true = check_X_y(y_score, y_true)

    # Make sure we use all the labels (max between the lenght and the higher
    # number in the array)
    lb = LabelBinarizer()
    lb.fit(np.arange(max(np.max(y_true) + 1, len(y_true))))
    binarized_y_true = lb.transform(y_true)

    if binarized_y_true.shape != y_score.shape:
        raise ValueError("y_true and y_score have different value ranges")

    scores = []

    # Iterate over each y_value_true and compute the DCG score
    for y_value_true, y_value_score in zip(binarized_y_true, y_score):
        actual = dcg_score(y_value_true, y_value_score, k)
        print("DCG : {}" .format(actual))
        best = dcg_score(y_value_true, y_value_true, k)
        scores.append(actual / best)

    return np.mean(scores)



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

    abs_kernel = fabs(simple_kernel_i_j)

    angle_kernel_result = (abs_kernel/ math.sqrt(simple_kernel_i_i * simple_kernel_j_j) )

    return angle_kernel_result



"""
Replicate the decision function of scikit : clf.decision_function(X)
Give the decision function of a specific point X
"""
def decision_function_point(params, sv, nv, a, b, X):
    # calculate the kernels
    k = kernel(params, sv, X)

    # define the start and end index for support vectors for each class
    start = [sum(nv[:i]) for i in range(len(nv))]
    end = [start[i] + nv[i] for i in range(len(nv))]

    # calculate: sum(a_p * k(x_p, x)) between every 2 classes
    c = [ sum(a[ i ][p] * k[p] for p in range(start[j], end[j])) +
          sum(a[j-1][p] * k[p] for p in range(start[i], end[i]))
                for i in range(len(nv)) for j in range(i+1,len(nv))]

    # add the intercept
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
        # calculate the kernels
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
def predict(params, sv, nv, a, b, cs, X):
    ''' params = model parameters
        sv = support vectors
        nv = # of support vectors per class
        a  = dual coefficients
        b  = intercepts 
        cs = list of class names
        X  = feature to predict       
    '''
    decision = decision_function(params, sv, nv, a, b, X)
    votes = [(i if decision[p] > 0 else j) for p,(i,j) in enumerate((i,j) 
                                           for i in range(len(cs))
                                           for j in range(i+1,len(cs)))]

    return cs[max(set(votes), key=votes.count)]

"""
Replicate the predict function of scikit :  clf.predict(X)
Give the prediction value of a given vector of points
"""
def predict_vector(params, sv, nv, a, b, cs, X):
    ''' params = model parameters
        sv = support vectors
        nv = # of support vectors per class
        a  = dual coefficients
        b  = intercepts 
        cs = list of class names
        X  = feature to predict       
    '''
    result_prediction = list()
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
        params = model.get_params()
        #Support vectors.
        sv = model.support_vectors
        #Number of support vectors for each class.
        nv = model.n_support_
        #Coefficients of the support vector in the decision function
        a  = model.dual_coef_
        #Constants in decision function.
        b  = model._intercept_
        #List of class name
        cs = model.classes_
        
        return params, sv, a, b, cs

    else :
        logging.warning('getparameters_from_models ==> instance is not SVC')


"""
count the number of positive label and negative label displayed to the user
"""
def compute_pourcentage_results(fully_labeled_train_dataset,train_dataset,id_to_display, number_to_consider = 20):

    if (number_to_consider > len(id_to_display)):
        number_to_consider = len(id_to_display) - 1

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count_positif = 0
    count_negatif = 0
    count_total = 0
    #Count posiitve and negative display to the user
    logging.debug("ID to daisplya : %s ",id_to_display)
    for i in range(len(id_to_display)):
        ask_id = id_to_display[i]
        #print("ASK ID {}".format(ask_id))
        lb = idealLabels.label(X[ask_id])
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

    logging.debug("Compute pourcentage result. Count positif : %s Count negatif %s | number to consider %s ",count_positif,count_negatif, number_to_consider)
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
def select_queries_uncertainty_decision_function(dataset, model_learning,m, correction_to_apply = 0):
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    features_all = np.array(features_all)
    idx_unlabeled_data, X_pool_unlabeled = zip(*dataset.get_unlabeled_entries())
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
def select_queries_uncertaintysampling(dataset, probabilities,m,return_score=False):
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
def feedback(whole_dataset,current_rank,model_learning, annotated_data_selection):
    #Have the score of the selected example
    logging.debug('Previous Rank : %s  ', current_rank)

    h = 0
    new_rank = 0

    idx_unlabeled_data, X_pool_unlabeled = zip(*whole_dataset.get_unlabeled_entries())
    X_pool_labeled, labels = zip(*whole_dataset.get_labeled_entries())
    decision_function_unlabeled = model_learning.decision_function(X_pool_unlabeled)

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
def training_data(training_dataset,model):
    model.fit(*(training_dataset.format_sklearn()))

"""
Step 3 Interactive Learning
Correction
"""
def correction(dataset,model,rank):
    label_all,features_all = zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    features_all = np.array(features_all)

    idx_unlabeled_data, X_pool_unlabeled = zip(*dataset.get_unlabeled_entries())
    decision_function_unlabeled = model.decision_function(X_pool_unlabeled)    
    decision_func_all_data = model.decision_function(features_all)

    if (use_norm):
        decision_function_unlabeled = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function_unlabeled)
        decision_func_all_data = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_func_all_data)

    #Sort decision function
    indices_rank_decision_function_positif_unlabeled, distance_rank_decision_function_positif_unlabeled = utils.sort_descending_order(decision_func_all_data)
    #print("\n Correction : len decision function : {} ".format(len(decision_func_all_data)))
    try:
        v = indices_rank_decision_function_positif_unlabeled[rank]
    except IndexError:
        rank = 0

    number_to_remove = distance_rank_decision_function_positif_unlabeled[rank]

    decision_function_unlabeled = [x + number_to_remove for x in decision_function_unlabeled]
    #print("\n [INFO] NEW DECISON FUNCTION : {} ".format(decision_function_unlabeled))
    return decision_function_unlabeled,number_to_remove

"""
Step 4 Interactive Learning
Preselection
"""
def preselection(dataset,decision_function, m):
    abs_decision_function = [fabs(number) for number in decision_function]
    tmp = copy.copy(abs_decision_function)
    sorted_indice = (np.array(abs_decision_function).argsort()[::-1])[:m]
    sorted_values = sorted(tmp,reverse = True)[:m]
    
    return sorted_indice, sorted_values

"""
Step 5 Interactive Learning
Selection
Consider the subset of images close to the boundary and then use a criterion related to Average precision to compute the cost g
"""
def selectioner(dataset,model_learning,kernel,y_train_gt, decision_function, pre_selection_data):
    feature_labeled,labeled = zip(*dataset.get_labeled_entries())
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])

    g = []
    distance_frontiere_abs = [fabs(number) for number in decision_function]
    decision_function_all = model_learning.decision_function(features_all)
    distance_frontiere_abs_all = [fabs(number) for number in decision_function_all]

    #Loop over all the uncertain example which have been chose
    for idx, value_id in enumerate(pre_selection_data):
       
        X = features_all[value_id]
        prediction = model_learning.predict(X.reshape(1, -1))

        if prediction[0] == -1 :
            prediction[0] = 1
            
        y_value_gt = [y_train_gt[value_id]]
        
        abs_decision_function_X = distance_frontiere_abs_all[value_id]
        #decision_function_X = model_learning.decision_function(X.reshape(1, -1))
        #abs_decision_function_X = fabs(decision_function_X)

        g_current = abs_decision_function_X * (1 - precision_score(y_value_gt,prediction))
        g.insert(idx,g_current)
        #print("Current g: {}".format(g_current))

    logging.debug('Final G : %s  ', len(g))

    return g

"""
Step 5 Interactive Learning
Diversity
https://www.aaai.org/Papers/ICML/2003/ICML03-011.pdf
"""
def diversifier(dataset,param_adjust,batch_number,g,params,pre_selection_data):
    kernel = params['kernel']
    gamma = params['gamma']
    selection = list()
    idx_all, features_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(dataset.data)])
    labeled_data = np.array(_item_all_label_data)
    idx_label_data = labeled_data[:,0]

    for q in range(0,batch_number):
        result = list()
        for i, value_id_i in enumerate(pre_selection_data):
            similarities_i_j = list()
            feature_selected_i = features_all[value_id_i]
            g_current_selected_i = g[i]

            for j, value_id_j in enumerate(selection):
                feature_selected_j = features_all[value_id_j]
                #Compute angle diversity
                similarity_measure = angle_kernel(kernel,gamma,feature_selected_i,feature_selected_j)
                similarities_i_j.append(similarity_measure)

            if similarities_i_j:
                max_similarity_measure_i_j = max(similarities_i_j)
            else :
                max_similarity_measure_i_j = 0

            
            result_diversity = param_adjust* fabs(g_current_selected_i) + (1 - param_adjust) * max_similarity_measure_i_j
            result.append(result_diversity)
            
        indice_select = np.argmin(result)
        id_selected =  pre_selection_data[indice_select]
        #Remove the concerned id
        #del pre_selection_data[select]
        selection.append(id_selected)
        pre_selection_data = np.delete(pre_selection_data, indice_select)

    logging.debug('[INFO] Final selection id : %s  ', selection)
    
    return selection


"""
At the beginning we need to match the results given by the similarity search with the dataset of unlabeled data.
So from the similarity search, (from the json) we get an array of categories 
Then in the fully labeled dataset, we make a loop, extract the path of each object, extract the category and try to match with the categories in the array
At the end we have a list of ID which will be the first results to display to the user in order to annotate
"""
def select_data_similaritySearch(fully_labeled_train_dataset,categorie_array_results):
    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)
    count = 0
    list_idx_result = list()

    for idx1,path1 in enumerate(categorie_array_results[:20]):
        for idx2, path2 in enumerate(fully_labeled_train_dataset.data):
            filename = os.path.basename(path2[0][1])

            #The initial name is desc_category.pcd. We keep only the part after desc_
            filename = filename.split('_',1)[1]
            #Remove extension
            filename = os.path.splitext(filename)[0]
    
            if (path1 == filename):
                #print(filename)
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


def compute_scores_SVM(X_test,y_test,y_pred,model_learning,plot_PR_curve = False,plot_cnf_matrix = False, plot_clf_report = False, save_figure = False):
    y_true = y_test
    ### F1 SCORE ###
    #The f1-score gives you the harmonic mean of precision and recall. 
    #The scores corresponding to every class will tell you the accuracy of the classifier 
    #in classifying the data points in that particular class compared to all other classes.
    #The support is the number of samples of the true response that lie in that class.
    f1score = f1_score(y_true, y_pred, average='weighted')  

    ### Average Precision SCORE ###
    AP = average_precision_score(y_true, y_pred) 

    ### Precision recall curve ###
    precision_curve, recall_curve, thresholds = precision_recall_curve(y_true, y_pred)
    recall = recall_score(y_true, y_pred, average = 'weighted')  
    precision = precision_score(y_true, y_pred, average = 'weighted')

    ### Classification report ###
    report_classification = classification_report(y_true, y_pred)

    ### Confusion matrix ###
    cnf_matrix = confusion_matrix(y_test, y_pred)
    np.set_printoptions(precision=2)


    logging.info("==> F1 score : %s",f1score)
    logging.info("==> Precision : %s",precision)
    logging.info("==> Recall : %s",recall)
    logging.info("==> Average Precision score : %s",AP)
    logging.info("==> Classification report : %s",report_classification)
    logging.info("==> Cconfusion_matrix : %s",cnf_matrix)


    return f1score,AP,precision,recall,precision_curve,recall_curve,report_classification,cnf_matrix


"""
Interactive learning without similarity search. Basic test
"""
def basic_interactiveLearning3DStart():
    E_in, E_out, rank_array = [], [], []
    nb_iterations = 1
    

    dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), dataset_descriptor)
    fully_dataset,train_dataset, X_test,y_test, y_train_gt, fully_labeled_train_dataset,path_list_train_views,path_list_test_views = utils.split_train_test_from_libsvm_data(dataset_filepath,view_path_file, test_size)
    unlabeled_entry_list = list(train_dataset.get_unlabeled_entries())

    #Initialization rank
    rank = len(unlabeled_entry_list)/2

    print("[INFO] Initial rank : {} ".format(rank))
    print("\n=====> Itération : {} ".format(nb_iterations))
    logging.info('\n=====> Itération : %s  ', nb_iterations)


    ################ FIRST PLOT ###################
    start = time.time()
    display_data_init(unlabeled_entry_list,path_list_train_views,nbre_data_selection)
    stop = time.time()
    print("Displya data init - time : {} s".format(stop - start))
    test_dataset,X_test_new,y_test_new,category_label = utils.get_testDataset_binary(X_test,y_test,view_path_file)

    y_train_binary_GT = utils.get_train_label_binary(y_train_gt,category_label)
    #Annotate first data
    id_unlabeled_display_first=[id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:nbre_data_selection])]
    total_annotated_id_list = annotate_data(train_dataset,id_unlabeled_display_first)
    
    ################ FIRST TRAINING ###################
    model_learning = svm.SVC(kernel=kernel_svm,C=C, gamma = gam, class_weight = 'balanced',probability = True)

    training_data(train_dataset,model_learning)


    #Get parameters from the svm model
    params = model_learning.get_params()
    sv = model_learning.support_vectors_
    nv = model_learning.n_support_
    a  = model_learning.dual_coef_
    b  = model_learning._intercept_
    cs = model_learning.classes_

    #Get the id and features from unlabeled data
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset.get_unlabeled_entries())
    #Get the id and features from all the data
    idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset.data)])

    #Get the probabilities result and decision function of all the data
    probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
    ##HAS BEEN CHANGED
    decision_function = model_learning.decision_function(X_pool_unlabeled)

    ################ GRAPHIC ###################
    #Add the score to the chart
    score = model_learning.score(*(test_dataset.format_sklearn()))
    E_out = np.append(E_out, 1 - score)
    E_in = np.append(E_in,score)
    score_ok = model_learning.score(*(test_dataset.format_sklearn()))*100
    score_error = (1 - model_learning.score(*(test_dataset.format_sklearn())))*100
    #title = "Score Success : " + str(score_ok)  + " Score Error : " + str(score_error) + " \n Rank : " + str(rank)
    rank_array = np.append(rank_array, rank)

    query_num = np.arange(0, 1)
    fig = plt.figure(figsize=(7, 8))
    ax = fig.add_subplot(2, 1, 1)
    p1, = ax.plot(query_num, E_out, 'r', label='Error')
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
  

    plt.show(block = False)
 
    if (use_norm):
        decision_function = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)


    #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
    indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function,False)
    idx_example_positif_selection = np.take(idx_unlabeled_data,indices_ranking_descending_order)[:nbre_data_selection]


    ################ SELECTION UNCERTAINTY ###################
    #ids_example_close_margin,_ = select_queries_uncertaintysampling(train_dataset, probabilities_samples_unlabeled,nbre_data_margins_selection_uncertainty,True)
    start = time.time()
    ids_example_close_margin = select_queries_uncertainty_decision_function(train_dataset, model_learning, nbre_data_margins_selection_uncertainty)
    stop = time.time()
    print("Select queries uncertainty decision - time : {} s".format(stop - start))
    total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))
    display_selected_data(train_dataset,path_list_train_views,idx_example_positif_selection, ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display])

    ###########=====>>>> SECOND STEP UPDATE  ###########
    total_annotated_id_list = annotate_data(train_dataset,total_id_annotation)
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset.get_unlabeled_entries())
    decision_function = model_learning.decision_function(X_pool_unlabeled)
    decision_function_custom = decision_function_vector(params, sv, nv, a, b, X_pool_unlabeled)

    decision_func = copy.copy(decision_function)
    i = 0
    while True :
        nb_iterations = nb_iterations + 1
        print("\n=====>[INFO] Itération : {} ".format(nb_iterations))
        startStep2 = time.time()

        ################ 1. FEEDBACK ###################
        rank = feedback(train_dataset,rank,model_learning, total_annotated_id_list)
 
        training_data(train_dataset,model_learning)

        # Get parameters from model
        params = model_learning.get_params()
        sv = model_learning.support_vectors_
        nv = model_learning.n_support_
        a  = model_learning.dual_coef_
        b  = model_learning._intercept_
        cs = model_learning.classes_

        # Add score to the chart
        score = model_learning.score(*(test_dataset.format_sklearn()))
        E_out = np.append(E_out, 1 - score)
        E_in = np.append(E_in, score)
        score_ok = model_learning.score(*(test_dataset.format_sklearn()))*100
        score_error = (1 - model_learning.score(*(test_dataset.format_sklearn())))*100
        rank_array = np.append(rank_array, rank)

        ax.set_xlim((0, i + 1))
        ax.set_ylim((0, max(E_out) + 0.2))
        ay.set_xlim((0, i + 1))
        ay.set_ylim((0, max(E_in) + 0.2))

        query_num = np.arange(0, i + 2)

        p1.set_xdata(query_num)
        p1.set_ydata(E_out)
        p2.set_xdata(query_num)
        p2.set_ydata(E_in)


        plt.show(block = False)

        idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset.get_unlabeled_entries())
        idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset.data)])
        probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
        decision_function = model_learning.decision_function(X_pool_unlabeled)

        indices_rank_decision_function_positif_without_correction, _ = utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection_without_correction = np.take(idx_unlabeled_data,indices_rank_decision_function_positif_without_correction)[:nbre_data_selection]

        ################ 2. CORRECTION ###################
        decision_function, number_to_remove = correction(train_dataset, model_learning, rank)
        indices_rank_new_decision_function_positif, distance_rank_decision_function_positif = utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection = np.take(idx_unlabeled_data,indices_rank_new_decision_function_positif)[:nbre_data_selection]

        ################ 3. PRESELECTION ###################
        ids_example_close_margin_correction = select_queries_uncertainty_decision_function(train_dataset, model_learning, nbre_data_margins_selection_uncertainty,number_to_remove)
        ids_example_close_margin_without_correction = select_queries_uncertainty_decision_function(train_dataset, model_learning, nbre_data_margins_selection_uncertainty)

        ################ 4. SELECTION ###################
        kernel_ = 0
        cost_g = selectioner(train_dataset,model_learning,kernel_,y_train_binary_GT, decision_function, ids_example_close_margin_correction)


        ################ 5. DIVERSIFIER ###################
        param_adjust = 0.5
        id_selection_diversification = diversifier(train_dataset,param_adjust,nbre_data_final_selection,cost_g,params,ids_example_close_margin_correction)


        stopStep2 = time.time()
        print("Computation time : {} s".format(stopStep2 - startStep2 ))
        display_selected_data(train_dataset,path_list_train_views,idx_example_positif_selection,id_selection_diversification)
        total_id_annotation = list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
        total_annotated_id_list = annotate_data(train_dataset,total_id_annotation)

        i = i + 1
    

"""
Main function for interactive learning with similarity search
"""
def similarity_search_interactiveLearning(result_jsonfile):
    global compute_full
    categorie_array_results = utils.extract_names_objects_from_result_json(result_jsonfile)

    display_color = True
    plot_PR_curve = False
    plot_cnf_matrix = False
    plot_clf_report = False
    #score of SVM for displaying data
    E_in, E_out, rank_array = [], [], []

    nb_iterations = 0

    category_label = 3
    dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), dataset_descriptor)


    if compute_full == "true":
        fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_full,testing_file_full,categorie_array_results,category_label)
            
    else :
        fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_views,testing_file_views,categorie_array_results,category_label)
        #fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size,categorie_array_results,category_label)
     
    #fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size,categorie_array_results,category_label)


    y_test_binary =[entry[1] for idx, entry in enumerate(test_dataset_binary_labeled.data)]

    unlabeled_entry_list = list(train_dataset_unlabeled.get_unlabeled_entries())

    #Initialization rank
    rank = len(unlabeled_entry_list)/2
    print("\nItération : {} ".format(nb_iterations))
    list_idx_result = select_data_similaritySearch(train_dataset_binary_labeled,categorie_array_results)

    display_color = True
    display_data_init_similarity_search_classic(list_idx_result,train_dataset_binary_labeled,display_color)

    #Annotate first data
    id_unlabeled_display_first = [id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:nbre_data_selection])]

    total_annotated_id_list = annotate_data(train_dataset_unlabeled,id_unlabeled_display_first)
    
    ################ FIRST TRAINING ###################
    model_learning = svm.SVC(kernel=kernel_svm,C = C, gamma = gam, class_weight = 'balanced', probability = True)

    training_data(train_dataset_unlabeled,model_learning)

    #Get parameters from the svm model
    params = model_learning.get_params()
    sv = model_learning.support_vectors_
    nv = model_learning.n_support_
    a  = model_learning.dual_coef_
    b  = model_learning._intercept_
    cs = model_learning.classes_

    #Get the id and features from unlabeled data
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
    #Get the id and features from all the data
    idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])

    #Get the probabilities result and decision function of all the data
    probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_all)
    decision_function = model_learning.decision_function(X_pool_all)


    X_test, y = zip(*test_dataset_binary_labeled.get_labeled_entries())
    X_test =  np.array(X_test)
    y_pred = model_learning.predict(X_test)

    f1score, AP, precision, recall, precision_curve,recall_curve,report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)


    ################ GRAPHIC ###################
    #Add the score to the chart
    score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
    E_out = np.append(E_out, 1 - score)
    E_in = np.append(E_in,score)
    score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
    score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
    #title = "Score Success : " + str(score_ok)  + " Score Error : " + str(score_error) + " \n Rank : " + str(rank)
    rank_array = np.append(rank_array, rank)

    query_num = np.arange(0, 1)
    fig = plt.figure(1,figsize=(7, 8))
    ax = fig.add_subplot(2, 1, 1)
    p1, = ax.plot(query_num, E_out, 'r', label='Error')
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
   
    
    if plot_PR_curve:
        # Plot Precision-Recall curve
        plt.figure(2)
        plt.plot(recall, precision, lw=2, color='navy',label='Precision-Recall curve')
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.ylim([0.0, 1.05])
        plt.xlim([0.0, 1.0])
        plt.title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
        plt.legend(loc="lower left")

    if plot_cnf_matrix:
        # Plot non-normalized confusion matrix
        plt.figure(3)
        class_names = ['Positif','Negatif']
        plotting.plot_confusion_matrix(cnf_matrix, classes=class_names,title='Confusion matrix, without normalization')

    
    if plot_clf_report:
        plotting.plot_classification_report(report_classification)


    plt.show(block = False)
 
    if (use_norm):
        decision_function = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)


    #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
    indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function,False)
    idx_example_positif_selection = np.take(idx_all_data,indices_ranking_descending_order)[:nbre_data_selection]


    ################ SELECTION UNCERTAINTY ###################
    #ids_example_close_margin,_ = select_queries_uncertaintysampling(train_dataset, probabilities_samples_unlabeled,nbre_data_margins_selection_uncertainty,True)
    ids_example_close_margin = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)



    total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))
    #display_selected_data(train_dataset,path_list_train_views,idx_example_positif_selection, ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display])
    save_figure = False
    display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display],display_color,save_figure)


    ###########=====>>>> SECOND STEP UPDATE  ###########
    total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)
    idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
    idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
    decision_function = model_learning.decision_function(X_pool_all)
    decision_function_custom = decision_function_vector(params, sv, nv, a, b, X_pool_unlabeled)
    #print("[INFO]Decision function classic: {} ".format(decision_function))
    decision_func = copy.copy(decision_function)
    i = 0
    while True :
        nb_iterations = nb_iterations + 1
        print("\nItération : {} ".format(nb_iterations))

        ################ 1. FEEDBACK ###################
        rank = feedback(train_dataset_unlabeled,rank,model_learning, total_annotated_id_list)
 
        training_data(train_dataset_unlabeled,model_learning)

        # Get parameters from model
        params = model_learning.get_params()
        sv = model_learning.support_vectors_
        nv = model_learning.n_support_
        a  = model_learning.dual_coef_
        b  = model_learning._intercept_
        cs = model_learning.classes_


        X_test, y = zip(*test_dataset_binary_labeled.get_labeled_entries())
        X_test =  np.array(X_test)
        y_pred = model_learning.predict(X_test)
        f1score, AP, precision, recall,precision_curve,recall_curve, report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)


        # Add score to the chart
        score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
        E_out = np.append(E_out, 1 - score)
        E_in = np.append(E_in, score)
        score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
        score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100

        rank_array = np.append(rank_array, rank)

        ax.set_xlim((0, i + 1))
        ax.set_ylim((0, max(E_out) + 0.2))
        ay.set_xlim((0, i + 1))
        ay.set_ylim((0, max(E_in) + 0.2))

        query_num = np.arange(0, i + 2)

        p1.set_xdata(query_num)
        p1.set_ydata(E_out)
        p2.set_xdata(query_num)
        p2.set_ydata(E_in)


        if plot_PR_curve:
            # Plot Precision-Recall curve
            plt.figure()
            plt.plot(recall, precision, lw=2, color='navy',label='Precision-Recall curve')
            plt.xlabel('Recall')
            plt.ylabel('Precision')
            plt.ylim([0.0, 1.05])
            plt.xlim([0.0, 1.0])
            plt.title('Precision-Recall example: Average Precision Score AUC={0:0.2f}'.format(AP))
            plt.legend(loc="lower left")

        if plot_cnf_matrix:
            # Plot non-normalized confusion matrix
            plt.figure()
            class_names = ['Positif','Negatif']
            plotting.plot_confusion_matrix(cnf_matrix, classes=class_names,title='Confusion matrix, without normalization')

    
        if plot_clf_report:
            plot_classification_report(report_classification)

        plt.show(block = False)

        idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
        idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
        probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_all)
        decision_function = model_learning.decision_function(X_pool_all)

        indices_rank_decision_function_positif_without_correction, _ = utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection_without_correction = np.take(idx_all_data,indices_rank_decision_function_positif_without_correction)[:nbre_data_selection]

        ################ 2. CORRECTION ###################
        decision_function, number_to_remove = correction(train_dataset_unlabeled, model_learning, rank)
        indices_rank_new_decision_function_positif, distance_rank_decision_function_positif = utils.sort_descending_order(decision_function,True)
        idx_example_positif_selection = np.take(idx_all_data,indices_rank_new_decision_function_positif)[:nbre_data_selection]

        ################ 3. PRESELECTION ###################
        ids_example_close_margin_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty,number_to_remove)
        ids_example_close_margin_without_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)

        ################ 4. SELECTION ###################
        kernel_ = 0
        cost_g = selectioner(train_dataset_unlabeled,model_learning,kernel_,y_train_binary_GT, decision_function, ids_example_close_margin_correction)


        ################ 5. DIVERSIFIER ###################
        param_adjust = 0.5
        id_selection_diversification = diversifier(train_dataset_unlabeled,param_adjust,nbre_data_final_selection,cost_g,params,ids_example_close_margin_correction)


        save_figure = False

        display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,id_selection_diversification,display_color,save_figure)
        #display_selected_data(train_dataset_unlabeled,path_list_train_views,idx_example_positif_selection,id_selection_diversification)
        total_id_annotation = list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
        total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)

        i = i + 1

"""
For the automatic labelling, take care of the labelling of the first step
i.e select some positive and negative label
We select 
"""
def automatic_annotate_first_step_long(fully_labeled_train_dataset,train_dataset_unlabeled,id_unlabeled_display_first,number_annotate = 6):
    #Loop over the first ID to annotate positively and negatively
    id_selected_positif = list()
    id_selected_negatif = list()

    if (number_annotate > len(id_unlabeled_display_first)):
        number_annotate = 6

    #Divide by two in order to take the first one and the last one
    number_to_annotate = math.ceil(number_annotate/2)

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, y = zip(*fully_labeled_train_dataset.data)


    count_annotate_positif = 0
    count_annotate_negatif = 0
    total_count = 0
    #Take the first 3 results of similarity search
    for i in range(number_to_annotate):
        current_id = id_unlabeled_display_first[i]
        lb = idealLabels.label(X[current_id])
        if (lb == 1):
            logging.debug('[Positif] Ask ID %s corresponds to label %s ',current_id,lb)
            id_selected_positif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_positif = count_annotate_positif + 1
        else : 
            logging.debug('[Negatif] Ask ID %s corresponds to label %s ',current_id,lb)
                
            id_selected_negatif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_negatif = count_annotate_negatif + 1

    #and the 3 last results of similarity search
    for i in range(len(id_unlabeled_display_first) - 1,len(id_unlabeled_display_first) - number_to_annotate-1,-1):
        current_id = id_unlabeled_display_first[i]
        lb = idealLabels.label(X[current_id])
        if (lb == 1):
            logging.debug('[Positif] Ask ID %s corresponds to label %s ',current_id,lb)
            id_selected_positif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_positif = count_annotate_positif + 1
        else : 
            logging.debug('[Negatif] Ask ID %s corresponds to label %s ',current_id,lb)
                
            id_selected_negatif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_negatif = count_annotate_negatif + 1

    #In case no positif or negative in the first three results or last three results, we loop deeper
    if (count_annotate_positif == 0):
        logging.debug('Look for positive example')
        for idx, id_unlabeled in enumerate(id_unlabeled_display_first):
            lb = idealLabels.label(X[id_unlabeled])
            if (lb == 1):
                logging.debug('[Positif] Ask ID %s corresponds to label %s ',id_unlabeled,lb)
                    
                id_selected_positif.append((id_unlabeled,lb))
                _item_all_label_data.append((id_unlabeled,lb))
                train_dataset_unlabeled.update(int(id_unlabeled), lb)

                count_annotate_positif = count_annotate_positif + 1

    if (count_annotate_negatif == 0):
        logging.debug('Look for negative example')
        for idx, id_unlabeled in enumerate(id_unlabeled_display_first):
            lb = idealLabels.label(X[id_unlabeled])
            if (lb != 1):
                logging.debug('[Negatif] Ask ID %s corresponds to label %s ',id_unlabeled,lb)
                    
                id_selected_negatif.append((id_unlabeled,lb))
                _item_all_label_data.append((id_unlabeled,lb))
                train_dataset_unlabeled.update(int(id_unlabeled), lb)

                count_annotate_negatif = count_annotate_negatif + 1

    #If not negative result in similarity search, take randomly one example negatif in the unlabeleded dataset
    if (count_annotate_negatif == 0):
        while (count_annotate_negatif == 0):
            id_random = random.randint(0, len(X)-1)
            feature_random = X[id_random]
            lb = idealLabels.label(feature_random)

            if (lb == - 1):
                count_annotate_negatif = count_annotate_negatif + 1
                id_selected_negatif.append((id_random,lb))
                _item_all_label_data.append((id_random,lb))
                train_dataset_unlabeled.update(int(id_random), lb)
                logging.debug('Label : %s ,negative has been selected randomly',lb)

    if (count_annotate_positif == 0):
        logging.debug('Still none positive')
        while (count_annotate_positif == 0):

            id_random = random.randint(0, len(X)-1)
            feature_random = X[id_random]

            lb = idealLabels.label(feature_random)

            if (lb == 1):
                count_annotate_positif = count_annotate_positif + 1
                id_selected_positif.append((id_random,lb))
                _item_all_label_data.append((id_random,lb))
                train_dataset_unlabeled.update(int(id_random), lb)
                logging.debug('Label : %s ,positive has been selected randomly',lb)
        

    #If still we did not find posiitf and negative, raise error because we couldn't train the svm
    if (count_annotate_positif == 0 or count_annotate_negatif == 0):
        logging.error('No positive or negative label could be selected ')
        exit(1)


    total_annotated_id_list = id_selected_positif + id_selected_negatif


    logging.debug('Update Nbr unlabeled : : %s',train_dataset_unlabeled.len_unlabeled())
    logging.debug('Update Nbr labeled : : %s',train_dataset_unlabeled.len_labeled())
    logging.debug('===> Step selection positif : %s',id_selected_positif)
    logging.debug('===> Step selection negatif : %s',id_selected_negatif)


    return total_annotated_id_list

"""
For the automatic labelling, take care of the labelling of the first step
i.e select some positive and negative label
We select 
"""
def automatic_annotate_first_step(fully_labeled_train_dataset,train_dataset_unlabeled,id_candidates,query_cloud):
    #Loop over the first ID to annotate positively and negatively
    id_selected_positif = list()
    id_selected_negatif = list()

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, y = zip(*fully_labeled_train_dataset.data)
   
    #exit()

    count_annotate_positif = 0
    count_annotate_negatif = 0
    total_count = 0

    logging.debug('IDX candidates %s ',id_candidates)
    for idx in id_candidates:
        current_id = idx
        lb = idealLabels.label(X[current_id])
        if (lb == 1):
            logging.debug('1. [Positif] Ask ID %s corresponds to label %s ',current_id,lb)

            id_selected_positif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_positif = count_annotate_positif + 1
        else : 
            logging.debug('1. [Negatif] Ask ID %s corresponds to label %s ',current_id,lb)
            
            id_selected_negatif.append((current_id,lb))
            _item_all_label_data.append((current_id,lb))
            train_dataset_unlabeled.update(int(current_id), lb)

            count_annotate_negatif = count_annotate_negatif + 1


    #If not negative result in similarity search, take randomly one example negatif in the unlabeleded dataset
    if (count_annotate_negatif == 0):
        while (count_annotate_negatif == 0):
            id_random = random.randint(0, len(X)-1)
            feature_random = X[id_random]

            lb = idealLabels.label(feature_random)
            if (lb == - 1):
                count_annotate_negatif = count_annotate_negatif + 1
                id_selected_negatif.append((id_random,lb))
                _item_all_label_data.append((id_random,lb))
                train_dataset_unlabeled.update(int(id_random), lb)
                logging.debug('Label %s for ID %s ,negative has been selected randomly',id_random,lb)

    if (count_annotate_positif == 0):

        logging.debug('Still none positive')
        while (count_annotate_positif == 0):
            id_random = random.randint(0, len(X)-1)
            feature_random = X[id_random]
           
            lb = idealLabels.label(feature_random)
            if (lb == 1):
                count_annotate_positif = count_annotate_positif + 1
                id_selected_positif.append((id_random,lb))
                _item_all_label_data.append((id_random,lb))
                train_dataset_unlabeled.update(int(id_random), lb)
                logging.debug('Label %s for ID %s ,positive has been selected randomly',id_random,lb)
        

    #If still we did not find posiitf and negative, raise error because we couldn't train the svm
    if (count_annotate_positif == 0 or count_annotate_negatif == 0):
        #print("No positive or negative could be selected result similarity")
        logging.error('No positive or negative label could be selected ')
        exit(1)


    total_annotated_id_list = id_selected_positif + id_selected_negatif


    return total_annotated_id_list
        
"""
During the main loop, this function take care of automatically label positive and negative label among the most uncertain one
We take the first one and we look for its label
"""
def automatic_annotate_most_uncertain_second_step(fully_labeled_train_dataset,train_dataset_unlabeled,id_unlabeled_most_uncertain,number_to_annotate = 5):
    #Loop over the first ID to annotate positively and negatively
    id_selected_positif = list()
    id_selected_negatif = list()

    number_positif_to_annotate = 3
    number_negatif_to_annotate = 3
    
    if (number_to_annotate > len(id_unlabeled_most_uncertain)):
        number_to_annotate = len(id_unlabeled_most_uncertain) - 1

    idealLabels = IdealLabeler(fully_labeled_train_dataset)
    X, _ = zip(*fully_labeled_train_dataset.data)

    count_annotate_positif = 0
    count_annotate_negatif = 0
    for i in range(number_to_annotate):
        current_id_unlabeled = id_unlabeled_most_uncertain[i]
        lb = idealLabels.label(X[current_id_unlabeled])
        
        if (lb == 1):
            logging.debug('[Positif] Ask ID %s corresponds to label %s ',current_id_unlabeled,lb)
                
            id_selected_positif.append((current_id_unlabeled,lb))
            _item_all_label_data.append((current_id_unlabeled,lb))
            train_dataset_unlabeled.update(int(current_id_unlabeled), lb)

            count_annotate_positif = count_annotate_positif + 1
        else : 
            
            logging.debug('[Negatif] Ask ID %s corresponds to label %s ',current_id_unlabeled,lb)
                
            id_selected_negatif.append((current_id_unlabeled,lb))
            _item_all_label_data.append((current_id_unlabeled,lb))
            train_dataset_unlabeled.update(int(current_id_unlabeled), lb)

            count_annotate_negatif = count_annotate_negatif + 1

    if (count_annotate_positif == 0):
        logging.warning('No positive label could be selected ')
    if (count_annotate_negatif == 0):
        logging.warning('No negative label could be selected ')

    total_annotated_id_list = id_selected_positif + id_selected_negatif

    logging.debug('Update Nbr unlabeled : : %s',train_dataset_unlabeled.len_unlabeled())
    logging.debug('Update Nbr labeled : : %s',train_dataset_unlabeled.len_labeled())
    logging.debug('===> Step selection positif : %s',id_selected_positif)
    logging.debug('===> Step selection negatif : %s',id_selected_negatif)

    return total_annotated_id_list



"""
Main function for interactive learning with similarity search - Automatic labelling (no user required)
"""
def automatic_interactiveLearning(result_jsonfile,number_to_annotate_first_step,number_to_annotate_uncertain,nb_iterations_max,repetition_experiment = 1,save_figures = True):

    global category_label
    categorie_array_results = utils.extract_names_objects_from_result_json(result_jsonfile)


    #score of SVM for displaying data
    E_in, E_out, score_pourcent, error_pourcent, rank_array, positif_display_pourcent, negatif_display_pourcent = [], [], [], [], [], [], []
    f1score, AP, precision_array, recall_array,precision_curve,recall_curve, report_classification, cnf_matrix = [], [], [], [], [], [], [], []
    #Result final
    results_error = []
    results_score = []

    nb_iterations = 1

    display_graphic_each_step = True
    plot_PR_curve = False
    plot_cnf_matrix = False
    plot_clf_report = False

    start = time.time()
    #Repeat the experiment
    for T in range(repetition_experiment):

        dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), dataset_descriptor)
    
        if compute_full == "true":
            print("[INFO] Training - testing on full object")
            fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_full,testing_file_full,categorie_array_results,category_label)
            
        else :
            print("[INFO] Training - testing on views object")
            fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_views,testing_file_views,categorie_array_results,category_label)
            #fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size,categorie_array_results,category_label)
        

        #fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size,categorie_array_results,category_label)
        unlabeled_entry_list = list(train_dataset_unlabeled.get_unlabeled_entries())

        #Initialization rank
        rank = len(unlabeled_entry_list)/2

        print("=====>Itération : ({}/{}) ".format(nb_iterations,nb_iterations_max))

        #list_idx_candidate = select_data_similaritySearch(train_dataset_binary_labeled,categorie_array_results)
        list_idx_result = select_data_similaritySearch(train_dataset_binary_labeled,categorie_array_results)  

        list_idx_result_first_three = select_data_similaritySearch_byindex(train_dataset_binary_labeled,categorie_array_results,0,3)
        list_idx_result_last_three = select_data_similaritySearch_byindex(train_dataset_binary_labeled,categorie_array_results,len(categorie_array_results)-4,len(categorie_array_results))
        list_idx_candidate = list_idx_result_first_three + list_idx_result_last_three
        

        if display_graphic_each_step:
            displayColor = True
            automatic_display_data_init_similarity_search(list_idx_result,train_dataset_binary_labeled,displayColor)
            #list_idx_result = display_data_init_similarity_search_test(train_dataset_binary_labeled,categorie_array_results, nbre_data_selection)

        #Annotate first data
        id_unlabeled_display_first = [id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:nbre_data_selection])]
        
        #total_annotated_id_list = automatic_annotate_first_step_long(train_dataset_binary_labeled,train_dataset_unlabeled,list_idx_result,number_to_annotate_first_step)
        total_annotated_id_list = automatic_annotate_first_step(train_dataset_binary_labeled,train_dataset_unlabeled,list_idx_candidate,query_cloud)

        #total_annotated_id_list = annotate_data(train_dataset_unlabeled,id_unlabeled_display_first)
        
        ################ FIRST TRAINING ###################
        model_learning = svm.SVC(kernel=kernel_svm,C=C, gamma = gam, class_weight = 'balanced',probability = True)

        training_data(train_dataset_unlabeled,model_learning)

            #Get parameters from the svm model
        params = model_learning.get_params()
        sv = model_learning.support_vectors_
        nv = model_learning.n_support_
        a  = model_learning.dual_coef_
        b  = model_learning._intercept_
        cs = model_learning.classes_

        #Get the id and features from unlabeled data
        idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
        #Get the id and features from all the data
        idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])


        #Get the probabilities result and decision function of all the data
        probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
        decision_function = model_learning.decision_function(X_pool_unlabeled)

        X_test, y_test_binary = zip(*test_dataset_binary_labeled.get_labeled_entries())
        X_test =  np.array(X_test)
        y_pred = model_learning.predict(X_test)
        #y_pred = model_learning.decision_function(X_test)
        f1score, AP, precision, recall, precision_curve,recall_curve,report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)

        ################ Score ###################
        #Add the score to the chart
        score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
        E_out = np.append(E_out, 1 - score)
        E_in = np.append(E_in,score)
        score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
        score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
        score_pourcent = np.append(score_pourcent, score_ok)
        error_pourcent = np.append(error_pourcent,score_error)
        rank_array = np.append(rank_array, rank)
        precision_array = np.append(precision_array,precision)
        recall_array = np.append(recall_array,recall)




        if display_graphic_each_step:
            #title = "Score Success : " + str(score_ok)  + " Score Error : " + str(score_error) + " \n Rank : " + str(rank)

            query_num = np.arange(0, 1)
            fig = plt.figure(figsize=(5, 5))
            ax = fig.add_subplot(2, 1, 1)
            p1, = ax.plot(query_num, E_out,'r', label='Error', linewidth = 2)
            ax.set_xlabel('Number of steps')
            ax.set_ylabel('Error')
            ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
                       shadow=True, ncol=5)


            ay = fig.add_subplot(2, 1, 2)
            p2, = ay.plot(query_num, E_in,'g', label='Score', linewidth = 2)
            ay.set_xlabel('Number of steps ')
            ay.set_ylabel('Score')
            ay.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True,
                       shadow=True, ncol=5)

  
  #fig.suptitle(title, fontsize=8, fontweight='bold')
            if plot_PR_curve:
                plotting.plot_precision_recall(recall,precision,AP)

            if plot_cnf_matrix:
                # Plot non-normalized confusion matrix
                plt.figure()
                class_names = ['Positif','Negatif']
                plot_conf = plotting.plot_confusion_matrix(cnf_matrix, classes=class_names,title='Confusion matrix, without normalization')
                title = "confusion_matrix_" + str(category_label) + ".png"
                plot_conf.savefig(title)
                #plotting.plot_classification_report(report_classification)

            plt.show(block = True)

     
        if (use_norm):
            #print(decision_function)
            decision_function = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)


        #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
        indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function,False)
        idx_example_positif_selection = np.take(idx_unlabeled_data,indices_ranking_descending_order)[:nbre_data_selection]

        

        pourcent_positif_displayed, pourcent_negatif_displayed = compute_pourcentage_results(train_dataset_binary_labeled,train_dataset_unlabeled,idx_example_positif_selection,20)
        positif_display_pourcent = np.append(positif_display_pourcent,pourcent_positif_displayed)
        negatif_display_pourcent = np.append(negatif_display_pourcent,pourcent_negatif_displayed)


        ################ SELECTION UNCERTAINTY ###################
        ids_example_close_margin = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)


        
        total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))
        
        if display_graphic_each_step:
            save_figure = False
            display_color = True
            display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display],display_color,save_figure)

        ###########=====>>>> SECOND STEP UPDATE  ###########
        print("\n")
        total_annotated_id_list = automatic_annotate_most_uncertain_second_step(train_dataset_binary_labeled,train_dataset_unlabeled,ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display],number_to_annotate_uncertain)

        idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
        decision_function = model_learning.decision_function(X_pool_unlabeled)
        decision_function_custom = decision_function_vector(params, sv, nv, a, b, X_pool_unlabeled)

        decision_func = copy.copy(decision_function)
        i = 0
        #run_onelabel(trn_ds, tst_ds, idealLabeler, model, id_list_uncertain)
        while nb_iterations < nb_iterations_max :
            nb_iterations = nb_iterations + 1
            print("\nAutomatique =====>Itération : ({}/{}) ".format(nb_iterations,nb_iterations_max))

            ################ 1. FEEDBACK ###################
            rank = feedback(train_dataset_unlabeled,rank,model_learning, total_annotated_id_list)
     
            training_data(train_dataset_unlabeled,model_learning)

            # Get parameters from model
            params = model_learning.get_params()
            sv = model_learning.support_vectors_
            nv = model_learning.n_support_
            a  = model_learning.dual_coef_
            b  = model_learning._intercept_
            cs = model_learning.classes_

            pourcent_positif_displayed,pourcent_negatif_displayed = compute_pourcentage_results(train_dataset_binary_labeled,train_dataset_unlabeled,idx_example_positif_selection,20)


            X_test, y_test_binary = zip(*test_dataset_binary_labeled.get_labeled_entries())
            X_test =  np.array(X_test)
            y_pred = model_learning.predict(X_test)
            #y_pred = model_learning.decision_function(X_test)
            f1score, AP, precision, recall, precision_curve,recall_curve,report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)

            # Add score to the chart
            score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
            E_out = np.append(E_out, 1 - score)
            E_in = np.append(E_in, score)
            score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
            score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
            score_pourcent = np.append(score_pourcent, score_ok)
            error_pourcent = np.append(error_pourcent,score_error)
            print("[INFO]  ==> SCORE : {}".format(score_ok))
            rank_array = np.append(rank_array, rank)
            precision_array = np.append(precision_array,precision)
            recall_array = np.append(recall_array,recall)

            positif_display_pourcent = np.append(positif_display_pourcent,pourcent_positif_displayed)
            negatif_display_pourcent = np.append(negatif_display_pourcent,pourcent_negatif_displayed)
            
            idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
            idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
            probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
            decision_function = model_learning.decision_function(X_pool_unlabeled)

            indices_rank_decision_function_positif_without_correction, _ = utils.sort_descending_order(decision_function,True)
            idx_example_positif_selection_without_correction = np.take(idx_unlabeled_data,indices_rank_decision_function_positif_without_correction)[:nbre_data_selection]

            ################ 2. CORRECTION ###################
            decision_function, number_to_remove = correction(train_dataset_unlabeled, model_learning, rank)
            indices_rank_new_decision_function_positif, distance_rank_decision_function_positif = utils.sort_descending_order(decision_function,True)
            idx_example_positif_selection = np.take(idx_unlabeled_data,indices_rank_new_decision_function_positif)[:nbre_data_selection]

            ################ 3. PRESELECTION ###################
            ids_example_close_margin_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty,number_to_remove)
            ids_example_close_margin_without_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)

            ################ 4. SELECTION ###################
            kernel_ = 0
            cost_g = selectioner(train_dataset_unlabeled,model_learning,kernel_,y_train_binary_GT, decision_function, ids_example_close_margin_correction)


            ################ 5. DIVERSIFIER ###################
            param_adjust = 0.5
            id_selection_diversification = diversifier(train_dataset_unlabeled,param_adjust,nbre_data_final_selection,cost_g,params,ids_example_close_margin_correction)


            if display_graphic_each_step:
                save_figure = False
                display_color = True
                print("IDX example positif selection {}".format(idx_example_positif_selection))
                display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,id_selection_diversification,display_color,save_figure)

                quota = nb_iterations - 1
                query_num = np.arange(0, quota + 1)
                                
                fig = plt.figure(1,figsize=(10,7))

                figure_colour=["g","b"]
                figure_data = [score_pourcent, positif_display_pourcent ]
                figure_label = ['Score','Positif display']
                figure_subtitle = ['Score of SVM testing over iterations','Number of positif samples displayed to the user in the top 20']


                plotting.plot_curves(query_num,figure_data,figure_colour,figure_label,figure_subtitle,"Curves results",save_figures)
                
                if plot_PR_curve:
                    plotting.plot_curves_precision_recall(query_num,recall_array,precision_array,recall_curve,precision_curve,AP,save_figures)

                if plot_cnf_matrix:
                    # Plot non-normalized confusion matrix
                    plt.figure()
                    class_names = ['Positif','Negatif']
                    plotting.plot_confusion_matrix(cnf_matrix, classes=class_names,title='Confusion matrix, without normalization')
                    #plotting.plot_classification_report(report_classification)
                plt.show(block = True)


            total_id_annotation = list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
            #total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)

            total_annotated_id_list = automatic_annotate_most_uncertain_second_step(train_dataset_binary_labeled,train_dataset_unlabeled,id_selection_diversification,number_to_annotate_uncertain)


            i = i + 1
            
        
        #print("SCORE POURCENT : {} ".format(score_pourcent))
        results_error.append(error_pourcent.tolist())
        results_score.append(score_pourcent.tolist())

    #Display graphic and save it
    end = time.time()
    time_experiment_elapsed = end - start
    title = "Time experiment = " + str(time_experiment_elapsed) + " s"+\
            "\n Score Max : " + str(max(score_pourcent))  + " %\n Current Score : " + str(score_pourcent[len(score_pourcent)-1]) +\
            " %\n Current Error : " + str(error_pourcent[len(error_pourcent)-1]) + " %"
    quota = nb_iterations_max - 1
    query_num = np.arange(0, quota + 1)
        
    fig = plt.figure(1,figsize=(10,7))
    plt.suptitle(title, fontsize=8, fontweight='bold')


    figure_colour=["g","b"]
    figure_data = [score_pourcent, positif_display_pourcent ]
    figure_label = ['Score','Positif display']
    figure_subtitle = ['Score of SVM testing over iterations','Number of positif samples displayed to the user in the top 20']

    plotting.plot_curves(query_num,figure_data,figure_colour,figure_label,figure_subtitle,"Curves results",save_figures)
    plotting.plot_curves_precision_recall(query_num,recall_array,precision_array,recall_curve,precision_curve,AP,save_figures)
        
    
    display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,id_selection_diversification,True,save_figures)
    plt.show(block = True)



"""
Main function for interactive learning with similarity search - Automatic labelling (no user required) with as an input list of object
"""
def automatic_interactiveLearning_objectlist(params,name_descriptor):
    list_of_random_objects, trained_dataset, name_dataset = params

    display_graphic_each_step = False
    save_figures = True
    global view_path_file
    global dataset_descriptor
    global output_graphicals_result
    global compute_full
    #For multiple Thread
    output_json_result = "data/results_similarity_"+name_dataset+"_"+name_descriptor+".json"
    view_file_to_analyse = root_data + name_dataset + "/dataset_descriptor_" + name_descriptor + ".txt"
    output_graphicals_result = "results_graphics"+name_dataset+"/"

    if not os.path.exists(output_graphicals_result):
        os.makedirs(output_graphicals_result)

    descriptor_file_to_analyse = root_data + name_dataset + "/descriptors_"+ name_descriptor + ".txt"

    view_path_file = view_file_to_analyse

    dataset_descriptor = descriptor_file_to_analyse


    if (os.path.exists(view_file_to_analyse) and os.path.exists(descriptor_file_to_analyse)):
        logging.debug("View file to analyse : %s",view_file_to_analyse)
        logging.debug("descriptor file to analyse : %s",descriptor_file_to_analyse)

    else :
        logging.error("View path file %s and dataset descriptor %s not found",view_file_to_analyse,descriptor_file_to_analyse)
        return 1

    #Result final
    results_error = []
    results_score = []

    #Interval matplotlib
    reg_interval_y = plticker.MultipleLocator(base=10.0) # this locator puts ticks at regular intervals
    reg_interval_x = plticker.MultipleLocator(base=1.0) # this locator puts ticks at regular intervals

    
    #Array which will contains the score and results of the results of the list of object (length is the number of categories) for the category
    final_results_error_category,final_results_score_category = [], []
    #Array which will contain the proportion of positifs results displayed to the users in comparaison of the number of negatif displayed for the category
    final_percent_positif_displayed,final_percent_negatif_displayed = [],[]

    final_average_precision, final_recall,final_precision, final_f1_score, final_recall_curve,final_precision_curve = [], [], [], [], [], []

    
    start_total_experiment = time.time()

    len_category_dataset = len(list_of_random_objects)
    #for each cateogry of the dataset
    for idx, object_list_category in enumerate(list_of_random_objects):
        print("=====>[INFO] Catégorie : ({}/{}) ".format(idx,len_category_dataset))
        logging.info("\n =====>[INFO] Catégorie : (%s/%s) ",idx,len_category_dataset)

        #Arrays which will contains the score and results of each objects (length is the number of objects per categories)
        score_current_object_list,error_current_object_list = [], []
        #Arrays which will contains the pourcentage of positif in the top 20 for the current object
        percent_positif_current_object_list, percent_negatif_current_object_list = [], []
        #Arrays which contains informations about F1 score, precision and recall for the current object
        f1score_current_object_list, AP_current_object_list, precision_current_object_list, recall_current_object_list, report_classification_current_object_list, cnf_matrix_current_object_list = [], [], [], [], [], []
        recall_curve_object_list, precision_curve_object_list = [], []

        category_label = idx + 1
        print("Categorie label : {} ".format(category_label))
        logging.info("Categorie label : %s ",category_label)

        print("\n =====> Processing current category  : {} <===== ".format(object_list_category[0]))
        logging.info("\n =====> Processing current category : %s ",object_list_category[0])
        
        start_total_category = time.time()

        #For each objects of dataset
        for obj in object_list_category[1]:
            #Current Nbre iteration per object         
            nb_iterations = 1
            #Store data score for displaying a graphic at the end of the experiment
            E_in, E_out, score_pourcent, error_pourcent, rank_array, positif_display_pourcent, negatif_display_pourcent = [], [], [], [], [], [], []
            f1score_array, AP_array, precision_array, recall_array, report_classification_array, cnf_matrix_array = [], [], [], [], [], []

            print("\n ###### Processing current OBJECT : {} \n".format(obj))
            logging.info("Processing current OBJECT : %s ",obj)

            #Check if "/" is at the end of the path. Otherwise add it            
            if not trained_dataset.endswith(os.path.sep):
                trained_dataset += os.path.sep


            #Run similarity search
            cmd = './cloudRetrieval_main -query ' + obj + ' -trained ' + trained_dataset + ' -descriptor ' + descriptor_similaritySearch + ' -k ' + str(k) + ' -leaf_resolution ' + str(leaf_resolution) + ' -output ' + output_json_result + ' -compute_full ' + compute_full 
            os.system(cmd) # returns the exit status

            print("\n [INFO] Please wait...")

            categorie_array_results = utils.extract_names_objects_from_result_json(output_json_result)


            dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), dataset_descriptor)

            if compute_full == "true":
                print("[INFO] Training - testing on full object")
                fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_full,testing_file_full,categorie_array_results,category_label)
            
            else :
                print("[INFO] Training - testing on views object")
                fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_training_testing_data_similaritySearch_GT(dataset_filepath, training_file_views,testing_file_views,categorie_array_results,category_label)
            #fully_dataset_labeled_binary,train_dataset_unlabeled, train_dataset_binary_labeled, y_train_binary_GT,test_dataset_binary_labeled, path_list_train_views,path_list_test_views = split_train_test_from_libsvm_data_similaritySearch_GT(dataset_filepath, test_size,categorie_array_results,category_label)
     
            unlabeled_entry_list = list(train_dataset_unlabeled.get_unlabeled_entries())

            #Initialization rank
            rank = len(unlabeled_entry_list)/2

            print("=====>Itération : ({}/{}) ".format(nb_iterations,nb_iterations_max))
            logging.info("=====>Itération : (%s/%s) ",nb_iterations,nb_iterations_max)



            list_idx_result = select_data_similaritySearch(train_dataset_binary_labeled,categorie_array_results)  
            #print("INFO : sselect_data_similaritySearch_byindex 1 ")
            list_idx_result_first_three = select_data_similaritySearch_byindex(train_dataset_binary_labeled,categorie_array_results,0,3)
            #print("INFO : sselect_data_similaritySearch_byindex 2 ")
            list_idx_result_last_three = select_data_similaritySearch_byindex(train_dataset_binary_labeled,categorie_array_results,len(categorie_array_results)-4,len(categorie_array_results))
            #List ID candidate for labelling at the first step
            list_idx_candidate = list_idx_result_first_three + list_idx_result_last_three

            if display_graphic_each_step:
                automatic_display_data_init_similarity_search(list_idx_result,train_dataset_binary_labeled,True)
                #list_idx_result = display_data_init_similarity_search_test(train_dataset_binary_labeled,categorie_array_results, nbre_data_selection)

            #Annotate first data
            id_unlabeled_display_first=[id_feature for id_feature, feature in enumerate(unlabeled_entry_list[:nbre_data_selection])]
            print("INFO : begin annotate first step")
            total_annotated_id_list = automatic_annotate_first_step(train_dataset_binary_labeled,train_dataset_unlabeled,list_idx_candidate,number_to_annotate_first_step)
            print("INFO : finish annotate first step")
            
            ################ FIRST TRAINING ###################
            model_learning = svm.SVC(kernel=kernel_svm,C=C, gamma = gam, class_weight = 'balanced',probability = True)

            training_data(train_dataset_unlabeled,model_learning)

                #Get parameters from the svm model
            params = model_learning.get_params()
            sv = model_learning.support_vectors_
            nv = model_learning.n_support_
            a  = model_learning.dual_coef_
            b  = model_learning._intercept_
            cs = model_learning.classes_

            #Get the id and features from unlabeled data
            idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
            #Get the id and features from all the data
            idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])


            #Get the probabilities result and decision function of all the data
            probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
            decision_function = model_learning.decision_function(X_pool_unlabeled)



            ################ Score ###################    
            X_test, y_test_binary = zip(*test_dataset_binary_labeled.get_labeled_entries())
            X_test =  np.array(X_test)
            y_pred = model_learning.predict(X_test)
            f1score, AP, precision, recall, precision_curve,recall_curve,report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)

            #Add the score to the chart
            score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
            E_out = np.append(E_out, 1 - score)
            E_in = np.append(E_in,score)
            score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
            score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
            score_pourcent = np.append(score_pourcent,score_ok)
            error_pourcent = np.append(error_pourcent,score_error)
            rank_array = np.append(rank_array, rank)
            f1score_array = np.append(f1score_array, f1score)
            AP_array = np.append(AP_array, AP)
            recall_array = np.append(recall_array, recall)
            precision_array = np.append(precision_array,precision)

            logging.info("==> F1 Score : %s ",f1score)
            logging.info("==> Precision : %s ",precision)
            logging.info("==> Recall : %s ",recall)
            logging.info("==> SCORE: %s ",score_ok)


            #print("==> F1 Score : {}".format(f1score))
            #print("==> Precision : {}".format(precision))
            #print("==> Recall : {}".format(recall))
            #print("==> SCORE : {}".format(score_ok))

         
            if (use_norm):
                decision_function = MinMaxScaler(feature_range=(-1, 1)).fit_transform(decision_function)


            #Sort the result in descending order so that we get the most certains results, i.e further point in the positive side
            indices_ranking_descending_order, distance_ranking_descending_order = utils.sort_descending_order(decision_function,False)
            idx_example_positif_selection = np.take(idx_unlabeled_data,indices_ranking_descending_order)[:nbre_data_selection]

            pourcent_positif_displayed, pourcent_negatif_displayed = compute_pourcentage_results(train_dataset_binary_labeled,train_dataset_unlabeled,idx_example_positif_selection,nb_max_positif_display)
            positif_display_pourcent = np.append(positif_display_pourcent,pourcent_positif_displayed)
            negatif_display_pourcent = np.append(negatif_display_pourcent,pourcent_negatif_displayed)


            ################ SELECTION UNCERTAINTY ###################
            #ids_example_close_margin,_ = select_queries_uncertaintysampling(train_dataset, probabilities_samples_unlabeled,nbre_data_margins_selection_uncertainty,True)
            ids_example_close_margin = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)


            
            total_id_annotation = list(itertools.chain(idx_example_positif_selection, ids_example_close_margin))

            ###########=====>>>> SECOND STEP UPDATE  ###########
            #print("\n")
            total_annotated_id_list = automatic_annotate_most_uncertain_second_step(train_dataset_binary_labeled,train_dataset_unlabeled,ids_example_close_margin[:nbre_data_margins_selection_uncertainty_display],number_to_annotate_second_step)

            #total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)
            idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
            decision_function = model_learning.decision_function(X_pool_unlabeled)
            decision_function_custom = decision_function_vector(params, sv, nv, a, b, X_pool_unlabeled)
            #print("[INFO]Decision function classic: {} ".format(decision_function))
            decision_func = copy.copy(decision_function)
            i = 0

            while nb_iterations < nb_iterations_max :
                nb_iterations = nb_iterations + 1
                print("\n=====>Itération : ({}/{}) ".format(nb_iterations,nb_iterations_max))
                logging.info("\n=====>Itération : (%s/%s) ",nb_iterations,nb_iterations_max)

                ################ 1. FEEDBACK ###################
                rank = feedback(train_dataset_unlabeled,rank,model_learning, total_annotated_id_list)
         
                training_data(train_dataset_unlabeled,model_learning)

                # Get parameters from model
                params = model_learning.get_params()
                sv = model_learning.support_vectors_
                nv = model_learning.n_support_
                a  = model_learning.dual_coef_
                b  = model_learning._intercept_
                cs = model_learning.classes_

                pourcent_positif_displayed,pourcent_negatif_displayed = compute_pourcentage_results(train_dataset_binary_labeled,train_dataset_unlabeled,idx_example_positif_selection,nb_max_positif_display)

                X_test, y_test_binary = zip(*test_dataset_binary_labeled.get_labeled_entries())
                X_test =  np.array(X_test)
                y_pred = model_learning.predict(X_test)
                

                #Compute SCORE SVM
                f1score, AP, precision, recall,precision_curve,recall_curve, report_classification, cnf_matrix = compute_scores_SVM(X_test,y_test_binary,y_pred,model_learning)



                ################ Score ###################    
                score = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))
                E_out = np.append(E_out, 1 - score)
                E_in = np.append(E_in, score)
                score_ok = model_learning.score(*(test_dataset_binary_labeled.format_sklearn()))*100
                score_error = (1 - model_learning.score(*(test_dataset_binary_labeled.format_sklearn())))*100
                score_pourcent = np.append(score_pourcent,score_ok)
                error_pourcent = np.append(error_pourcent,score_error)
                rank_array = np.append(rank_array, rank)
                positif_display_pourcent = np.append(positif_display_pourcent,pourcent_positif_displayed)
                negatif_display_pourcent = np.append(negatif_display_pourcent,pourcent_negatif_displayed)
                precision_array = np.append(precision_array,precision)
                recall_array = np.append(recall_array,recall)
                f1score_array = np.append(f1score_array, f1score)
                AP_array = np.append(AP_array, AP)

                #print("==> F1 Score : {}".format(f1score))
                #print("==> Precision : {}".format(precision))
                #print("==> Recall : {}".format(recall))
                #print("==> SCORE : {}".format(score_ok))
                logging.info("==> F1 Score : %s ",f1score)
                logging.info("==> Precision : %s ",precision)
                logging.info("==> Recall : %s ",recall)
                logging.info("==> SCORE: %s ",score_ok)
                
                idx_unlabeled_data, X_pool_unlabeled = zip(*train_dataset_unlabeled.get_unlabeled_entries())
                idx_all_data, X_pool_all =  zip(*[(idx, entry[0]) for idx, entry in enumerate(train_dataset_unlabeled.data)])
                probabilities_samples_unlabeled = model_learning.predict_proba(X_pool_unlabeled)
                decision_function = model_learning.decision_function(X_pool_unlabeled)

                indices_rank_decision_function_positif_without_correction, _ = utils.sort_descending_order(decision_function,True)
                idx_example_positif_selection_without_correction = np.take(idx_unlabeled_data,indices_rank_decision_function_positif_without_correction)[:nbre_data_selection]

                ################ 2. CORRECTION ###################
                decision_function, number_to_remove = correction(train_dataset_unlabeled, model_learning, rank)
                indices_rank_new_decision_function_positif, distance_rank_decision_function_positif = utils.sort_descending_order(decision_function,True)
                idx_example_positif_selection = np.take(idx_unlabeled_data,indices_rank_new_decision_function_positif)[:nbre_data_selection]

                ################ 3. PRESELECTION ###################
                ids_example_close_margin_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty,number_to_remove)
                ids_example_close_margin_without_correction = select_queries_uncertainty_decision_function(train_dataset_unlabeled, model_learning, nbre_data_margins_selection_uncertainty)

                ################ 4. SELECTION ###################
                kernel_ = 0
                cost_g = selectioner(train_dataset_unlabeled,model_learning,kernel_,y_train_binary_GT, decision_function, ids_example_close_margin_correction)


                ################ 5. DIVERSIFIER ###################
                param_adjust = 0.5
                id_selection_diversification = diversifier(train_dataset_unlabeled,param_adjust,nbre_data_final_selection,cost_g,params,ids_example_close_margin_correction)


                if display_graphic_each_step:
                    save_figure = False
                    display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,id_selection_diversification,save_figure)

                total_id_annotation = list(itertools.chain(idx_example_positif_selection, id_selection_diversification))
                #total_annotated_id_list = annotate_data(train_dataset_unlabeled,total_id_annotation)

                total_annotated_id_list = automatic_annotate_most_uncertain_second_step(train_dataset_binary_labeled, train_dataset_unlabeled, id_selection_diversification,7)

                #plot confusion matrix
                if nb_iterations < nb_iterations_max - 1:
                    # Plot non-normalized confusion matrix
                    class_names = ['Positif','Negatif']
                    plot_conf = plotting.plot_confusion_matrix(cnf_matrix, classes=class_names,title='Confusion matrix, without normalization')
                    base=os.path.basename(obj)
                    name_object =  os.path.splitext(base)[0]
                    title = "confusion_matrix_" + str(name_descriptor) + "_" + str(name_dataset) + "_" + str(name_object) +".png"
                    output_graphicals_result = "results_graphics"+name_dataset+"/" + str(title)
                    plot_conf.savefig(output_graphicals_result)

                i = i + 1
                
            #After processing one object, put the results in another lists. So that, at the end we can perform a mean of each category
            score_current_object_list.append(score_pourcent)
            error_current_object_list.append(error_pourcent)
            percent_positif_current_object_list.append(positif_display_pourcent)
            percent_negatif_current_object_list.append(negatif_display_pourcent)
            f1score_current_object_list.append(f1score_array)
            AP_current_object_list.append(AP)
            precision_current_object_list.append(precision_array)
            recall_current_object_list.append(recall_array)
            recall_curve_object_list.append(recall_curve)
            precision_curve_object_list.append(precision_curve)
             


        end_total_category = time.time()
        time_experiment_category_elapsed = end_total_category - start_total_category
        
        print("Category : {} | Time Elapsed : {} s".format(object_list_category[0],time_experiment_category_elapsed))
        logging.info('Category : %s | Time Elapsed : %s s',object_list_category[0],time_experiment_category_elapsed)

        ########################################### END COMPUTATION FOR A SPECIFIC CATEGORY ###########################################
        
        #Mean of resuts from a specific category
        mean_score_objects_array = np.mean(score_current_object_list, axis = 0)
        mean_error_objects_array = np.mean(error_current_object_list, axis = 0)
        mean_positif_percent_array = np.mean(percent_positif_current_object_list, axis = 0)
        mean_negatif_percent_array = np.mean(percent_negatif_current_object_list, axis = 0)
        mean_f1_score_objects_array = np.mean(f1score_current_object_list, axis = 0)
        mean_average_precision_objects_array = np.mean(AP_current_object_list, axis = 0)
        mean_precision_objects_array = np.mean(precision_current_object_list, axis = 0)
        mean_recall_objects_array = np.mean(recall_current_object_list, axis = 0)
        #As it may possible that the recall curve list and precision curve does not have the same length
        mean_precision_curve_array =  [float(sum(l))/len(l) for l in zip(*precision_curve_object_list)]
        mean_recall_curve_array =  [float(sum(l))/len(l) for l in zip(*recall_curve_object_list)]

        if (len(mean_recall_curve_array) < number_object_to_select_randomly ):
            mean_recall_curve_array.append(0)
        if (len(mean_precision_curve_array) < number_object_to_select_randomly ):
            mean_precision_curve_array.append(1)

        #Reshape if needed
        mean_score_objects_array =  np.reshape(mean_score_objects_array, (nb_iterations_max, -1))
        mean_error_objects_array =  np.reshape(mean_error_objects_array, (nb_iterations_max, -1))


    
        ################ GRAPHICS DRAWING ###################
        name_category = object_list_category[0]
        title = "Mean score for category : " + str(name_category) + " and for descriptor : " + str(name_descriptor) +\
            "\nNumber of objects used : " + str(len(object_list_category[1])) + \
            "\nNumber samples to select first step  : " + str(number_to_annotate_first_step) +\
            "\nNumber uncertain to select from second step  : " + str(number_to_annotate_second_step) +\
            "\nAverage Score Max : " + str(np.max(mean_score_objects_array))  + " %\n Average Current Score : " +\
             str(mean_score_objects_array[len(mean_score_objects_array)-1]) +\
              "\nTime elapsed : " + str(time_experiment_category_elapsed) + " s"
        quota = nb_iterations_max - 1
        query_num = np.arange(0, quota + 1)

        
        figure_colour=["g","c"]
        figure_data = [mean_score_objects_array, mean_positif_percent_array]
        figure_label = ['Score','Positif display']
        figure_subtitle = ['Score of SVM testing over iterations','Number of positif samples displayed to the user in the top 20']
        name_figure = ['score',"display"]

        plotting.plot_curves_2(query_num,figure_data,figure_colour,figure_label,figure_subtitle,title,name_figure,name_dataset,name_category,name_descriptor,output_graphicals_result,save_figures)
        plt.close('all')
        title = "Average Precision Score for category : " + str(name_category) + " and for descriptor : " + str(name_descriptor)
        plotting.plot_f1_precision_recall(query_num,mean_f1_score_objects_array,mean_precision_objects_array,mean_recall_objects_array,mean_recall_curve_array,mean_precision_curve_array,mean_average_precision_objects_array,name_dataset,name_category,name_descriptor,output_graphicals_result,title,save_figures)
        plt.close('all')
        #Save points
        for i in range(len(figure_data)):
            title_points = name_dataset + "_" + name_category + "_" + name_descriptor + "_mean_averageprecision.txt"
            path_to_save = output_graphicals_result + title_points
            utils.save_list_points(figure_data[i],path_to_save)
 
        #plt.show(block = False)
        
        
        #Add mean result of category to the list which will contain the results of all categories
        final_results_error_category.append(mean_error_objects_array)
        final_results_score_category.append(mean_score_objects_array)
        final_percent_positif_displayed.append(mean_positif_percent_array)
        final_percent_negatif_displayed.append(mean_negatif_percent_array)
        final_f1_score.append(mean_f1_score_objects_array)
        final_average_precision.append(mean_average_precision_objects_array)
        final_recall.append(mean_recall_objects_array)
        final_precision.append(mean_precision_objects_array)
        final_recall_curve.append(mean_recall_curve_array)
        final_precision_curve.append(mean_precision_curve_array)


    ########################################### END COMPUTATION FOR ALL THE CATEGORY ###########################################

    ## Perform mean of all the category 

    #Convert to array
    final_results_score_category = np.asarray(final_results_score_category)
    final_results_error_category = np.asarray(final_results_error_category)
    final_percent_positif_displayed = np.asarray(final_percent_positif_displayed)
    final_percent_negatif_displayed = np.asarray(final_percent_negatif_displayed)
    final_f1_score = np.asarray(final_f1_score)
    final_average_precision = np.asarray(final_average_precision)
    final_recall = np.asarray(final_recall)
    final_precision = np.asarray(final_precision)
    final_f1_score = np.asarray(final_f1_score)
    final_precision_curve = np.asarray(final_precision_curve)
    final_recall_curve = np.asarray(final_recall_curve)

    #Reshape
    final_results_score_category =  np.reshape(final_results_score_category, (len_category_dataset, -1))
    final_results_error_category =  np.reshape(final_results_error_category, (len_category_dataset, -1))
    final_percent_positif_displayed =  np.reshape(final_percent_positif_displayed, (len_category_dataset, -1))
    final_percent_negatif_displayed =  np.reshape(final_percent_negatif_displayed, (len_category_dataset, -1))
    final_f1_score  = np.reshape(final_f1_score, (len_category_dataset, -1))
    final_average_precision  = np.reshape(final_average_precision, (len_category_dataset, -1))
    final_recall  = np.reshape(final_recall, (len_category_dataset, -1))
    final_precision = np.reshape(final_precision, (len_category_dataset, -1))
    final_precision_curve = np.reshape(final_precision_curve, (len_category_dataset, -1))
    final_recall_curve = np.reshape(final_recall_curve, (len_category_dataset, -1))

    #print(final_percent_positif_displayed.shape)
    #Compute the mean of each mean result of each category
    final_mean_score_results_category_mean = np.mean(final_results_score_category,axis = 0)
    final_mean_error_results_category_mean = np.mean(final_results_error_category,axis = 0)
    final_percent_positif_displayed_mean = np.mean(final_percent_positif_displayed,axis = 0)
    final_percent_negatif_displayed_mean = np.mean(final_percent_negatif_displayed,axis = 0)
    final_f1_score_mean = np.mean(final_f1_score, axis = 0)
    final_average_precision_mean = np.mean(final_average_precision, axis = 0)
    final_recall_mean = np.mean(final_recall, axis = 0)
    final_precision_mean = np.mean(final_precision, axis = 0)
    final_precision_curve_mean = np.mean(final_precision_curve, axis = 0)
    final_recall_curve_mean = np.mean(final_recall_curve, axis = 0)



    end_total_experiment = time.time()
    time_experiment_elapsed = end_total_experiment - start_total_experiment
    logging.info('Dataset : %s | Time Elapsed : %s',name_dataset,time_experiment_elapsed)

    ################ GRAPHICS DRAWING ###################

    title = "Mean score for dataset : " + str(name_dataset) + " and for descriptor : " + str(name_descriptor) +\
            "\nNumber samples to select first step  : " + str(number_to_annotate_first_step) +\
            "\nNumber uncertain to select from second step  : " + str(number_to_annotate_second_step) +\
            "%\nAverage Score Max : " + str(np.max(final_mean_score_results_category_mean))  + " %\n Average Current Score : " +\
             str(final_mean_score_results_category_mean[len(final_mean_score_results_category_mean)-1]) + " %\n Average Current Error : " +\
              str(final_mean_error_results_category_mean[len(final_mean_error_results_category_mean)-1]) + " %" +\
              "\nTime elapsed : " + str(time_experiment_elapsed) + " s"
    quota = nb_iterations_max - 1
    query_num = np.arange(0, quota + 1)

        
   
    fig = plt.figure(figsize=(11,7))
    plt.suptitle(title, fontsize=8, fontweight='bold')

    figure_colour=["g","c"]
    figure_data = [final_mean_score_results_category_mean, final_percent_positif_displayed_mean ]
    figure_label = ['Score','Positif display']
    figure_subtitle = ['Score of SVM testing over iterations','Number of positif samples displayed to the user in the top 20']
    name_figure = ['score',"display"]

    plotting.plot_curves_final(query_num,figure_data,figure_colour,figure_label,figure_subtitle,title,name_figure,name_dataset,name_descriptor,output_graphicals_result,save_figures)

    title = "Average Precision Score for dataset : " + str(name_dataset) + " and for descriptor : " + str(name_descriptor)
    plotting.plot_f1_precision_recall_final(query_num,final_f1_score_mean,final_precision_mean,final_recall_mean,final_recall_curve_mean,final_precision_curve_mean,final_average_precision_mean[0],name_dataset,name_category,name_descriptor,output_graphicals_result,title,save_figures)

       
    plt.close('all')
    #display_selected_data_similarity_search_withGT(train_dataset_binary_labeled,train_dataset_unlabeled, path_list_train_views,idx_example_positif_selection,id_selection_diversification,save_figures)
    #plt.show(block = False)
    #print("Final result score : {}".format(final_results_score_category))
    print("Finished for the category : {} and descriptor : {}".format(name_category, name_descriptor))
    return (name_descriptor,final_results_score_category),(name_descriptor,final_percent_positif_displayed)

    
def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')



def main():
    print('The scikit-learn version is {}.'.format(sklearn.__version__))
    global nb_iterations_max, descriptor_similaritySearch,k, leaf_resolution, output_graphicals_result,select_full_object,descriptors,name_dataset
    #Ignore deprecation warninf from python
    warnings.filterwarnings("ignore", category=DeprecationWarning) 
    #Log 
    #logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s | %(message)s',datefmt='%H:%M:%S',)
    logging.basicConfig(filename = log_file,filemode='a',level=logging.DEBUG, format='%(asctime)s - %(levelname)s | %(message)s',datefmt='%H:%M:%S',)

    #Where to save the json results of similarity search programm
    output_json_result = "data/results_similarity.json"

    """
    The option we want to use for running this program
    0 --> Run interactive learning 3D without similarity search. Need to change the variable view_path_file and dataset_descriptor at the top of the program
    1 --> Run interactive learning 3D with similarity search. The user makes a query, a similarity search is performed and then interactive learning is perform manually
    Need to change the variable view_path_file and dataset_descriptor at the top of the program
    2 -->  Run interactive learning 3D with similarity search AUTOMATICALLY by submitting a specific point clous query. Then a similarity search is performed and interactive learning as well
    3 --> Run interactive learning 3D with similarity search AUTOMATICALLY. For each dataset and descriptor, a similarity search is performed and interactive learning as well. Take long time
    """
    class option_interactiveLearning:
        without_similaritySearch = 0
        similaritySearch_manuel = 1
        similaritySearch_automatic_oneobject = 2
        similaritySearch_automatic_alltesting = 3

    #Choose here the option you want
    current_option = option_interactiveLearning.similaritySearch_automatic_oneobject


    #Run basic interactive learning without similarity search
    if current_option == option_interactiveLearning.without_similaritySearch:
        print("\n [INFO] Please wait...")
        basic_interactiveLearning3DStart()

    #Run interactive learning with similarity search
    elif current_option == option_interactiveLearning.similaritySearch_manuel:
        #Check if the binary for cloud retrieval is available
        if not os.path.isfile("cloudRetrieval_main"):
            print("\n [ERROR] Cloud Retrieval main binary does not exist - Please check the path")
            logging.error("Cloud Retrieval main binary does not exist - Please check the path")
            exit(1)
        else :

            ap = argparse.ArgumentParser()
            ap.add_argument("-query", "--query", required=True, help="The query point cloud (in PCD)")
            ap.add_argument("-trained", "--trained", required=True, help="The trained dataset where the index of similarity search is located")
            ap.add_argument("-descriptor", "--descriptor", required=False, help="change descriptor. Available : esf, vfh, cvfh, ourcvfh, gshot, usc, grsd. Per default : esf")
            ap.add_argument("-output", "--output", required=False, help="where to save the json results file. Per default : results.json")
            ap.add_argument("-leaf_resolution", "--resolution", required=False, help="For cloud resolution invariance,. Per defaut : 0.01")
            ap.add_argument("-k", "--k", required=False, help="number of results to find. Per defaut : 10")
            ap.add_argument("-nbiterations", "--iterations", required=False, help="Number of iterations for automatic labelling. Per defaut : 10")
            #ap.add_argument("-automation", "--automation", type=str2bool, const=True, required=False, help="Label manually or automatically")

            args = vars(ap.parse_args())

            query_cloud = args["query"]
            trained_dataset = args["trained"]
            if args["descriptor"] is not None:
                descriptor_similaritySearch = args["descriptor"]
            if args["output"] is not None:
                output_json_result = args["output"]
            if args["resolution"] is not None:
                leaf_resolution = args["resolution"]
            if args["k"] is not None:
                k = args["k"]
            if args["iterations"] is not None:
                nb_iterations_max = args["iterations"]

            cmd = './cloudRetrieval_main -query ' + query_cloud + ' -trained ' + trained_dataset + ' -descriptor ' + descriptor_similaritySearch + ' -k ' + str(k) + ' -leaf_resolution ' + str(leaf_resolution) + ' -output ' + output_json_result + ' -compute_full ' + compute_full 
            os.system(cmd) # returns the exit status

            print("\n [INFO] Please wait...")
            logging.info("\n [INFO] Please wait...")

            similarity_search_interactiveLearning(output_json_result)

    #Run automatic interactive learning with similarity search and by sumbittming one point cloud
    elif current_option == option_interactiveLearning.similaritySearch_automatic_oneobject:

        #Check if the binary for cloud retrieval is available
        if not os.path.isfile("cloudRetrieval_main"):
            print("\n [ERROR] Cloud Retrieval main binary does not exist - Please check the path")
            logging.error("Cloud Retrieval main binary does not exist - Please check the path")
            exit(1)
        else :
            ap = argparse.ArgumentParser()
            ap.add_argument("-query", "--query", required=True, help="The query point cloud (in PCD)")
            ap.add_argument("-trained", "--trained", required=True, help="The trained dataset where the index of similarity search is located")
            ap.add_argument("-descriptor", "--descriptor", required=False, help="change descriptor. Available : esf, vfh, cvfh, ourcvfh, gshot, usc, grsd. Per default : esf")
            ap.add_argument("-output", "--output", required=False, help="where to save the json results file. Per default : results.json")
            ap.add_argument("-leaf_resolution", "--resolution", required=False, help="For cloud resolution invariance,. Per defaut : 0.01")
            ap.add_argument("-k", "--k", required=False, help="number of results to find. Per defaut : 10")
            ap.add_argument("-nbiterations", "--iterations", required=False, help="Number of iterations for automatic labelling. Per defaut : 10")
            #ap.add_argument("-automation", "--automation", type=str2bool, const=True, required=False, help="Label manually or automatically")

            args = vars(ap.parse_args())

            query_cloud = args["query"]
            trained_dataset = args["trained"]
            if args["descriptor"] is not None:
                descriptor_similaritySearch = args["descriptor"]
            if args["output"] is not None:
                output_json_result = args["output"]
            if args["resolution"] is not None:
                leaf_resolution = float(args["resolution"])
            if args["k"] is not None:
                k = args["k"]
            if args["iterations"] is not None:
                nb_iterations_max = int(args["iterations"])
            #if args["automation"] is not None:
            #    automation = args["automation"]

    
            cmd = './cloudRetrieval_main -query ' + query_cloud + ' -trained ' + trained_dataset + ' -descriptor ' + descriptor_similaritySearch + ' -k ' + str(k) + ' -leaf_resolution ' + str(leaf_resolution) + ' -output ' + output_json_result + ' -compute_full ' + compute_full 
            os.system(cmd) # returns the exit status

            print("\n [INFO] Please wait...")
            logging.info("\n [INFO] Please wait...")
      
            automatic_interactiveLearning(output_json_result,number_to_annotate_first_step,number_to_annotate_second_step,nb_iterations_max,repetition_experiment,True)

    elif current_option == option_interactiveLearning.similaritySearch_automatic_alltesting:
        use_multiprocessing = False
        num_cores = mp.cpu_count() 

        ap = argparse.ArgumentParser()
        ap.add_argument("-nbiterations", "--iterations", required=False, help="Number of iterations max for each interactive learning session (Per defaut : 20)")
        ap.add_argument("-multiprocessing", "--multiprocessing", type=str2bool, required=False, help="If run the program in one cpu or multiple (Per defaut : True)")
        ap.add_argument("-cores", "--numcores", required=False, help="Number of cores to use (Per defaut : cpu_count()) ")
       
        args = vars(ap.parse_args())

        if args["iterations"] is not None:
            nb_iterations_max = int(args["iterations"])
        if args["multiprocessing"] is not None:
            use_multiprocessing = args["multiprocessing"]
        if args["numcores"] is not None:
            num_cores = int(args["numcores"])
        
        print("\n [INFO] Please wait...")

        #Check if the binary for cloud retrieval is available
        if not os.path.isfile("cloudRetrieval_main"):
            print("Error")
            logging.error("Cloud Retrieval main binary does not exist - Please check the path")
            exit(1)
        else :
            pool = mp.Pool(processes=num_cores) 

            path_dataset = os.path.abspath(os.path.join(training_file_full, os.pardir))
            print(path_dataset)
            print("Number to select randomly : {}".format(number_object_to_select_randomly))
            
            list_of_random_objects,category_list_name_array = utils.get_random_objects_from_dataset(path_dataset,number_object_to_select_randomly,select_full_object,"ply")

            if (len(list_of_random_objects) == 0):
                logging.error("List of objects is empty")
                print("[ERROR] List of objects is empty")

            if (use_multiprocessing):
                #params = (list_of_random_objects, trained_dataset, name_dataset)
                params = (list_of_random_objects, path_dataset, name_dataset)
                func = partial(automatic_interactiveLearning_objectlist, params)
                print(params)
                print(descriptors)
                results_accuracy_score,results_positif_display = zip(*pool.map(func, descriptors))

                #print("Completed....Going to draw final results...")
                #results = pool.map(func, descriptors)


                results_accuracy_score = list(results_accuracy_score)
                results_positif_display = list(results_positif_display)
                 
                print(results_accuracy_score)
                print(len(results_accuracy_score))
                score = results_accuracy_score[1][1]
                positif_disp = results_positif_display[1][1]

                score_array = list()
                for i in range(len(results_accuracy_score)):
                    item = results_accuracy_score[i]
                    score = item[1]
                    score = score[:,score.shape[1]-1]
                    score_array.append(score)

                #print("Score Array :  {}".format(score_array))
                score_display_array = list()
                for i in range(len(results_positif_display)):
                    item = results_positif_display[i]
                    score = item[1]
                    score = score[:,score.shape[1]-1]
                    score_display_array.append(score)
                #print("Categories :  {}".format(category_list_name_array))
                #print("Descriptors :  {}".format(descriptors))
                title = "Score by categories | Number iterations : " + str(nb_iterations_max)
                #figure = plotting.plot_bar(descriptors,category_list_name_array,score_array,title)
                figure = plotting.plot_points(descriptors,category_list_name_array,score_array,title)

                output_graphicals_result = "results_graphics"+name_dataset+"/"

                name_save_figure = output_graphicals_result + name_dataset + "_score_accuracy_bar.png"
                print("1.Saving to {}".format(name_save_figure))
                figure.savefig(name_save_figure)
                #Save points
                title_points = output_graphicals_result + name_dataset + "_score_accuracy_bar.txt"
                utils.save_list_points(score_array,title_points)

                title = "Positif display by categories | Number iterations : " + str(nb_iterations_max)
                #figure1 = plotting.plot_bar(descriptors,category_list_name_array,score_display_array,title)
                figure1 = plotting.plot_points(descriptors,category_list_name_array,score_display_array,title)
                name_save_figure = output_graphicals_result + name_dataset + "_score_display_bar.png"

                #Save points
                title_points = output_graphicals_result + name_dataset + "_score_display_bar.txt"
                utils.save_list_points(score_display_array,title_points)

                print("2.Saving to {}".format(name_save_figure))
                figure1.savefig(name_save_figure)

    else :
        logging.error('Invalid Interactive Learning Option')
        exit(1)

    

if __name__ == '__main__':
    main()


