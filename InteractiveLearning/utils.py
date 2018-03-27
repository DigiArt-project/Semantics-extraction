from importing_modules import *
import re


__all__ = ['inherit_docstring_from', 'seed_random_state', 'zip']

zip = zip

"""
Check is a string is encoded in ASCII or not
"""
def is_ascii(text):
    if isinstance(text, unicode):
        try:
            text.encode('ascii')
        except UnicodeEncodeError:
            return False
    else:
        try:
            text.decode('ascii')
        except UnicodeDecodeError:
            return False
    return True



def startTimer():
    start = time.time()
    return start

def stopTimer():
    stop = time.stop()
    return stop

def getTime(start,end):
    time_experiment_elapsed = end - start
    print("Time elapsed : {}".format(time_experiment_elapsed))


    #Take items closest to 0
"""
Given an array, compute the absolute value and get the values in ascending order. This functions allows to take value closest to zero
"""
def take_closest_to_zero(data):

    tmp = data

    absolute_score = abs(np.array(tmp))
    sorted_values_ascending = sorted(absolute_score,reverse = False)
    multiple_id_ranking_ascending =  np.argsort(np.array(absolute_score))

    return multiple_id_ranking_ascending

"""
Sort data array in ascending order (smallest to biggest)
if get_negative_only activated, return only the sorted array with only negatives value. Discard value above zero
"""
def sort_ascending_order(data,get_negative_only = False):
    tmp = copy.copy(data)
    
    sorted_indice = np.argsort(np.array(data))
    sorted_values = sorted(tmp,reverse = False)
    if get_negative_only:
        sorted_values = [elem for elem in sorted_values if elem < 0]
        sorted_indice = sorted_indice[:len(sorted_values)]

    return sorted_indice, sorted_values

"""
Sort data array in descending order (biggest to smallest)
if get_positive_only activated, return only the sorted array with only positive value. Discard value below zero
"""
def sort_descending_order(data,get_positive_only = False):
    tmp = copy.copy(data)
    sorted_indice_descending_order = np.array(tmp).argsort()[::-1]
    sorted_values_descending_order = sorted(tmp,reverse = True)
    #print("Sorted value descending order : {}" .format(sorted_values_descending_order))
    if get_positive_only:
        sorted_values_descending_order = [elem for elem in sorted_values_descending_order if elem > 0]
        sorted_indice_descending_order = sorted_indice_descending_order[:len(sorted_values_descending_order)]
    
    return sorted_indice_descending_order, sorted_values_descending_order



"""
From a specific txt file, get the categories
"""
def get_categories_from_file(path):
    dataset_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), path)
    categories = list()
    with open(dataset_filepath, "r") as ins:
        #array = []
        i = 1
        for line in ins:
            #print(line)
            if "/" not in line: 
                #remove carriage
                line = line.rstrip('\r\n')
                t1 = (line,i)
                categories.append(t1)
                i = i + 1
    return categories

"""
Assume you have something like this :
/Users/user/digiArt/Tools/cat10_descriptors/car/descriptors/alldesc4/desc_car0_8_esfgshotgrsdvfh.txt
we want to get only the name : car0_8 i.e car0, view 8
"""
def get_filename_descriptor_from_path(path,number_to_split = 9):
    #print(path)
    filename = path.split("/")[number_to_split]
    filename_test = path.split("/")
    #print(filename)
    #print(filename_test)
    
    filename = filename.split('_',1)[1]
    filename = filename.rsplit('_',1)[0]
    #Remove extension
    filename = os.path.splitext(filename)[0]

    return filename

"""
Assume you have something like this :
/Users/user/digiArt/Tools/cat10_descriptors/car/descriptors/alldesc4/desc_car0_8_esfgshotgrsdvfh.txt
we want to get only the category : car
"""
def get_category_name_from_path(path,number_to_split = 6):
    filename = path.split("/")[number_to_split]



 
"""
First loop over category folder in the dataset
Assume that the structure of each dataset follows this structure : dataset/category_i/object.pcd/ply
In each category, select maximum 'number_object_to_select_randomly' pcd file. If the number object to select is > to the number of object inside the category, reduce
the numer automatically
"""    
def get_random_objects_from_dataset2(directory,number_object_to_select_randomly,extension = "pcd"):
    #For each category inside the dataset
    result_list_object_random = list()
    result_list_categories = list()
    list_of_random_items = 0
    category_list = list()
    for category in os.listdir(directory):
        result_list_categories.append(category)
        pattern = re.compile("training_*")
        if (not pattern.match(category) and not category.startswith('.') and category != "libsvmdata"):
            category_list.append(category)
            path = directory+ "/"+ category
            #print("path {}".format(path))
            if os.path.isdir(path):
                #Print over category
                filename_array = []
                for filename in os.listdir(path):
                    extension = "."+extension
                    if filename.endswith(extension):
                        #print(os.path.join(path, filename))
                        filename_array.append(os.path.join(path, filename))
                reduce_number = 1
                if (number_object_to_select_randomly > len(filename_array)):
                    tmp_number_object_to_select = number_object_to_select_randomly 
                    while (tmp_number_object_to_select > len(filename_array) ):
                        tmp_number_object_to_select = tmp_number_object_to_select - reduce_number
                        reduce_number = reduce_number + 1
                    if (tmp_number_object_to_select == 0):
                        tmp_number_object_to_select = 1
                    list_of_random_items = random.sample(filename_array, tmp_number_object_to_select)

                    #print("\n Random choice for category {} : {} ".format(category,list_of_random_items))
                    result_list_object_random.append((category,list_of_random_items))
                else :
                    list_of_random_items = random.sample(filename_array, number_object_to_select_randomly)
                    result_list_object_random.append((category,list_of_random_items))
          

        #print("\n Random choice for category {} : {} ".format(category,list_of_random_items))
        #logging.debug('\n Random choice for category %s : %s ',category,list_of_random_items )
    #print(type(result_list_object_random))
    return result_list_object_random,category_list

def get_all_objects_from_dataset(path_dataset,full_object = True,extension = "pcd"):
    #For each category inside the dataset
    result_list_object_all= list()
    result_list_categories = list()
    result_list_numberObjectsPerClass = list()
    list_of_random_items = 0
    category_list = list()
    extension = "."+extension
    type_folder = ""
    if full_object:
        type_folder="/full"
    else:
        type_folder="/views"
    for root, directories, filenames in os.walk(path_dataset):
        for directory in directories:
            directory = os.path.join(root, directory)
            
            parent_dir_category = os.path.abspath(os.path.join(directory, os.pardir))
            name_category = basename(parent_dir_category)
            
            if type_folder in directory and not "/descriptors" in directory:
                category_list.append(name_category)
                #print(name_category)
                #print(directory)
                #Print over category
                filename_array = []
                numberOnlyFile = next(os.walk(directory))[2] #dir is your directory path as string
                result_list_numberObjectsPerClass.append(len(numberOnlyFile))
                for filename in os.listdir(directory):
                    #print(filename)
                    if filename.endswith(extension):
                        #print(os.path.join(directory, filename))
                        filename_array.append(os.path.join(directory, filename))
                        #print(filename_array)
                
                result_list_object_all.append((name_category,filename_array))
                
          

        #print("\n Random choice for category {} : {} ".format(category,list_of_random_items))
        #logging.debug('\n Random choice for category %s : %s ',category,list_of_random_items )
    #print(result_list_object_random)
    #print(result_list_numberObjectsPerClass)
    return result_list_object_all,category_list,result_list_numberObjectsPerClass


def get_random_objects_from_dataset(path_dataset,number_object_to_select_randomly,full_object = True,extension = "pcd"):
    #For each category inside the dataset
    result_list_object_random = list()
    result_list_categories = list()
    result_list_numberObjectsPerClass = list()
    list_of_random_items = 0
    category_list = list()
    extension = "."+extension
    type_folder = ""
    if full_object:
        type_folder="/full"
    else:
        type_folder="/views"
    for root, directories, filenames in os.walk(path_dataset):
        for directory in directories:
            directory = os.path.join(root, directory)
            parent_dir_category = os.path.abspath(os.path.join(directory, os.pardir))
            name_category = basename(parent_dir_category)
            
            if type_folder in directory and not "/descriptors" in directory:
                category_list.append(name_category)
                #print(name_category)
                #print(directory)
                #Print over category
                filename_array = []
                numberOnlyFile = next(os.walk(directory))[2] #dir is your directory path as string
                result_list_numberObjectsPerClass.append(len(numberOnlyFile))
                for filename in os.listdir(directory):
                    #print(filename)
                    if filename.endswith(extension):
                        #print(os.path.join(directory, filename))
                        filename_array.append(os.path.join(directory, filename))
                reduce_number = 1
                if (number_object_to_select_randomly > len(filename_array)):
                    tmp_number_object_to_select = number_object_to_select_randomly 
                    while (tmp_number_object_to_select > len(filename_array) ):
                        tmp_number_object_to_select = tmp_number_object_to_select - reduce_number
                        reduce_number = reduce_number + 1
                    if (tmp_number_object_to_select == 0):
                        tmp_number_object_to_select = 1
                    list_of_random_items = random.sample(filename_array, tmp_number_object_to_select)

                    #print("\n Random choice for category {} : {} ".format(category,list_of_random_items))
                    result_list_object_random.append((name_category,list_of_random_items))
                else :
                    list_of_random_items = random.sample(filename_array, number_object_to_select_randomly)
                    result_list_object_random.append((name_category,list_of_random_items))
          

        #print("\n Random choice for category {} : {} ".format(category,list_of_random_items))
        #logging.debug('\n Random choice for category %s : %s ',category,list_of_random_items )
    #print(result_list_object_random)
    #print(result_list_numberObjectsPerClass)
    return result_list_object_random,category_list,result_list_numberObjectsPerClass


def countNumberLines(file):
    if os.path.isfile(file):
        with open(file) as f:
            nbLine = len(f.readlines())
            return nbLine
    else :
        logging.error("Impossible to read the file - CountNumberLines()")
        return 0

def save_list_points(input_list,output):
    input_list = np.asarray(input_list)
    dimension_array = input_list.ndim
    if (dimension_array > 1):
        with open(output, 'w') as outfile:
            outfile.write('# Array shape: {0}\n'.format(input_list.shape))
            writer = csv.writer(outfile, delimiter=' ')
            writer.writerows(input_list)
    else :
        file = open(output, 'w')
        file.write('# Array shape: {0}\n'.format(input_list.shape))
        for item in input_list:
            file.write("%s " % item)

    logging.debug("[Saving] Points have been saved to %s", output)

def getYdataFromDataset(dataset):
    y =[entry[1] for idx, entry in enumerate(dataset.data)]
    return y



"""
Given a json file which contain the results of the similarity search, extract from the path the name of the object
So that we can link the categories to interactive learning
"""
def extract_names_objects_from_result_json(result_jsonfile):
    path_array_results = []
    categorie_array_results = []
    with open(result_jsonfile, 'r') as jsonfile:
        array = json.load(jsonfile)

        #print (array)
        for line in array:
            path_array_results.append(line["path"])
            #Get the filename from path
            filename = os.path.basename(line["path"])
            #The initial name is desc_category.pcd. We keep only the part after desc_
            filename = filename.split('_',1)[1]
            #Remove extension (.pcd)
            filename = filename.split(".")[0]
            #print(filename)
            categorie_array_results.append(filename)

    return categorie_array_results

"""
Given a json file which contain the results of the similarity search, extract the category of each result
So that we can link the categories to interactive learning
"""
def extract_categories_from_result_json(result_jsonfile):
    path_array_results = []
    categorie_array_results = []
    with open(result_jsonfile, 'r') as jsonfile:
        array = json.load(jsonfile)

        #print (array)
        for line in array:
            #Get the tag corresponding to the category
            categorie_array_results.append(line["Category"])
           

    return categorie_array_results

"""
From a file of descriptors and another file which contains its path to views, create the database
At the end we have 
- the fully dataset object : contains features X + corresponding path, and its label
- train_dataset_unlabeled object : contains the train part Features X + label unknown
- feature_test : features X from the test set
- y_test : label from the test set
- y_train : label from the train set
- fully_labeled_train_dataset object : fully dataset features X + label y
"""
def split_train_test_from_libsvm_data(dataset_filepath,view_path_file, test_size_split):
    start = time.time()
    X, y = import_libsvm_sparse(dataset_filepath).format_sklearn()
    X = MinMaxScaler().fit_transform(X)
    end = time.time()
    print("Time elapsed import libsvm : {} s".format(end - start))
    #print(y)
    #print(X)
    test = ""
    with open(view_path_file, "r") as ins:
        #array = []
        array = np.chararray(len(X),itemsize="1500")
        test_list = list()
        #print(len(array))
        i = 0
        for line in ins:
            if "ply" in line or "pcd" in line or "txt" in line: 
                #print(line.strip())
                array[i]=str(line.strip())
                test_list.append(str(line.strip()))
                #array.append(line.strip()) 
                #array[i] = array[i].decode("utf-8")
                #print(array[i].decode("utf-8"))
                test = line.strip()

                i = i + 1

    descripteur_and_path_order = OrderedDict(zip(test_list, X))
    X_dataset_with_path = list(zip(X,  test_list))

    fully_dataset = Dataset(X_dataset_with_path, y)

    #random.shuffle(X_dataset_with_path)
    #print("Number descriptors : {}" .format(len(X_dataset_with_path)))
    X_train, X_test, y_train, y_test = train_test_split(X_dataset_with_path, y, test_size=test_size_split)

    #print("[INFO] X_train  {}".format(X_train))
    #print("[INFO] Y_train  {}".format(y_train))
  
    path_list_train_views = list()
    feature_train = list()
    for i in X_train:
        #print(i[1])
        path_list_train_views.append(i[1])
        feature_train.append(i[0])

    path_list_test_views = list()
    feature_test = list()
    for i in X_test:
        path_list_test_views.append(i[1])
        feature_test.append(i[0])

    feature_test = np.array(feature_test)
     
    y_train_unlabeled = np.concatenate([y_train[:0], [None] * (len(y_train))])

    train_dataset_unlabeled = Dataset(feature_train, y_train_unlabeled)
    test_dataset_labeled = Dataset(feature_test, y_test)
    
    #HAs been changed
    #fully_labeled_trn_ds = Dataset(X_train, y_train)
    fully_labeled_train_dataset = Dataset(X_train, y_train)


    #On cree une nouvelle liste qui contiendra trois elements (entry, path, feature)
    newList_X = list()
    for idx, entry in enumerate(X_train):
        #idx is the entry, entry[0] correspond to the feature of the array (which contains two entries : feature and path), and entry[1] correspond to the path
        newList_X.append((idx,entry[1],entry[0]))

    
    #return unlabeled train dataset, test dataset, y label train, labeled train dataset, path to views training, path to views testing
    return fully_dataset,train_dataset_unlabeled, feature_test,y_test, y_train, fully_labeled_train_dataset, path_list_train_views,path_list_test_views


"""
Replicate the train_test_split function of scikit but with a cariation
What we want at the end is a training set (X_train and y_train) and a test set (X_test and y_test)
But we add a constraint. We get a category_array_result which an array which contain the categories found by similarity search.
We want to be sure that all the object perteining to the categories belong to the train set and not test set. 
So if an object belong to test set after splitting, we remove from it and we add it to the train set
"""
def train_test_split_according_to_categories(X, y,categorie_array_results,test_size = 0.33):

    size_array_total = len(X)

    #print("Size X total : {}".format(size_array_total))
    logging.debug(' Size X total : %s  ', size_array_total)
    size_test_data = math.ceil(test_size*size_array_total)
    size_train_data = size_array_total - size_test_data

    c = list(zip(X,y))
    random.shuffle(c)

    X, y = zip(*c)

    X_train = list(X[:size_train_data])
    X_test = list(X[size_train_data:])
    y_train = list(y[:size_train_data])
    y_test = list(y[size_train_data:])


    logging.debug('[INFO] Size X_train before : %s  ', len(X_train))
    logging.debug('[INFO] Size X_test before : %s  ', len(X_test))
    list_result = list()
    for idx1,path1 in enumerate(categorie_array_results):
        for idx2,path2 in enumerate(X_test):
            filename = os.path.basename(path2[1])
            #The initial name is desc_category.pcd. We keep only the part after desc_
            filename = filename.split('_',1)[1]
            #Remove extension
            filename = os.path.splitext(filename)[0]

            #Check if there is additional data that we don't need . Ex donut3_0_esfgshotgrsdvfh , we want to remove _esfgshotgrsdvfh
            #print(filename)
            if (filename.count('_') == 2):
                filename = filename.rsplit('_',1)[0]
                #print(filename)
            #print(path2)
            #Keep only the name
            #filename = filename.rsplit('_',1)[0]
            if (path1 == filename):
                if path1 not in list_result:
                    list_result.append(path1)
                    #Add to train set
                    X_train.append(X_test[idx2])
                    y_train.append(y_test[idx2])

                    #Remove from test set
                    del X_test[idx2]
                    del y_test[idx2]

    
    return np.asarray(X_train), np.asarray(X_test), np.asarray(y_train), np.asarray(y_test)

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


def get_testDataset_binary(X_test,y_test,view_path_file):
    #Get categories name and associated label from file
    categories_list = get_categories_from_file(view_path_file)
    banner = "[For Testing] Enter the positive category by its number: \n"
    banner += str(categories_list) + ' ' 
    category_label = input(banner)
    y_test_binary= [1 if y == int(category_label) else -1 for y in y_test]
    
    y_test_binary = np.array(y_test_binary)
    #print("[INFO] Y_test binary {}".format(y_test_binary))
    #print("X test  {}".format(X_test))
    
    tst_ds = Dataset(X_test, y_test_binary)
    #print(tst_ds.data)

    return tst_ds,X_test,y_test_binary,category_label

def get_train_label_binary(y_train,category_label):
    #Get categories name and associated label from file
    
    y_train_binary= [1 if y == int(category_label) else -1 for y in y_train]
    #print("[INFO] Y_train binary {}".format(y_train_binary))
    y_train_binary = np.array(y_train_binary)
    #print("X test  {}".format(X_test))
    #print("[INFO] Y_train binary {}".format(y_train_binary))

    return y_train_binary

def minimum_spanning_tree(X, copy_X=True):
    """X are edge weights of fully connected graph"""
    if copy_X:
        X = X.copy()

    if X.shape[0] != X.shape[1]:
        raise ValueError("X needs to be square matrix of edge weights")
    n_vertices = X.shape[0]
    spanning_edges = []

    # initialize with node 0:
    visited_vertices = [0]
    num_visited = 1
    # exclude self connections:
    diag_indices = np.arange(n_vertices)
    X[diag_indices, diag_indices] = np.inf

    while num_visited != n_vertices:
        new_edge = np.argmin(X[visited_vertices], axis=None)
        # 2d encoding of new_edge from flat, get correct indices
        new_edge = divmod(new_edge, n_vertices)
        new_edge = [visited_vertices[new_edge[0]], new_edge[1]]
        # add edge to tree
        spanning_edges.append(new_edge)
        visited_vertices.append(new_edge[1])
        # remove all edges inside current tree
        X[visited_vertices, new_edge[1]] = np.inf
        X[new_edge[1], visited_vertices] = np.inf
        num_visited += 1
    return np.vstack(spanning_edges)


def polar_to_cartesian(arr, r):
    a = np.concatenate((np.array([2 * np.pi]), arr))
    si = np.sin(a)
    si[0] = 1
    si = np.cumprod(si)
    co = np.cos(a)
    co = np.roll(co, -1)
    return si * co * r
