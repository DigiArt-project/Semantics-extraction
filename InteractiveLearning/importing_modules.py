import numpy as np
import copy
import os
from os.path import basename

#divers
import csv
import time
import warnings
import numpy as np
import argparse
import json
import subprocess
import math
from math import fabs
import random
from collections import OrderedDict
from pathlib import Path
#Itertools
import itertools
from itertools import product

#Interactive learning
from base.dataset import Dataset, import_libsvm_sparse
from models import *
from query_strategies import *
from labelers import IdealLabeler


#Skelearn
#from sklearn.cross_validation import train_test_split
import sklearn
from sklearn.preprocessing import LabelBinarizer
from sklearn.utils import check_X_y
from sklearn.cross_validation import train_test_split
from sklearn.preprocessing import StandardScaler,MinMaxScaler,Normalizer
from sklearn.metrics import precision_score,average_precision_score,classification_report,confusion_matrix
from sklearn.decomposition import PCA,KernelPCA,TruncatedSVD,SparsePCA,NMF,IncrementalPCA,FastICA
from sklearn import svm, datasets
from sklearn.metrics import f1_score,precision_recall_curve,precision_score,recall_score

#Multiprocessing
import multiprocessing as mp
from functools import partial
import itertools
from collections import OrderedDict


#Log
import logging

#Matplotlib
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.image import AxesImage
import matplotlib.gridspec as gridspec
import matplotlib.ticker as plticker
import matplotlib.ticker as plticker


