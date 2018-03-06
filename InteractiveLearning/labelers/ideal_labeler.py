"""
Ideal/Noiseless labeler that returns true label

"""
import numpy as np

from base.interfaces import Labeler


class IdealLabeler(Labeler):

    """
    Provide the errorless/noiseless label to any feature vectors being queried.

    Parameters
    ----------
    dataset: Dataset object
        Dataset object with the ground-truth label for each sample.

    """

    def __init__(self, dataset, **kwargs):
        X, y = zip(*dataset.get_entries())
        # make sure the input dataset is fully labeled
        assert (np.array(y) != np.array(None)).all()
        self.X = X
        self.y = y

    def label(self, feature):
        return self.y[np.where([np.array_equal(x, feature)
                                for x in self.X])[0][0]]


    def multiple_label_query(self, feature_list):
        label_list_GT = list()
        for feature in feature_list:
           #print("Feature {}".format(feature))
            result = self.y[np.where([np.array_equal(x, feature) for x in self.X])[0][0]]
            label_list_GT.append(result)

        #print("Label list groundtruth contains {} label : {} ".format(len(label_list_GT),label_list_GT))
        return label_list_GT


