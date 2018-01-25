""" Uncertainty Sampling

This module contains a class that implements two of the most well-known
uncertainty sampling query strategies: the least confidence method and the
smallest margin method (margin sampling).

"""
import numpy as np

from base.interfaces import QueryStrategy, ContinuousModel, \
    ProbabilisticModel

zip = zip


class UncertaintySampling(QueryStrategy):

    def __init__(self, *args, **kwargs):
        super(UncertaintySampling, self).__init__(*args, **kwargs)

        self.model = kwargs.pop('model', None)
        if self.model is None:
            raise TypeError(
                "__init__() missing required keyword-only argument: 'model'"
            )
        if not isinstance(self.model, ContinuousModel) and \
                not isinstance(self.model, ProbabilisticModel):
            raise TypeError(
                "model has to be a ContinuousModel or ProbabilisticModel"
            )

        self.model.train(self.dataset)

        self.method = kwargs.pop('method', 'lc')
        if self.method not in ['lc', 'sm', 'entropy']:
            raise TypeError(
                "supported methods are ['lc', 'sm', 'entropy'], the given one "
                "is: " + self.method
            )

        if self.method=='entropy' and \
                not isinstance(self.model, ProbabilisticModel):
            raise TypeError(
                "method 'entropy' requires model to be a ProbabilisticModel"
            )

    def make_query(self, return_score=False):
        """Return the index of the sample to be queried and labeled and
        selection score of each sample. Read-only.

        No modification to the internal states.

        Returns
        -------
        ask_id : int
            The index of the next unlabeled sample to be queried and labeled.

        score : list of (index, score) tuple
            Selection score of unlabled entries, the larger the better.

        """
        dataset = self.dataset
        self.model.train(dataset)

        unlabeled_entry_ids, X_pool = zip(*dataset.get_unlabeled_entries())

        if isinstance(self.model, ProbabilisticModel):
            dvalue = self.model.predict_proba(X_pool)
        elif isinstance(self.model, ContinuousModel):
            dvalue = self.model.predict_real(X_pool)

        if self.method == 'lc':  # least confident
            score = -np.max(dvalue, axis=1)

        elif self.method == 'sm':  # smallest margin
            if np.shape(dvalue)[1] > 2:
                # Find 2 largest decision values
                dvalue = -(np.partition(-dvalue, 2, axis=1)[:, :2])
            score = -np.abs(dvalue[:, 0] - dvalue[:, 1])

        elif self.method == 'entropy':
            score = np.sum(-dvalue * np.log(dvalue), axis=1)

        ask_id = np.argmax(score)

        if return_score:
            return unlabeled_entry_ids[ask_id], \
                   list(zip(unlabeled_entry_ids, score))
        else:
            return unlabeled_entry_ids[ask_id]

    def make_query_multi(self,number,return_score=False):
        dataset = self.dataset
        self.model.train(dataset)

        unlabeled_entry_ids, X_pool = zip(*dataset.get_unlabeled_entries())

        if isinstance(self.model, ProbabilisticModel):
            dvalue = self.model.predict_proba(X_pool)
        elif isinstance(self.model, ContinuousModel):
            dvalue = self.model.predict_real(X_pool)

        if self.method == 'lc':  # least confident
            #np.max --> return the maximum along the first axis.
            #Ici retourne une liste contenant la probabilité maximimal de chaque classe pour chque feature
            #print("NP MAX : {} ".format(np.max(dvalue, axis=1)))
            #axis =1 , prend le maximum de chaque list et les concatene en une seule
            #list1=[[3,4,5],[2,10,20],[5,5,30]] ==> np.max(list1,axis=1) ==> array([ 5, 20, 30])
            #argmin Returns the indices of the minimum values along an axis.
            #Least confident strategy --> On prend le label dont la probabilité est le moins sûr 
            ask_id = np.argmin(np.max(dvalue, axis=1))
            #Return the n smallest element, here n =3
            ask_multiple_id = np.argsort(np.max(dvalue, axis=1))[:number]
            #print("ID ask : {} ".format(ask_id))
            #print("The {} less sure id about the label : {} ".format(number, ask_multiple_id))

        elif self.method == 'sm':  # smallest margin
            if np.shape(dvalue)[1] > 2:
                # Find 2 largest decision values
                dvalue = -(np.partition(-dvalue, 2, axis=1)[:, :2])
            margin = np.abs(dvalue[:, 0] - dvalue[:, 1])
            ask_id = np.argmin(margin)

        elif self.method == 'entropy':
            entropy = np.sum(-dvalue * np.log(dvalue), axis=1)
            ask_id = np.argmax(entropy)
            ask_multiple_id = np.argsort(np.argmax(entropy))[:number]
        #Return an array formed from the elements of a at the given indices
        if return_score:
            return np.take(unlabeled_entry_ids,ask_multiple_id), \
                   list(zip(unlabeled_entry_ids, score))
        else:
            return np.take(unlabeled_entry_ids,ask_multiple_id)

