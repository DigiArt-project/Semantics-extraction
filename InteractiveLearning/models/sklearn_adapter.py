"""scikit-learn classifier adapter
"""
from base.interfaces import Model, ContinuousModel, ProbabilisticModel


class SklearnAdapter(Model):
    def __init__(self, clf):
        self._model = clf

    def train(self, dataset, *args, **kwargs):
        return self._model.fit(*(dataset.format_sklearn() + args), **kwargs)

    def predict(self, feature, *args, **kwargs):
        return self._model.predict(feature, *args, **kwargs)

    def score(self, testing_dataset, *args, **kwargs):
        return self._model.score(*(testing_dataset.format_sklearn() + args),
                                **kwargs)


class SklearnProbaAdapter(ProbabilisticModel):

    def __init__(self, clf):
        self._model = clf

    def train(self, dataset, *args, **kwargs):
        return self._model.fit(*(dataset.format_sklearn() + args), **kwargs)

    def predict(self, feature, *args, **kwargs):
        return self._model.predict(feature, *args, **kwargs)

    def score(self, testing_dataset, *args, **kwargs):
        return self._model.score(*(testing_dataset.format_sklearn() + args),
                                **kwargs)

    def predict_real(self, feature, *args, **kwargs):
        return self._model.predict_proba(feature, *args, **kwargs) * 2 - 1

    def predict_proba(self, feature, *args, **kwargs):
        return self._model.predict_proba(feature, *args, **kwargs)
