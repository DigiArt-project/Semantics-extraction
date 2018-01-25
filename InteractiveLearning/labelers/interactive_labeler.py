"""Interactive Labeler

This module includes an InteractiveLabeler.
"""
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from six.moves import input

from libact.base.interfaces import Labeler
from libact.utils import inherit_docstring_from


class InteractiveLabeler(Labeler):

    """Interactive Labeler

    InteractiveLabeler is a Labeler object that shows the feature through image
    using matplotlib and lets human label each feature through command line
    interface.

    Parameters
    ----------
    label_name: list
        Let the label space be from 0 to len(label_name)-1, this list
        corresponds to each label's name.

    """

    def __init__(self, **kwargs):
        self.label_name = kwargs.pop('label_name', None)

    @inherit_docstring_from(Labeler)
    def label(self, feature):
        plt.imshow(feature, cmap=plt.cm.gray_r, interpolation='nearest')
        plt.draw()

        banner = "Enter the associated label with the image: "

        if self.label_name is not None:
            banner += str(self.label_name) + ' '

        lbl = input(banner)

        while (self.label_name is not None) and (lbl not in self.label_name):
            print('Invalid label, please re-enter the associated label.')
            lbl = input(banner)

        return self.label_name.index(lbl)

    def label_multiple(self, img_plot_tab,train_dataset, ask_id_multiple):
        label_name_list = list()
        count = 0
        for j in ask_id_multiple:
            print("J : {}".format(j))

            feature = train_dataset.data[j][0].reshape(8, 8)
            img_plot_tab[count].imshow(feature, cmap=plt.cm.gray_r, interpolation='nearest')
            plt.axis('off')
            img_plot_tab[count].set_axis_off()
            plt.draw()
            if self.label_name is not None:
                banner = "Enter the associated label with the images: "
                banner += str(self.label_name) + ' '

            lbl = input(banner)
            if (lbl  in self.label_name):
                label_name_list.append(self.label_name.index(lbl))
                print("Label name list contains now : {} ".format(label_name_list))

            while (self.label_name is not None) and (lbl not in self.label_name):
                print('Invalid label, please re-enter the associated label.')
                lbl = input(banner)
                label_name_list.append(self.label_name.index(lbl))
            img_plot_tab[count].cla()
            plt.axis('off')
            img_plot_tab[count].set_axis_off()
            plt.draw()
            count = count + 1

        return label_name_list
