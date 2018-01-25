import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as plticker


#Interval matplotlib
reg_interval_y = plticker.MultipleLocator(base=10.0) # this locator puts ticks at regular intervals

def autolabel(ax,rects):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                '%d %%' % int(height),bbox=dict(boxstyle="round4", fc="w"),
                ha='center', va='bottom',color='blue', fontweight='bold')


def main(file,name_descriptor,directory_save):
    count = 0
    category_list_name_array = list()
    axis_x = ['k1','k5','k10','k15','k20','Average']
    with open(file) as f:
        content = f.readlines()
        
        for line in content:
            if (count > 0):
                line = line.strip()
                scores = line.split()
                if len(line.strip()) != 0 :
                    name_category = scores[0]
                    category_list_name_array.append(name_category)
                    print("\n",scores[0])

                    del(scores[0])
                    #Convert all string to int
                    scores = list(map(float, scores))
                    print(scores)


                    ##### Draw
                    bar_width = 0.25
                    opacity = 0.7
                    fig = plt.figure(figsize=(8, 8))
                    ax = fig.add_subplot(111)
                    #To preserve order of axis c
                    #ax.set_xticklabels(axis_x)

                    D = {u'k1':26, u'k5': 17, u'k10':30}
                    
                    #keys = range(len(axis_x))
                    dicts = dict(zip(axis_x, scores))
                    barlist = plt.bar(range(len(dicts)), dicts.values(), bar_width,align='center',alpha = opacity, color = 'b',label = 'Score')
                    plt.xticks(range(len(dicts)), dicts.keys())
                    barlist[5].set_color('g')
                    barlist[5].set_linewidth(4)
                    barlist[5].set_width(0.50)

                    title = ax.set_title("Descriptor {} | Average precision similarity search for category : {}".format(name_descriptor,name_category),fontsize=14, fontweight='bold')
                    title.set_position([.5, 1.05])
                    autolabel(ax,barlist)
                   
                    ax.set_xlabel('Knn')
                    ax.set_ylabel('Scores')
                    ax.set_ylim([0, 110])
                    ax.yaxis.set_major_locator(reg_interval_y)
                    #plt.legend()
                    plt.tight_layout()


                    title_figure = name_descriptor + "_" + name_category + "_similarity_plot.png"
                    paht_to_save = directory_save + title_figure
                    fig.savefig(paht_to_save, bbox_inches='tight')

                    #plt.show(block = False)  
                    plt.close()  


            count = count + 1


    print("Category list : {}".format(category_list_name_array))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-file", "--file", required=True, help="File which contains the score for similarity search")
    ap.add_argument("-save", "--save", required=True, help="Directory to save the images")
    args = vars(ap.parse_args())

    file = args["file"]
    
    base=os.path.basename(file)
    name_descriptor = base.split("_")[0]

    directory_save = args["save"]
    main(file,name_descriptor,directory_save)