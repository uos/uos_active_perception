# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

from scipy import stats
import pylab
import numpy as np
import sys
import common

from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Computer Modern Sans Serif']})
rc('text', usetex=True)


def main():
    if len(sys.argv) != 2:
        print "Need exactly one .tab file as argument"
        sys.exit()

    print "Evaluating", sys.argv[1:]

    f = open(sys.argv[1])
    costs = []
    times = []
    for (c,t,s) in [line.split('\t') for line in f.readlines()[1:]]:
        if s[:-1] == "success":
            costs.append(float(c))
            times.append(float(t))
        else:
            print "skipped", s[:-1]

    slope, intercept, r_value, p_value, std_err = stats.linregress(costs, times)

    print "Regression: time =", slope, "x +", intercept
    print 'r value', r_value
    print 'p_value', p_value
    print 'standard deviation', std_err

    # Set up plot
    # Common sizes: (10, 7.5) and (12, 9)
    pylab.figure(facecolor="white", figsize=(12, 9))
    # plot frame lines
    ax = pylab.subplot(111)
    ax.spines["top"].set_visible(False)
    ax.spines["bottom"].set_visible(True)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(True)
    # tick marks
    pylab.tick_params(axis="both", which="both", bottom="on", top="off", labelbottom="on", left="off", right="off", labelleft="on")
    # Gridlines
    ax.yaxis.grid(b=True, which='both', color='black', alpha=0.3, ls='--')

    pylab.plot(costs, times, "o", color=common.tableau20[0], markeredgecolor=common.tableau20[0])
    pylab.plot(costs, slope * np.array(costs) + intercept, "-", lw=1.5, color=common.tableau20[4])

    #pylab.legend(loc="lower right")
    pylab.xlabel('path cost', x=1, ha="right")
    pylab.ylabel('driving time [s]', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.show()

if __name__=="__main__":
    main()