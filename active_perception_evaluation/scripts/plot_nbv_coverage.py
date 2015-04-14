#!/usr/bin/python -u
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

import pylab
import numpy as np
import sys
import os
import common

def main():

    color = "blue"
    showtime = True

    cellcounts = []
    meantimes = []
    ymax = 0
    n = sys.maxint
    with open("/home/thorsten/ros_fuerte/race/uos_active_perception/race_object_search/config/maps/nbveval.csv") as df:
        for line in df:
            elems = line.split(",")
            if elems[0] == "cells":
                cells = np.array([float(x) for x in elems[1:]])
                cellcounts.append(cells)
                ymax = max(ymax, cells[-1])
                n = min(n, len(cells))
            if elems[0] == "response":
                meantimes.append(float(elems[1]) / float(elems[2]))
    time_per_sample = np.mean(meantimes)

    means = np.zeros(n)
    stds = np.zeros(n)
    for i in range(n):
        vals = [cells[i] for cells in cellcounts]
        means[i] = np.mean(vals)
        stds[i] = np.std(vals)

    # Common sizes: (10, 7.5) and (12, 9)
    pylab.figure(facecolor="white", figsize=(6, 3))

    # Remove the plot frame lines. They are unnecessary chartjunk.
    ax = pylab.subplot(111)
    ax.spines["top"].set_visible(False)
    ax.spines["bottom"].set_visible(True)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(True)
    # Remove the tick marks; they are unnecessary with the tick lines we just plotted.
    pylab.tick_params(axis="both", which="both", bottom="on", top="off", labelbottom="on", left="off", right="off", labelleft="on")
    # Gridlines
    ax.yaxis.grid(b=True, which='both', color='black', alpha=0.3, ls='--')

    x = np.array(range(n))
    if showtime:
        x = [float(val) * time_per_sample for val in x]
        print x

    pylab.plot(x, means/ymax, ls="-", lw=2.0, color=color)
    upper = (means + stds) / ymax
    lower = (means - stds) / ymax
    pylab.plot(x, upper, ls="-", lw=2.0, color="grey")
    pylab.plot(x, lower, ls="-", lw=2.0, color="grey")
    pylab.fill_between(x, upper, lower, color=color, alpha='0.3')

    if showtime:
        pylab.xlabel('time [s]', x=1, ha="right")
    else:
        pylab.xlabel('# samples', x=1, ha="right")
    pylab.ylabel('coverage', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.tight_layout()
    pylab.show()

if __name__=="__main__":
    main()