# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

from scipy import stats
import pylab
import numpy as np
import sys
import os

# These are the "Tableau 20" colors as RGB.
tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),
             (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),
             (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),
             (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]

# Scale the RGB values to the [0, 1] range, which is the format matplotlib accepts.
for i in range(len(tableau20)):
    r, g, b = tableau20[i]
    tableau20[i] = (r / 255., g / 255., b / 255.)

def readValsFile(d):
    fvals = open(d + "/vals.log")
    vals = []
    val_idx = -1
    for line in fvals:
        fields = line.split('\t')
        if len(fields) < 2:
            vals.append({})
            val_idx += 1
            if val_idx + 1 != int(line):
                print "WARNING: Corrupt vals.log; Got iteration", line, ", expected", val_idx + 1
        elif len(fields) == 2:
            if fields[0] in vals[val_idx].keys():
                print "WARNING: Corrupt vals.log; Duplicate values for", fields[0], "in iteration", val_idx + 1
            vals[val_idx][fields[0]] = float(fields[1])
        else:
            print "WARNING: Corrupt vals.log; Illegal line", line
    fvals.close()
    return vals

def readLabel(d):
    try:
        f = open(d + "/label")
        label = f.readline()[:-1]
        f.close()
    except:
        label = os.path.relpath(d, d+"/..")
    return label

def vallist(vals, key):
    return [d[key] for d in vals if key in d.keys()]

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

    pylab.plot(costs, times, "o")
    pylab.plot(costs, slope * np.array(costs) + intercept, "-", lw=1.5)

    #pylab.legend(loc="lower right")
    pylab.xlabel('path cost', x=1, ha="right")
    pylab.ylabel('driving time [s]', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.show()

if __name__=="__main__":
    main()