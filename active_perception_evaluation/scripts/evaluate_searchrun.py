# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

import pylab
import numpy as np
import sys
import os
import bisect

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

tableau20 = ['#6d0000', '#ffa000']

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
        label = os.path.relpath(d, d+"/..")[:-2]
    return label

def vallist(vals, key):
    return [d[key] for d in vals if key in d.keys()]

def main():
    if len(sys.argv) < 2:
        print "Need at least one log_dir as argument"
        sys.exit()

    print "Evaluating", sys.argv[1:]

    vals_batch = [readValsFile(log_dir) for log_dir in sys.argv[1:]]
    label_batch = [readLabel(log_dir) for log_dir in sys.argv[1:]]

    # Set up plot
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

    # To compare two runs, both must find the target with equal probability. If they don't, we must find the run with
    # the lowest success probability and truncate all other plans to match that probability
    min_psuccess = np.min([1 - np.prod([1 - p for p in vallist(vals, "gain")]) for vals in vals_batch])

    for trial_nr, vals in enumerate(vals_batch):
        label = label=label_batch[trial_nr]
        color = tableau20[trial_nr]
        print "\nEvaluating trial: ", label

        # movement time from [i-1]th step to i-th step
        move_times = [0] + vallist(vals, "expected_move_time")
        # clock at step i
        timeline = np.cumsum(move_times)
        # probability to see target with step i
        move_gains = np.array([0] + vallist(vals, "gain"))
        # expected probability to see target with step i
        expected_move_gains = np.array([0] + vallist(vals, "expected_gain"))
        # probability to see target with or before step i
        # pdone = np.array([1 - np.prod([1 - move_gains[i] for i in range(k+1)]) for k in range(len(move_gains))])
        pdone = 1 - np.cumprod([1 - g for g in move_gains])
        expected_pdone = 1 - np.cumprod([1 - g for g in expected_move_gains])
        # probability to see the target after step i
        psucc = np.array(vallist(vals, "success_probability"))
        # probability that a target is in unknown space after step i
        psum = np.array(vallist(vals, "probability_sum"))

        # Sanity check for gain values
        move_gains_alternative = np.array([0]+[1 - (1-psum[i-1]) / (1-psum[i]) for i in range(1, len(timeline))])
        if 0.0001 < np.max(np.abs(move_gains - move_gains_alternative)):
            print "ERROR: GAIN VALUE SANITY CHECK FAILED!"

        # plot pdone vs timeline
        pylab.step(timeline, pdone, "-", lw=2.0, color=color, label=label, where='post')
        #pylab.step(timeline, expected_pdone, ls="-", lw=2.0, color=color, label=label, where='post')

        # --- Other interesting plots ---
        # Estimation of total success probability
        # pylab.step(timeline, 1-(1-psucc)*(1-pdone), "-", lw=2.0, color=color, label=label, where='post')
        # Total object presence probability (should be constant):
        # pylab.step(timeline, 1-(1-psum)*(1-pdone), "-", lw=2.0, color=color, label=label, where='post')
        # pylab.step(timeline, move_gains - expected_move_gains, "-", lw=2.0, color=color, label=label, where='post')

        etime = 0;
        for i in range(len(move_times)-1):
            etime += (1.0 - pdone[i]) * move_times[i+1]
        pylab.axvline(etime, ls="--", lw=2.0, color=color)
        print "etime %.2f" % etime

        nbv_sampling_times = [d["nbv_sampling_time"] for d in vals if "nbv_sampling_time" in d.keys()]
        print "nbv_sampling_time min %.2f max %.2f average %.2f" % (np.min(nbv_sampling_times), np.max(nbv_sampling_times), np.mean(nbv_sampling_times))

        path_planning_times = [d["initial_tt_lut_time"] + d["mutual_tt_lut_time"] for d in vals if "initial_tt_lut_time" in d.keys() and "mutual_tt_lut_time" in d.keys()]
#        print "path_planning_time min %.2f max %.2f med %.2f" % (np.min(path_planning_times), np.max(path_planning_times), np.median(path_planning_times))

        planning_times = [d["planning_time"] for d in vals if "planning_time" in d.keys()]
        print "planning_time min %.2f max %.2f average %.2f" % (np.min(planning_times), np.max(planning_times), np.mean(planning_times))

    pylab.legend(loc="lower right")
    pylab.xlabel('time [s]', x=1, ha="right")
    pylab.ylabel('P', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.tight_layout()
    pylab.show()

if __name__=="__main__":
    main()