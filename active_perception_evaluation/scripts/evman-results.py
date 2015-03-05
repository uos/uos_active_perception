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

def iros0215(N=20):
    script = "iros0215_withmap"
    print "*** %s ***" % script
    if not os.path.exists(script):
        print "ERROR: directory %s does not exist, skipping script" % script
        return False
    nbv_times = []
    lut_times = []
    greedy_etimes = []
    greedy_ecputimes = []
    greedy_ptimes = []
    for n in range(N):
        # --- GREEDY ---
        print "# Evaluating n=%i greedy" % n
        logdir = "%s/%s/greedy_%i" % (os.getcwd(), script, n)
        if not os.path.exists(logdir):
            print "WARNING: aborted at %s" % logdir
            break
        vals = readValsFile(logdir)
        nbv_times += vallist(vals, "nbv_sampling_time")
        lut_times += list(np.array(vallist(vals, "initial_tt_lut_time")) + np.array(vallist(vals, "mutual_tt_lut_time")))
        greedy_ptimes += vallist(vals, "planning_time")
        move_times = [0] + vallist(vals, "expected_move_time")
        move_gains = [0] + vallist(vals, "gain")
        move_cputimes = np.array([0] + vallist(vals, "nbv_sampling_time")[:-1]) + np.array([0] + vallist(vals, "planning_time"))
        etime = np.sum([np.prod([1-g for g in move_gains[:i]]) * move_times[i] for i in range(1, len(move_times))])
        ecputime = np.sum([np.prod([1-g for g in move_gains[:i]]) * move_cputimes[i] for i in range(1, len(move_times))])
        greedy_etimes.append(etime)
        greedy_ecputimes.append(ecputime)
    planning_etimes = []
    planning_ecputimes = []
    planning_ptimes = []
    for n in range(N):
        # --- PLANNING ---
        print "# Evaluating n=%i planning" % n
        logdir = "%s/%s/planning_%i" % (os.getcwd(), script, n)
        if not os.path.exists(logdir):
            print "WARNING: aborted at %s" % logdir
            break
        vals = readValsFile(logdir)
        nbv_times += vallist(vals, "nbv_sampling_time")
        lut_times += list(np.array(vallist(vals, "initial_tt_lut_time")) + np.array(vallist(vals, "mutual_tt_lut_time")))
        planning_ptimes += vallist(vals, "planning_time")
        move_times = [0] + vallist(vals, "expected_move_time")
        move_gains = [0] + vallist(vals, "gain")
        move_cputimes = np.array([0] + vallist(vals, "nbv_sampling_time")[:-1]) + np.array([0] + vallist(vals, "planning_time"))
        etime = np.sum([np.prod([1-g for g in move_gains[:i]]) * move_times[i] for i in range(1, len(move_times))])
        ecputime = np.sum([np.prod([1-g for g in move_gains[:i]]) * move_cputimes[i] for i in range(1, len(move_times))])
        planning_etimes.append(etime)
        planning_ecputimes.append(ecputime)

    tdiffs = np.array(greedy_etimes) - np.array(planning_etimes)

    print "# Results Greedy:"
    print "> mean:     ", np.mean(greedy_etimes)
    print ">  std:     ", np.std(greedy_etimes)
    print "> mean cpu: ", np.mean(greedy_ecputimes)
    print ">  std cpu: ", np.std(greedy_ecputimes)
    print "> avrg planning time: %.2f (std: %.2f)" % (np.mean(greedy_ptimes), np.std(greedy_ptimes))
    print "# Results Planning:"
    print "> mean:     ", np.mean(planning_etimes)
    print ">  std:     ", np.std(planning_etimes)
    print "> mean cpu: ", np.mean(planning_ecputimes)
    print ">  std cpu: ", np.std(planning_ecputimes)
    print "> avrg planning time: %.2f (std: %.2f)" % (np.mean(planning_ptimes), np.std(planning_ptimes))
    print "# Extreme diffs (greedy - planning):"
    print ">  max: %f at %i" % (np.max(tdiffs), np.argmax(tdiffs))
    print ">  min: %f at %i" % (np.min(tdiffs), np.argmin(tdiffs))
    print "> ", np.argsort(tdiffs)
    print "# Average times:"
    if nbv_times:
        print "> NBV Sampling: %.2f (std: %.2f)" % (np.mean(nbv_times), np.std(nbv_times))
    if lut_times:
        print "> Trans. Times: %.2f (std: %.2f)" % (np.mean(lut_times), np.std(lut_times))

def main():
    iros0215()

if __name__=="__main__":
    main()
