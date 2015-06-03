#!/usr/bin/python -u
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

import numpy as np
import os
import sys
from common import *

# timeout value
TOTIME = 600
SANE_CLOCK = (120, 220)

class PlanningSeries:
    def __init__(self, trial):
        self.eTMove = []
        self.tPlanning = []
        self.eTPlanning = []
        self.timeouts = 0
        for (r,p) in trial:
            if np.isnan(r.tPlanning[0]):
                self.timeouts = self.timeouts + 1
                self.eTMove.append(p.eTMove)
                self.tPlanning.append(TOTIME)
                self.eTPlanning.append(np.sum((1 - p.pDone) * TOTIME))
            else:
                self.eTMove.append(p.eTMove)
                self.tPlanning.append(r.tPlanning[0])
                self.eTPlanning.append(np.sum((1 - p.pDone) * r.tPlanning[0]))
        self.eTMove = np.array(self.eTMove)
        self.tPlanning = np.array(self.tPlanning)
        self.eTPlanning = np.array(self.eTPlanning)
        self.tNbv = np.array([x for t,_ in trial for x in t.tNbv])
        self.tLut = np.array([x for t,_ in trial for x in t.tLut])

def evaluate(names):
    trials = [] # will be a list containing lists of (SearchRun, SearchPlan) for all configurations
    maxn = sys.maxint
    scale_idx = 0
    autosort = False
    if not names:
        names = list(set([s.rsplit("_", 1)[0] for s in os.listdir(os.getcwd()) if "_" in s and s.rsplit("_", 1)[1].isdigit()]))
        scale_idx = names.index("greedy")
        autosort = True
    for name in names:
        t = [] # will contain all (SearchRun, SearchPlan) for this configuration
        for n in xrange(sys.maxint):
            logdir = "%s/%s_%i" % (os.getcwd(), name, n)
            if not os.path.exists(logdir):
                maxn = min(maxn, n)
                print "*** %s: %s" % (name, n)
                break
            try:
                t.append((SearchRun(logdir), SearchPlan(logdir)))
                if t[-1][1].eTMove < SANE_CLOCK[0] or t[-1][1].eTMove > SANE_CLOCK[1]:
                    print "WARNING: eTMove of %s %s outside sane bounds: %s" % (name, n, t[-1][1].eTMove)
            except IOError as e:
                maxn = min(maxn, n)
                print "*** %s: %s" % (name, n)
                print e
                break
        trials.append(t)

    # series is in same order as names
    # first element is anchor for relative values
    series = [PlanningSeries(t[:maxn]) for t in trials]

    # now calculate values for output, table, and plotting
    algos = []
    scale = series[scale_idx].eTMove + series[scale_idx].tPlanning
    print names[scale_idx]
    for i, name in enumerate(names):
        # remember: all these elements (sumtime, eTMove ...) are vectors
        eTMove = meanstd(series[i].eTMove)
        tPlanning = meanstd(series[i].tPlanning)
        relTime = meanstd((series[i].eTMove + series[i].tPlanning) / scale)
        algos.append((name, eTMove, tPlanning, relTime, series[i].timeouts))

    if autosort:
        algos.sort(key=lambda x: nameDecode(x[0], True)[2])
        algos.sort(key=lambda x: nameDecode(x[0], True)[1])
        algos.sort(key=lambda x: nameDecode(x[0], True)[0])

    print("[plot data]")
    print repr(algos)
    print("[end plot data]")

    print "[LaTeX]"
    print r"\begin{tabular}{ccccccc}"
    print r"\toprule"
    print r"$\xi$ & $\psi$ & $\phi$ & Execution [\si{\second}] & Planning [\si{\second}] & Relative duration & Timeouts" "\\\\"
    print r"\midrule"
    for a in algos:
        tostr = a[4] if a[4] else "--"
        line = "%s & $%.1f \\pm %.1f$ & $%.1f \\pm %.1f$ & $%.2f \\pm %.2f$ & %s \\\\" % (nameDecode(a[0]), a[1][0], a[1][1], a[2][0], a[2][1], a[3][0], a[3][1], tostr)
        print line
    print r"\bottomrule"
    print r"\end{tabular}"
    print "[end LaTeX]"

    if autosort:
        algos.sort(key=lambda x: x[3][0])

    print "Name            eTMove \t\t tPlanning \t relTime \t timeouts"
    for a in algos:
        print "%s %.1f ± %.1f \t %.1f ± %.1f \t %.2f ± %.2f \t %s" % (a[0].ljust(15), a[1][0], a[1][1], a[2][0], a[2][1], a[3][0], a[3][1], a[4])

    allTNbv = np.concatenate([s.tNbv for s in series])
    allTLut = np.concatenate([s.tLut for s in series])
    print "# Average times:"
    print "> NBV Sampling: %.2f (std: %.2f)" % meanstd(allTNbv)
    print "> Trans. Times: %.2f (std: %.2f)" % meanstd(allTLut)

    if len(series) == 2:
        eTMoveDiff = series[0].eTMove - series[1].eTMove
        print "# Extreme diffs (%s - %s):" % (names[0], names[1])
        print ">  max: %.2f at %i" % (np.max(eTMoveDiff), np.argmax(eTMoveDiff))
        print ">  min: %.2f at %i" % (np.min(eTMoveDiff), np.argmin(eTMoveDiff))
        print ">", np.argsort(eTMoveDiff)

def main():
    evaluate(sys.argv[1:])

if __name__=="__main__":
    main()
