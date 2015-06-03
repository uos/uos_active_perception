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

SANE_CLOCK = (120, 220)

class SearchRunSeries:
    def __init__(self, trials):
        self.eTMove = np.array([t.eTMove for t in trials])
        self.eTCpu = np.array([t.eTCpu for t in trials])
        self.eTPlanning = np.array([t.eTPlanning for t in trials])
        self.tPlanning = np.array([x for t in trials for x in t.tPlanning])
        self.tNbv = np.array([x for t in trials for x in t.tNbv])
        self.tLut = np.array([x for t in trials for x in t.tLut])

def evaluate(names):
    scale_idx = 0
    autosort = False
    if not names:
        names = list(set([s.rsplit("_", 1)[0] for s in os.listdir(os.getcwd()) if "_" in s and s.rsplit("_", 1)[1].isdigit()]))
        scale_idx = names.index("greedy")
        autosort = True
    trials = []
    maxn = sys.maxint
    for name in names:
        t = []
        for n in xrange(sys.maxint):
            logdir = "%s/%s_%i" % (os.getcwd(), name, n)
            if not os.path.exists(logdir):
                maxn = min(maxn, n)
                print "*** %s: %s" % (name, n)
                break
            try:
                t.append(SearchRun(logdir))
                if t[-1].eTMove < SANE_CLOCK[0] or t[-1].eTMove > SANE_CLOCK[1]:
                    print "WARNING: eTMove of %s %s outside sane bounds: %s" % (name, n, t[-1].eTMove)
            except (IOError, ValueError) as e:
                maxn = min(maxn, n)
                print "*** %s: %s" % (name, n)
                print e
                break
        trials.append(t)

    series = [SearchRunSeries(t[:maxn]) for t in trials]

    # now calculate values for output, table, and plotting
    algos = []
    scale = series[scale_idx].eTMove + series[scale_idx].eTPlanning
    for i, name in enumerate(names):
        # remember: all these elements (sumtime, eTMove ...) are vectors
        eTMove = meanstd(series[i].eTMove)
        tPlanning = meanstd(series[i].tPlanning)
        eTPlanning = meanstd(series[i].eTPlanning)
        relTime = meanstd((series[i].eTMove + series[i].eTPlanning) / scale)

        algos.append((name, eTMove, eTPlanning, relTime, tPlanning))

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
    print r"$\xi$ & $\psi$ & $\phi$ & Execution [\si{\second}] & Planning [\si{\second}] & Relative duration" "\\\\"
    print r"\midrule"
    for a in algos:
        line = "%s & $%.1f \\pm %.1f$ & $%.1f \\pm %.1f$ & $%.2f \\pm %.2f$ \\\\" % (nameDecode(a[0]), a[1][0], a[1][1], a[2][0], a[2][1], a[3][0], a[3][1])
        print line
    print r"\bottomrule"
    print r"\end{tabular}"
    print "[end LaTeX]"

    if autosort:
        algos.sort(key=lambda x: x[3][0])

    print "Name            eTMove \t\t tPlanning \t avrg Planning \t relTime"
    for a in algos:
        print "%s %.1f ± %.1f \t %.1f ± %.1f \t %.1f ± %.1f \t %.2f ± %.2f" % (a[0].ljust(15), a[1][0], a[1][1], a[2][0], a[2][1], a[4][0], a[4][1], a[3][0], a[3][1])

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
