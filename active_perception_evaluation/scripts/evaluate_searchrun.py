#!/usr/bin/python -u
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 14 15:18:39 2015

@author: Thorsten Gedicke
"""

import pylab
import sys
import common
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Computer Modern Sans Serif']})
rc('text', usetex=True)

def main():
    if len(sys.argv) < 2:
        print "Need at least one log_dir as argument"
        sys.exit()

    print "Evaluating", sys.argv[1:]

    trials = []
    for arg in sys.argv[1:]:
        if arg.startswith("--plan:"):
            trials.append(common.SearchPlan(arg[7:]))
        else:
            trials.append(common.SearchRun(arg))

    # Set up plot
    nplots = 1
    # Common sizes: (10, 7.5) and (12, 9)
    pylab.figure(facecolor="white", figsize=(8, 4))
    for i in range(1, nplots + 1):
        # plot frame lines
        ax = pylab.subplot(nplots, 1, i)
        ax.spines["top"].set_visible(False)
        ax.spines["bottom"].set_visible(True)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_visible(True)
        # tick marks
        pylab.tick_params(axis="both", which="both", bottom="on", top="off", labelbottom="on", left="off", right="off", labelleft="on")
        # Gridlines
        ax.yaxis.grid(b=True, which='both', color='black', alpha=0.3, ls='--')

    for trial_nr, trial in enumerate(trials):
        color = common.graphColors(len(trials))[trial_nr]
        color = common.tableau20[[6,3][trial_nr]]
        lv = common.nameDecode(trial.label, True)
        label = r"$\psi=%s$, $\phi=%s$, $\xi=%s$" % (lv[1], lv[2], lv[0])
        print "\nEvaluating trial: ", trial.label
        print "eTMove:", trial.eTMove
        print "eTCpu:", trial.eTCpu

        # plot pdone vs timeline
        pylab.subplot(nplots, 1, 1)
        pylab.step(trial.clock, trial.pDone, ls="-", lw=2.0, color=color, label=label, where='post')
        if nplots >= 2:
            pylab.subplot(nplots, 1, 2)
            pylab.step(trial.clock[:-1], trial.pMissed, ls="--", lw=2.0, color=color, label=label, where='post')
            pylab.step(trial.clock[:-1], trial.pUnexpected, ls="-", lw=2.0, color=color, label=label, where='post')

        # --- Other interesting plots ---
        pylab.subplot(nplots, 1, 1)
        # Estimation of total success probability
        # pylab.step(trial.clock, trial.pTotalSucc, "-", lw=2.0, color=color, label=trial.label, where='post')

        pylab.subplot(nplots, 1, 1)
        pylab.axvline(trial.eTMove, ls="--", lw=2.0, color=color)

    pylab.subplot(nplots, 1, 1)
    pylab.legend(loc="lower right", prop={'size':11})
    pylab.xlabel('time [s]', x=1, ha="right")
    pylab.ylabel('P', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.tight_layout()
    pylab.show()

if __name__=="__main__":
    main()