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
    if len(sys.argv) < 2:
        print "Need at least one log_dir as argument"
        sys.exit()

    print "Evaluating", sys.argv[1:]

    trials = [common.SearchRun(d) for d in sys.argv[1:]]

    # Set up plot
    nplots = 2
    # Common sizes: (10, 7.5) and (12, 9)
    pylab.figure(facecolor="white", figsize=(6, 3))
    for i in range(1, nplots + 1):
        # Remove the plot frame lines. They are unnecessary chartjunk.
        ax = pylab.subplot(nplots, 1, i)
        ax.spines["top"].set_visible(False)
        ax.spines["bottom"].set_visible(True)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_visible(True)
        # Remove the tick marks; they are unnecessary with the tick lines we just plotted.
        pylab.tick_params(axis="both", which="both", bottom="on", top="off", labelbottom="on", left="off", right="off", labelleft="on")
        # Gridlines
        ax.yaxis.grid(b=True, which='both', color='black', alpha=0.3, ls='--')

    for trial_nr, trial in enumerate(trials):
        color = common.graphColors(len(trials))[trial_nr]
        print "\nEvaluating trial: ", trial.label
        print "eTMove:", trial.eTMove
        print "eTCpu:", trial.eTCpu

        # plot pdone vs timeline
        pylab.subplot(nplots, 1, 1)
        pylab.step(trial.clock, trial.pDone, ls="-", lw=2.0, color=color, label=trial.label, where='post')
        if nplots >= 2:
            pylab.subplot(nplots, 1, 2)
            pylab.step(trial.clock[:-1], trial.pMissed, ls="--", lw=2.0, color=color, label=trial.label, where='post')
            pylab.step(trial.clock[:-1], trial.pUnexpected, ls="-", lw=2.0, color=color, label=trial.label, where='post')

        # --- Other interesting plots ---
        pylab.subplot(nplots, 1, 1)
        # Estimation of total success probability
        # pylab.step(trial.clock, trial.pTotalSucc, "-", lw=2.0, color=color, label=trial.label, where='post')

        pylab.subplot(nplots, 1, 1)
        pylab.axvline(trial.eTMove, ls="--", lw=2.0, color=color)

    pylab.subplot(nplots, 1, 1)
    pylab.legend(loc="lower right")
    pylab.xlabel('time [s]', x=1, ha="right")
    pylab.ylabel('P', y=1, va="top")
    #pylab.title('Search progress over time', y=1.01, va="bottom")
    pylab.tight_layout()
    pylab.show()

if __name__=="__main__":
    main()