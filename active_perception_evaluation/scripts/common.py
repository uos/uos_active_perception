# -*- coding: utf-8 -*-
"""
Created on Thu Mar 26 10:53:43 2015

@author: Thorsten Gedicke
"""

import os
import numpy as np
import pylab

def graphColors(n):
    return pylab.plt.get_cmap('afmhot')(np.linspace(0.2, 0.65, n))

class SearchRun:
    def __init__(self, path):
        self._dir = path
        self._vals = self._readValsFile()
        self.label = self._readLabel()

        # movement time from [i-1]th step to i-th step
        self.tMove = np.array(self._vallist("expected_move_time"))
        # clock at step i
        self.clock = np.concatenate(([0], np.cumsum(self.tMove)))
        # probability to see target with step i
        self.gain = np.array(self._vallist("gain"))
        # expected probability to see target with step i
        self.expectedGain = np.array(self._vallist("expected_gain"))
        # probability to be already done at step i
        self.pDone = np.concatenate(([0], 1 - np.cumprod(1 - self.gain)))
        # probability to see the target with or after step i
        self.pSucc = np.array(self._vallist("success_probability"))
        # probability that a target is in unknown space after step i
        self.pSum = np.array(self._vallist("probability_sum"))
        # probability mass of missed cells
        self.pMissed = np.array(self._vallist("missed"))
        # probability mass of unexpected cells
        self.pUnexpected = np.array(self._vallist("unexpected"))
        # time spent for NBV evaluation
        self.tNbv = np.array(self._vallist("nbv_sampling_time")[:-1])
        # time spent for LUT generation
        self.tLut = np.array(self._vallist("initial_tt_lut_time")) + np.array(self._vallist("mutual_tt_lut_time"))
        # time spent planning
        self.tPlanning = np.array(self._vallist("planning_time"))
        # time spent for all cpu operations
        self.tCpu = self.tNbv + self.tPlanning
        # Estimation of total success probability
        self.pTotalSucc = 1 - (1 - self.pSucc) * (1 - self.pDone)
        # Total object presence probability (should be constant):
        self.pPresence = 1 - (1 - self.pSum) * (1 - self.pDone)

        self.eTMove = 0;
        for i in range(len(self.tMove)):
            self.eTMove += (1.0 - self.pDone[i]) * self.tMove[i]
        self.eTCpu = 0;
        for i in range(len(self.tCpu)):
            self.eTCpu += (1.0 - self.pDone[i]) * self.tCpu[i]

        # Sanity checks
        gain_alternative = np.array([1 - (1-self.pSum[i-1]) / (1-self.pSum[i]) for i in range(1, len(self.pSum))])
        if 0.0001 < np.max(np.abs(self.gain - gain_alternative)):
            raise Exception("gain sanity check failed")
        eTMove_alternative = np.sum([np.prod([1-g for g in self.gain[:i]]) * self.tMove[i] for i in range(len(self.tMove))])
        if 0.0001 < np.max(np.abs(self.eTMove - eTMove_alternative)):
            raise Exception("eTMove sanity check failed")
        eTCpu_alternative = np.sum([np.prod([1-g for g in self.gain[:i]]) * self.tCpu[i] for i in range(len(self.tCpu))])
        if 0.0001 < np.max(np.abs(self.eTCpu - eTCpu_alternative)):
            raise Exception("eTCpu sanity check failed")
        pPresenceAbsolute = 1 - (1 - self.pSum) * (1 - self.pDone)
        if 0.0001 < np.max(np.abs(np.gradient(pPresenceAbsolute))):
            raise Exception("absolute object presence probability not constant - sanity check failed")

    def _readValsFile(self):
        fvals = open(self._dir + "/vals.log")
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

    def _readLabel(self):
        try:
            f = open(self._dir + "/label")
            label = f.readline()[:-1]
            f.close()
        except:
            label = os.path.relpath(self._dir, self._dir + "/..")[:-2]
        return label

    def _vallist(self, key):
        return [d[key] for d in self._vals if key in d.keys()]