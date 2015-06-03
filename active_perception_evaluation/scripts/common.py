# -*- coding: utf-8 -*-
"""
Created on Thu Mar 26 10:53:43 2015

@author: Thorsten Gedicke
"""

import os
import numpy as np
import pylab

# Tableau 20 Colors
tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),
             (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),
             (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),
             (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]
# Tableau Color Blind 10
tableau20blind = [(0, 107, 164), (255, 128, 14), (171, 171, 171), (89, 89, 89),
             (95, 158, 209), (200, 82, 0), (137, 137, 137), (163, 200, 236),
             (255, 188, 121), (207, 207, 207)]
# Rescale to values between 0 and 1
for i in range(len(tableau20)):
    r, g, b = tableau20[i]
    tableau20[i] = (r / 255., g / 255., b / 255.)
for i in range(len(tableau20blind)):
    r, g, b = tableau20blind[i]
    tableau20blind[i] = (r / 255., g / 255., b / 255.)

def graphColors(n):
    return pylab.plt.get_cmap('afmhot')(np.linspace(0.2, 0.65, n))

def meanstd(x):
    return (np.mean(x), np.std(x, ddof=1))

def nameDecode(s, raw=False):
    if s == "greedy":
        ud = 0
        dl = 0
        phi = 1.0
    else:
        parts = s.split("_")
        ud = 0
        dl = "\\infty"
        phi = "1.5"
        for p in parts:
            if p.startswith("phi"):
                phi = "%.1f" % ((10.0 + float(p[3:])) / 10)
            if p.startswith("dl"):
                dl = p[2:]
            if p == "ud":
                ud = 1
            if p.startswith("id"):
                dl = "\\mathit{iter}_\\psi(%.1f)" % (float(p[2:]) / 10)
            if p.startswith("iw"):
                phi = "\\mathit{iter}_\\phi(%.1f)" % (float(p[2:]) / 10)
    if raw:
        return (ud, dl, phi)
    else:
        return "%s & $%s$ & $%s$ " % (ud, dl, phi)

class SearchRun:
    def __init__(self, path):
        self._dir = path
        self._vals = self._readValsFile()
        self.label = self._readLabel()

        # movement time from [i]th step to [i+1]-th step
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
        # time spent for LUT generation
        self.tLut = np.array(self._vallist("initial_tt_lut_time")) + np.array(self._vallist("mutual_tt_lut_time"))
        # time spent planning
        self.tPlanning = np.array(self._vallist("planning_time"))
        # time spent for NBV evaluation
        self.tNbv = np.array(self._vallist("nbv_sampling_time")[:len(self.tPlanning)])
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
        self.eTPlanning = 0;
        for i in range(len(self.tPlanning)):
            self.eTPlanning += (1.0 - self.pDone[i]) * self.tPlanning[i]

        # Sanity checks
        gain_alternative = np.array([1 - (1-self.pSum[i-1]) / (1-self.pSum[i]) for i in range(1, len(self.pSum))])
        if self.gain.size and 0.0001 < np.max(np.abs(self.gain - gain_alternative)):
            raise Exception("gain sanity check failed")
        eTMove_alternative = np.sum([np.prod([1-g for g in self.gain[:i]]) * self.tMove[i] for i in range(len(self.tMove))])
        if self.gain.size and 0.0001 < np.max(np.abs(self.eTMove - eTMove_alternative)):
            raise Exception("eTMove sanity check failed")
        eTCpu_alternative = np.sum([np.prod([1-g for g in self.gain[:i]]) * self.tCpu[i] for i in range(len(self.tCpu))])
        if self.gain.size and 0.0001 < np.max(np.abs(self.eTCpu - eTCpu_alternative)):
            raise Exception("eTCpu sanity check failed")
        pPresenceAbsolute = 1 - (1 - self.pSum) * (1 - self.pDone)
        if self.gain.size and 0.0001 < np.max(np.abs(np.gradient(pPresenceAbsolute))):
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
                try:
                    vals[val_idx][fields[0]] = float(fields[1])
                except ValueError:
                    vals[val_idx][fields[0]] = float('nan')
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

class SearchPlan:
    def __init__(self, path):
        self._dir = path
        self._vals = self._readValsFile()
        self.label = self._readLabel()

        # clock at step i
        self.clock = np.array(self._vallist(1))
        # movement time from [i]th step to [i+1]-th step
        self.tMove = np.diff(self.clock)
        # probability to be already done at step i
        self.pDone = np.array(self._vallist(0))
        # probability to see target with step i
        self.gain = np.array([1 - (1 - self.pDone[i+1]) / (1 - self.pDone[i]) for i in range(len(self.pDone)-1)])
        # expected probability to see target with step i
        self.expectedGain = self.gain
        # Estimation of total success probability
        self.pTotalSucc = np.array([self.pDone[-1]] * len(self.pDone))
        # Total object presence probability (should be constant):
        self.pPresence = np.array([self.pDone[-1]] * len(self.pDone))
        # probability to see the target with or after step i
        self.pSucc = np.array([1 - (1 - self.pDone[-1]) / (1 - p) for p in self.pDone])
        # probability that a target is in unknown space after step i
        self.pSum = np.array([1 - (1 - self.pDone[-1]) / (1 - p) for p in self.pDone])
        # probability mass of missed cells
        self.pMissed = np.array([float("NaN")] * len(self.gain))
        # probability mass of unexpected cells
        self.pUnexpected = np.array([float("NaN")] * len(self.gain))
        # time spent for NBV evaluation
        self.tNbv = np.array([float("NaN")] * len(self.gain))
        # time spent for LUT generation
        self.tLut = np.array([float("NaN")] * len(self.gain))
        # time spent planning
        self.tPlanning = np.array([float("NaN")] * len(self.gain))
        # time spent for all cpu operations
        self.tCpu = np.array([float("NaN")] * len(self.gain))

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
        with open(self._dir + "/plan-timeplot-1.tab") as fvals:
            vals = []
            next(fvals) # skip header
            for line in fvals:
                fields = line.split('\t')
                if len(fields) != 3:
                    print "WARNING: Corrupt plan file; wrong number of values:", line
                else:
                    vals.append([float(x) for x in fields])
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
        return [d[key] for d in self._vals]