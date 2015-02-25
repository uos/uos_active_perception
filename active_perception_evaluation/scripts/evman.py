#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 25 11:14:27 2015

@author: Thorsten Gedicke
"""

import subprocess
import os
import sys
import time
import signal
import random
import numpy as np

class Evman:
    def __init__(self, script="."):
        self.campoints = [(float(p[0]), float(p[1])) for p in [l.split() for l in CAM_POINTS.split("\n")] if len(p) > 1]
        self.procsBase = []
        self.procMapping = None
        self.procSearchMan = None
        self.procRviz = None
        self.baselog = open('%s/evman_base.log' % script, 'w')
        self.calllog = open('%s/evman_call.log' % script, 'w')
        self.nbvlog = open('%s/evman_nbv.log' % script, 'w')
        self.smlog = open('%s/evman_sm.log' % script, 'w')

    def initBaseSystem(self, gui=False):
        print "> Launching base system..."
        if gui:
            guival = "true"
        else:
            guival = "false"
        self.procsBase.append(subprocess.Popen("exec roslaunch active_perception_evaluation race_world.launch gui:=%s" % guival, shell=True, preexec_fn=os.setsid, stdout=self.baselog, stderr=self.baselog))
        time.sleep(30)
        self.procsBase.append(subprocess.Popen("exec roslaunch active_perception_evaluation spawn_race_objects.launch", shell=True, preexec_fn=os.setsid, stdout=self.baselog, stderr=self.baselog))
        self.procsBase.append(subprocess.Popen("exec roslaunch active_perception_evaluation spawn_floating_kinect.launch", shell=True, preexec_fn=os.setsid, stdout=self.baselog, stderr=self.baselog))
        time.sleep(30)
        subprocess.call("exec rosservice call /gazebo/unpause_physics", shell=True, stdout=self.baselog, stderr=self.baselog)
        self.procsBase.append(subprocess.Popen("exec roslaunch active_perception_evaluation floating_kinect_navigation.launch", shell=True, preexec_fn=os.setsid, stdout=self.baselog, stderr=self.baselog))
        print "> Base system ready"

    def setCamera(self, xypose=None):
        print "> Setting camera to random position"
        # xypose = map-gz-tf + random valid cam pose. cam is set to point upwards
        if not xypose:
            xypose = tuple(np.array((-12.28, -10.20)) + np.array(self.campoints[random.randint(0, len(self.campoints)-1)]))
        subprocess.call("exec rosservice call /gazebo/set_model_state 'model_state:\n  model_name: floating_kinect\n  pose:\n    position:\n      x: %s\n      y: %s\n      z: 1.5\n    orientation:\n      x: 0.0\n      y: 0.71\n      z: 0.0\n      w: -0.71'" % xypose, shell=True, stdout=self.calllog, stderr=self.calllog)
        return xypose

    def resetGzworld(self):
        print "> Resetting Gazebo"
        subprocess.call("exec rosservice call /gazebo/reset_world", shell=True, stdout=self.calllog, stderr=self.calllog)

    def initMapping(self):
        if not self.procsBase:
            print "> WARNING: Running Mapping without Base"
            self.initBaseSystem()
        print "> Starting Mapping"
        self.procMapping = subprocess.Popen("exec roslaunch race_object_search object_search_prerequisites_pr2.launch sim:=true map:=race_extended_inflated fk:=true", shell=True, preexec_fn=os.setsid, stdout=self.nbvlog, stderr=self.nbvlog)
        time.sleep(10)

    def stopMapping(self):
        print "> Stopping Mapping"
        if self.procMapping:
            os.killpg(self.procMapping.pid, signal.SIGTERM)
            self.procMapping.wait()
            self.procMapping = None

    def initSearchMan(self, dl=0, la=0.75, bl=1.5, to=60, lss=0, gss=200, rs=0, log=os.getcwd()):
        if not self.procMapping:
            print "> WARNING: Running SearchMan without Mapping"
            self.initMapping()
        print "> Starting SearchMan"
        params = (dl, la, bl, to, log, lss, gss, rs)
        cmd = "rosrun race_object_search object_search_manager _world_frame_id:=/map _robot:=floating_kinect _planning_mode:=search _depth_limit:=%i _relative_lookahead:=%f _max_rel_branch_cost:=%f _planning_timeout:=%f _log_dir:=%s _local_sample_size:=%i _global_sample_size:=%i _ray_skip:=%f" % params
        self.procSearchMan = subprocess.Popen("exec " + cmd, shell=True, preexec_fn=os.setsid, stdout=self.smlog, stderr=self.smlog)
        time.sleep(10)

    def stopSearchMan(self):
        print "> Stopping SearchMan"
        if self.procSearchMan:
            os.killpg(self.procSearchMan.pid, signal.SIGTERM)
            self.procSearchMan.wait()
            self.procSearchMan = None

    def initRviz(self, gui=False):
        if not gui:
            return
        if not self.procsBase:
            print "> WARNING: Running Rviz without Base"
            self.initBaseSystem()
        print "> Starting Rviz"
        fnull = open(os.devnull, 'w')
        self.procRviz = subprocess.Popen("exec rosrun rviz rviz -d /home/thorsten/ros_fuerte/race/uos_active_perception/race_object_search/config/object_search.vcg", shell=True, preexec_fn=os.setsid, stdout=fnull, stderr=fnull)
        time.sleep(5)

    def stopRviz(self):
        print "> Stopping Rviz"
        if self.procRviz:
            os.killpg(self.procRviz.pid, signal.SIGTERM)
            self.procRviz.wait()
            self.procRviz = None

    def runTest(self, cmd="reset table1 table2 counter shelf1 shelf2 shelf3"):
        if not self.procSearchMan:
            print "> WARNING: Running Test without SearchMan"
            self.initSearchMan
        print "> Running Test"
        subprocess.call("rosrun race_object_search object_search_manager_test " + cmd, shell=True, preexec_fn=os.setsid, stdout=self.calllog, stderr=self.calllog)

    def shutdown(self):
        print "> Shutting down ROS processes..."
        self.stopSearchMan()
        self.stopMapping()
        self.stopRviz()
        for p in self.procsBase:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait()
        self.procsBase = []
        print "> All processes terminated."

    def shutdown_handler(self, signum, frame):
        self.shutdown()
        sys.exit()

    def shutdown_excepthook(self, exctype, value, tb):
        print 'Type:', exctype
        print 'Value:', value
        print 'Traceback:', tb
        self.shutdown()
        sys.exit()

def init(script="."):
    evman = Evman(script)
    sys.excepthook = evman.shutdown_excepthook
    signal.signal(signal.SIGTERM, evman.shutdown_handler)
    signal.signal(signal.SIGINT, evman.shutdown_handler)
    return evman

def iros0215_nomap(N=20, gui=False):
    script = "iros0215_nomap"
    print "*** %s ***" % script
    if os.path.exists(script):
        print "WARNING: directory %s already exists, skipping script" % script
        return False
    os.mkdir(script)
    evman = init(script)
    for n in range(N):
        # --- GREEDY ---
        print "# Evaluating n=%i greedy" % n
        logdir = "%s/%s/greedy_%i" % (os.getcwd(), script, n)
        os.mkdir(logdir)
        evman.initBaseSystem(gui)
        evman.initRviz(gui)
        xypose = evman.setCamera()
        evman.initMapping()
        evman.initSearchMan(dl=0, log=logdir)
        evman.runTest()
        evman.shutdown()
        # --- PLANNING ---
        print "# Evaluating n=%i planning" % n
        logdir = "%s/%s/planning_%i" % (os.getcwd(), script, n)
        os.mkdir(logdir)
        evman.initBaseSystem(gui)
        evman.initRviz(gui)
        evman.setCamera(xypose)
        evman.initMapping()
        evman.initSearchMan(dl=5, log=logdir)
        evman.runTest()
        evman.shutdown()
        # --- CLEANUP ---
        try:
            os.remove("persistent_samples")
        except OSError:
            print "WARNING: Expected to see persistent_samples, but there were none!"
    evman.shutdown()
    return True

def iros0215_withmap(N=20, gui=False):
    script = "iros0215_withmap"
    print "*** %s ***" % script
    if os.path.exists(script):
        print "WARNING: directory %s already exists, skipping script" % script
        return False
    os.mkdir(script)
    evman = init(script)
    # --- INIT ---
    evman.initBaseSystem(gui)
    evman.initRviz(gui)
    evman.initMapping()
    evman.initSearchMan(dl=0, log=script)
    evman.runTest()
    for n in range(N):
        # --- GREEDY ---
        print "# Evaluating n=%i greedy" % n
        logdir = "%s/%s/greedy_%i" % (os.getcwd(), script, n)
        os.mkdir(logdir)
        evman.resetGzworld()
        xypose = evman.setCamera()
        evman.initSearchMan(dl=0, log=logdir)
        evman.runTest()
        evman.stopSearchMan()
        # --- PLANNING ---
        print "# Evaluating n=%i planning" % n
        logdir = "%s/%s/planning_%i" % (os.getcwd(), script, n)
        os.mkdir(logdir)
        evman.resetGzworld()
        evman.setCamera(xypose)
        evman.initSearchMan(dl=5, log=logdir)
        evman.runTest()
        evman.stopSearchMan()
        # --- CLEANUP ---
        try:
            os.remove("persistent_samples")
        except OSError:
            print "WARNING: Expected to see persistent_samples, but there were none!"
    evman.shutdown()
    return True

def main():
    iros0215_nomap()
    iros0215_withmap()

CAM_POINTS = """9.147	10.5173	27.0398
11.1854	10.276	48.3509
9.12263	11.5707	50.2993
4.9752	13.306	43.9274
11.9221	11.8566	41.4409
11.667	10.552	28.7855
11.8429	10.2029	42.4091
1.49492	12.4837	56.4743
5.51736	11.8996	34.3329
11.1522	11.9901	46.0075
6.96425	9.99261	37.1898
11.5289	10.7157	45.0167
1.41461	10.7334	70.9344
9.15461	13.43	43.2888
9.69409	13.0525	23.1117
8.99297	10.2901	26.9925
1.99165	10.7775	50.7728
5.86211	7.67383	50.3805
1.92771	12.5625	72.8214
1.57872	12.0874	58.0517
1.2834	12.9701	61.669
1.91956	12.0819	74.9677
6.25307	10.2973	52.1454
6.16069	12.0045	42.6586
8.6314	12.4265	28.1398
5.11731	12.205	50.3587
5.16706	12.6801	39.3048
6.37901	9.93703	40.3333
2.01425	11.401	52.0236
1.32209	12.7789	61.7376
9.85875	10.0014	26.2494
6.28692	8.33674	46.5827
6.41959	9.77153	49.0745
6.73033	8.89286	46.4519
9.33684	12.6405	29.3231
12.8418	11.566	18.388
1.53197	10.5344	70.6463
2.17246	9.53899	65.0894
6.49893	9.66894	37.6247
0.151935	12.9201	60.8917
6.6446	9.59563	35.5816
4.64304	12.3313	55.3493
1.70484	10.5019	56.457
8.14656	12.9248	38.975
1.04349	10.1772	56.1778
1.33024	9.03981	67.1069
1.92455	11.1552	71.0072
2.67083	10.2306	66.1682
1.64776	11.5131	53.1616
0.837156	10.0758	66.8044
6.23985	11.5467	55.6777
2.55896	11.2877	51.478
11.8371	12.5775	30.759
6.66635	6.98476	33.6985
11.8986	10.7351	20.9076
6.67185	10.0132	29.4522
5.77764	7.39687	61.1109
5.97982	12.295	55.4227
2.05708	12.8634	54.0031
12.7815	10.1238	29.4081
7.03199	9.98947	38.6029
4.45735	12.0529	49.3381
2.02188	9.42741	58.458
1.65309	11.9578	51.8177
12.6733	11.505	43.9939
9.75506	9.65804	24.8606
8.17	12.7831	26.5072
6.09493	12.4846	33.8085
1.39242	12.5007	54.6892
1.25504	11.8975	72.5398
6.81636	13.012	36.0227
5.56682	11.7229	32.4072
6.49606	6.86423	49.3599
6.40593	9.58819	57.8149
11.7125	9.17405	30.7363
1.21544	10.5889	73.0136
7.76207	12.8441	45.3652
2.7109	12.5912	75.1598
5.00074	11.7127	36.9634
6.78963	9.9536	50.9346
7.27739	10.1368	36.1192
2.16189	12.2344	54.5281
6.34074	9.64275	47.2788
6.43212	12.8316	47.463
5.04032	12.5021	37.5489
5.96447	13.3604	49.8919
9.10515	11.5954	28.726
2.08614	10.2391	53.6333
5.38261	12.6359	56.5265
4.54785	13.1237	48.1762
0.951861	10.6717	71.4895
6.63457	9.56492	48.0738
2.09047	10.6155	70.3712
5.41828	12.1645	35.7748
1.34622	10.6296	52.0958
11.597	10.0521	43.0061
9.27469	9.85976	36.9851
5.77461	11.7137	35.0976
6.25327	7.6327	34.7343
6.56107	13.3091	36.5527
7.03847	10.23	29.8458
4.7858	12.033	50.5092
6.5117	12.3993	33.0592
0.349093	12.7914	51.332
13.0417	10.6758	39.9078
7.13571	13.119	49.6958
12.7132	11.704	37.5101
5.18628	11.9288	42.4696
4.73217	13.0197	55.9459
0.886139	9.67639	52.9024
1.98898	12.7719	53.9968
9.01543	10.0911	26.5255
1.21556	13.1956	59.3414
5.29876	7.23205	57.394
12.1875	11.3673	18.7817
5.8651	12.9939	44.7533
6.69563	6.9805	33.8083
6.2161	13.4138	37.138
7.17895	10.4865	30.86
1.74291	9.83394	66.4891
6.16053	7.77772	49.9528
7.55684	10.2606	53.5352
9.33758	12.8728	36.9873
8.35466	10.2936	26.9541
11.1046	12.4307	36.3988
6.36812	9.5827	49.9249
5.35864	11.9345	57.3371
10.8495	13.4725	33.0508
2.1078	11.2334	54.0422
1.73087	10.5308	54.3695
2.19758	11.5882	72.3547
6.19323	7.5944	50.5511
8.41913	12.4114	38.402
8.96613	12.3837	25.2488
4.9417	12.2897	48.887
0.282763	10.0092	61.1157
1.0876	10.3468	55.4562
0.966886	11.4309	61.1567
1.54968	10.7199	51.7199
11.7869	9.83941	40.3531
5.94808	12.6541	34.0876
2.201	10.8437	48.576
6.82539	9.69564	51.4368
9.53213	13.3317	47.4122
9.30776	9.53376	25.4705
8.82013	13.0588	24.8181
1.62228	11.6326	62.4802
1.32057	11.5401	64.8478
9.49028	9.55859	27.5974
11.7895	13.033	38.9481
6.04576	7.83265	49.8912
12.1527	11.1566	45.6365
6.48249	10.7059	56.635
5.6405	11.7751	46.882
6.15747	12.992	52.086
1.30257	10.8111	47.5052
1.96688	11.3057	60.4339
0.993779	11.3329	49.0642
1.86591	11.078	51.8768
1.42047	9.6707	71.5904
6.99459	10.0097	29.786
12.6437	10.8493	20.5165
6.55404	8.68208	31.7669
6.02183	13.1071	33.0976
1.34806	9.86216	63.3491
5.3969	11.8045	50.3482
6.4775	9.64364	52.2992
10.5875	9.5319	30.3958
2.19791	10.0413	59.4415
5.61927	13.0795	31.0567
0.894829	11.4047	48.2121
9.25027	9.83901	44.124
2.24436	9.91027	66.8952
1.95422	9.83097	55.9082
7.06604	10.3185	52.0119
6.77651	10.4936	46.2679
0.105197	12.6498	50.7074
2.42458	11.7956	66.0248
8.94544	9.8412	30.9063
7.52409	10.3001	35.8433
6.41973	9.54574	36.5688
1.59937	9.92702	64.261
2.94374	12.705	52.8341
8.94591	10.5046	35.4393
11.5518	13.15	36.6729
9.24066	9.7443	42.2371
8.18329	10.2266	44.7423
10.356	13.5622	45.5709
9.50362	10.1302	50.2255
10.0845	9.79932	32.1238
1.77681	13.1558	63.2766
"""

if __name__=="__main__":
    main()