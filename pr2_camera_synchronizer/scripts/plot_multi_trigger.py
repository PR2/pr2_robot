#! /usr/bin/env python

from __future__ import with_statement

import roslib; roslib.load_manifest('pr2_camera_synchronizer')
import rospy
from ethercat_trigger_controllers.msg import MultiWaveform
import dynamic_reconfigure.client
import threading
import sys
import copy
      
class Trigger:
    def __init__(self, parent, trigger_name):
        self.parent = parent
        parent.add(self)
        self.name = trigger_name
        self.ready = False
        rospy.Subscriber(trigger_name + "/waveform", MultiWaveform, self.callback)

    def compute(self):
        if not self.ready:
            return False
        return True

    def callback(self, msg):
        #print >> sys.stderr, "Trigger callback", self.name
        self.period = msg.period
        self.ready = True
        offset = msg.zero_offset
        self.pts = [ ((trans.time + offset) % self.period, trans.value % 2) for trans in msg.transitions ]
        self.pts.sort()
        self.parent.update()

class Camera:
    def __init__(self, parent, node_name, trigger):
        self.name = node_name
        self.ready = False
        self.parent = parent
        parent.add(self)
        self.trigger = trigger
        self.client = dynamic_reconfigure.client.Client(node_name,
                config_callback = self.callback)

    def compute(self):
        if not self.ready or not self.trigger.ready:
            return False
        if not self.config['ext_trig']:
            return False
        self.period = self.trigger.period
        self.pts = []
        rising = self.config['rising_edge_trig']
        before = 0 if rising else 1
        after = 1 - before
        pretrigpts = []
        imager_period = 1.0 / self.config['imager_rate']

        # Figure out the trigger times and the corresponding register set.
        for i in range(0, len(self.trigger.pts)):
            if self.trigger.pts[i-1][1] == before and self.trigger.pts[i][1] == after:
                pretrigpts.append(self.trigger.pts[i][0])
        trigpts = []
        for i in range(0, len(pretrigpts)):
            d1 = (pretrigpts[i] - pretrigpts[i-1]) % self.period
            d2 = (pretrigpts[i-len(pretrigpts)+1] - pretrigpts[i]) % self.period
            if d1 > imager_period or d2 > imager_period:
                reg_set = self.config['register_set']
                if reg_set == 2:
                    reg_set = 0 if d1 < imager_period else 1
                trigpts.append((pretrigpts[i], reg_set))
        print sys.stderr, trigpts, pretrigpts

        # Figure out exposure times for each register set.
        cfg_suffix = [ "", "_alternate" ]
        exposure_min = []
        exposure_max = []
        max_exp = self.config['max_exposure']
        for i in range(0, 2):
            auto_exp = self.config['auto_exposure'+cfg_suffix[i]]
            set_exp = self.config['exposure'+cfg_suffix[i]]
            cur_min = 0
            cur_max = max_exp
            if not auto_exp:
                cur_min = set_exp
                cur_max = set_exp
            exposure_min.append(cur_min)
            exposure_max.append(cur_max)

        # Generate the exposure points.
        for (t, rs) in trigpts:
            exp_end = t + imager_period
            exp_start_early = exp_end - exposure_max[rs]
            exp_start_late = exp_end - exposure_min[rs]
            if exp_start_early != exp_start_late:
                self.pts.append(((exp_start_early - 1e-4) % self.period, 0.5))
            self.pts.append((exp_start_late % self.period, 1))
            self.pts.append(((exp_end + 1e-4) % self.period, 0))
        self.pts.sort()
        return True
        
    def callback(self, config):
        #print >> sys.stderr, "Camera callback", self.name
        self.config = copy.deepcopy(config)
        self.ready = True
        self.parent.update()

class TriggerPlotter:
    def __init__(self):
        self.triggers = []
        self.mutex = threading.Lock()

    def add(self, trigger):
        self.triggers.append(trigger)

    def spin(self):
        rate = rospy.Duration(1)
        while not rospy.is_shutdown():
            rospy.sleep(rate)

    def update(self):
      with self.mutex:
        n = len(self.triggers)
        for i in range(0, n):
            if not self.triggers[i].compute():
                print >> sys.stderr, 'No data for %s'%self.triggers[i].name
                return
        period = self.triggers[0].period
        for i in range(0, n):
            if self.triggers[i].period != period:
                print >> sys.stderr, 'Period for %s is %f, expected %f'%(self.triggers[i].name,
                        self.triggers[i].period, period)
                return
        
        style = 'with linespoints title "%s"'
        print 'plot "-"', style%self.triggers[0].name,
        for i in range(1, n):
            print ', "-"', style%self.triggers[i].name,
        print
        for i in range(0, n):
            t = self.triggers[i]
            pts = t.pts
            def plot_pt(x, y):
                print x, (n - i - 1) * 1.1 + y%2
            plot_pt(0, pts[-1][1])
            for j in range(0, len(pts)):
                plot_pt(pts[j][0], pts[j-1][1])
                plot_pt(pts[j][0], pts[j][1])
            plot_pt(period, pts[-1][1])
            print "e"
            print
            sys.stdout.flush()

rospy.init_node('trigger_plotter', anonymous = True)
tp = TriggerPlotter()
head_trig = Trigger(tp, '/head_camera_trigger')
Camera(tp, '/narrow_stereo_both', head_trig)
Trigger(tp, '/projector_trigger')
Camera(tp, '/wide_stereo_both', head_trig)
tp.spin()
