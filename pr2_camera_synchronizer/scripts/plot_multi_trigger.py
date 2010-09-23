#! /usr/bin/env python

from __future__ import with_statement

PKG='pr2_camera_synchronizer'
import roslib; roslib.load_manifest(PKG)
import rospy
from ethercat_trigger_controllers.msg import MultiWaveform
import dynamic_reconfigure.client
import threading
import sys
import copy
import time
import subprocess
import atexit

# TODO: 
# - Check that exposure duration is correct for projectorless modes.
# - Check modes that aren't synchronized with the projector.

slight_shift = 1e-6 #ms

class TriggerBase:
    def sorted_pts(self):
        #print >> sys.stderr, self.name
        #print >> sys.stderr, self.pts
        pts = [ (x % self.period, y) for (x, y) in self.pts ]
        firstpt = pts.index(min(pts, key = lambda (x,y):x))
        pts = pts[firstpt:] + pts[:firstpt]
        #print >> sys.stderr, pts
        #print >> sys.stderr
        return pts

    def intervals(self):
        if not self.pts:
            return []
        if self.pts[-1][1]:
            raise Exception("Last point in sequence has non-zero y coord in %s."%self.name)
        intervals = []
        idx = 0
        while idx < len(self.pts):
            if self.pts[idx][1]:
                startidx = idx
                max_value = 0
                while self.pts[idx][1]:
                    max_value = max(max_value, self.pts[idx][1])
                    idx += 1
                intervals.append((self.pts[startidx][0], self.pts[idx][0], max_value))
            idx += 1
        return intervals

def replicate_interval(intervals, period, reps):
    out = list(intervals)
    for i in range(1, reps):
        offset = i * period
        out += [(s+offset,e+offset,v) for (s,e,v) in intervals]
    return out

def interval_lengths(intervals):
    return [e-s for s,e,v in intervals]

def interval_intersection(a, b, period):
    a0, a1, _ = a
    b0, b1, _ = b

    a0 += period
    a1 += period
    b0 += period
    b1 += period

    # Assume that all these values are now positive.
    assert(a0 > 0)
    assert(a1 > 0)
    assert(b0 > 0)
    assert(b1 > 0)

    # Get all the intervals to start in the first period.
    a0w = a0 % period
    a1w = a1 + a0w - a0
    b0w = b0 % period
    b1w = b1 + b0w - b0

    # Get the second interval to start before the first, but not more than
    # one period before.
    if b0w > a0w:
        b0w -= period
        b1w -= period

    # How much intersection?
    total = max(0,  min(b1w, a1w) - max(b0w, a0w))

    # How much intersection when the second interval gets shifted forward
    # one period?

    b0w += period
    b1w += period
    total += max(0,  min(b1w, a1w) - max(b0w, a0w) )

    #print >> sys.stderr, "Interval intersection", a, b, period, total
    return total





class Trigger(TriggerBase):
    def __init__(self, parent, trigger_name):
        self.parent = parent
        parent.add(self)
        self.name = trigger_name
        self.ready = False
        rospy.Subscriber(trigger_name + "/waveform", MultiWaveform, self.callback)
        self.last_msg = None

    def compute(self):
        if not self.ready:
            return False
        return True

    def callback(self, msg):
        #print >> sys.stderr, "Trigger callback", self.name
        if self.last_msg == msg:
            return
        self.last_msg = msg
        self.period = msg.period
        self.ready = True
        offset = msg.zero_offset
        self.pts = [ ((trans.time + offset) % self.period, trans.value % 2) for trans in msg.transitions ]
        self.parent.update()






class Camera(TriggerBase):
    def __init__(self, parent, node_name, trigger):
        self.name = node_name
        self.ready = False
        self.parent = parent
        parent.add_camera(self)
        self.trigger = trigger
        self.trigger_delay = 0.00008
           # Approximate, but good enough given that everything else is on the 1kHz ethercat clock.
        self.client = dynamic_reconfigure.client.Client(node_name,
                config_callback = self.callback)

    def check(self, projector):
        try:
            projector_intervals = projector.intervals()
            camera_intervals = self.intervals()

            #print >> sys.stderr
            #print >> sys.stderr, self.name
            #print >> sys.stderr, projector_intervals
            #print >> sys.stderr, camera_intervals
    
            reps = self.period / projector.period
            if reps % 1:
                raise Exception("Camera %s period is %f, projector period is %f, ratio %f"%
                        (self.name,self.period,projector.period,reps))
            reps = int(reps)
    
            projector_intervals = replicate_interval(projector_intervals, projector.period, reps)
            projector_pulse_len = max(interval_lengths(projector_intervals)+[0])
    
            for s, e, v in camera_intervals:
                info = "camera %s exposing from %f to %f"%(self.name,s,e)
                if v == 0.75:
                    alt = True
                    interval = s+0.0004, e-0.0004, v # Conservative check that we don't lose texture light.
                elif v == 1:
                    alt = False
                    interval = s-0.0005, e+0.0005, v # Conservative check that we don't hit texture light.
                else:
                    raise Exception("Can't determine register set, %s.", info)

                red_light_times = [ interval_intersection(interval, pi, self.period) for pi in projector_intervals ]
                red_light_times.sort()
                red_light_time = sum(red_light_times)
                if red_light_time and red_light_time != red_light_times[-1]:
                    #print >> sys.stderr, red_light_times
                    #print >> sys.stderr, projector_intervals
                    raise Exception("Intersection with multiple pulses, %s."%info)

                if red_light_time and abs(red_light_time - projector_pulse_len) > slight_shift:
                    #print >> sys.stderr, projector_intervals
                    #print >> sys.stderr, projector.pts
                    raise Exception("Partial intersection with pulse (alt=%s), %s, %f s of %f, delta %e."%(alt, info, red_light_time, projector_pulse_len, red_light_time-projector_pulse_len))
                if alt and not red_light_time:
                    raise Exception("Alternate imager but no projector, %s."%info)
                if not alt and red_light_time:
                    raise Exception("Primary imager and has projector, %s."%info)
                with_proj_exp = projector_pulse_len + 0.0015 + 2 * slight_shift
                if alt and e - s > with_proj_exp:
                    raise Exception("Too long exposure for textured image %f instead of %f, %s."%(e-s,with_proj_exp,info))
        except Exception, e:
            #import traceback
            #traceback.print_exc()
            self.parent.set_error(repr(e))

    def compute(self):
        if not self.ready or not self.trigger.ready:
            #print >> sys.stderr, "Not ready", self.name
            return False
        if not self.config['ext_trig']:
            #print >> sys.stderr, "Not ext trig", self.name
            return False
        self.period = self.trigger.period
        self.pts = []
        rising = self.config['rising_edge_trig']
        before = 0 if rising else 1
        after = 1 - before
        pretrigpts = []
        imager_period = 1.0 / self.config['imager_rate']

        #print >> sys.stderr, "\n", self.name

        # Figure out the trigger times and the corresponding register set.
        tpts = self.trigger.sorted_pts()
        for i in range(len(tpts)):
            if tpts[i-1][1] == before and tpts[i][1] == after:
                pretrigpts.append(tpts[i][0] + self.trigger_delay)
                #print >> sys.stderr, "preselected", tpts[i]
        trigpts = []

        # Run through the triggers once to find the last trigger time in
        # the sequence.
        last_trig = -imager_period
        trig_count = 0
        last_count = 0
        for i in range(len(pretrigpts)):
            trig_count += 1
            if pretrigpts[i] - last_trig > imager_period:
                last_trig = pretrigpts[i]
                last_count = trig_count
        
        # Run through the triggers again keeping 
        first_trig = last_trig
        last_trig = first_trig - self.period
        for i in range(len(pretrigpts)):
            trig_count += 1
            if pretrigpts[i] - last_trig > imager_period:
                last_trig = pretrigpts[i]
                reg_set = self.config['register_set']
                if reg_set == 2:
                    reg_set = 1 if (last_count - trig_count) % 2 else 0
                trigpts.append((pretrigpts[i], reg_set))
                last_count = trig_count
            else:
                #print >> sys.stderr, "dropped", pretrigpts[i]
                pass
        if last_trig != first_trig:
            self.parent.set_error("Non-consistent triggering times for %s."%self.name)

        # Figure out exposure times for each register set.
        cfg_suffix = [ "", "_alternate" ]
        exposure_min = []
        exposure_max = []
        max_exp = self.config['max_exposure']
        for i in range(2):
            auto_exp = self.config['auto_exposure'+cfg_suffix[i]]
            set_exp = self.config['exposure'+cfg_suffix[i]]
            cur_min = 0
            cur_max = max_exp
            if not auto_exp:
                cur_min = min(max_exp, set_exp)
                cur_max = cur_min
            exposure_min.append(cur_min)
            exposure_max.append(cur_max)

        # Generate the exposure points.
        for (t, rs) in trigpts:
            alternate_offset = - rs * 0.25
            exp_end = t + imager_period
            exp_start_early = exp_end - exposure_max[rs]
            exp_start_late = exp_end - exposure_min[rs]
            if exp_start_early != exp_start_late:
                self.pts.append((exp_start_early - slight_shift, 0.5 + alternate_offset))
            self.pts.append((exp_start_late, 1 + alternate_offset))
            self.pts.append((exp_end + slight_shift, 0))
        return True
        
    def callback(self, config):
        #print >> sys.stderr, "Camera callback", self.name
        if self.ready and self.config == config:
            return
        self.config = copy.deepcopy(config)
        self.ready = True
        self.parent.update()






class TriggerChecker:
    def __init__(self, plot, silent = False):
        self.do_plot = plot
        self.triggers = []
        self.mutex = threading.Lock()
        self.error = None
        self.updated = False
        self.cameras = []
        self.projector = None
        self.silent = silent
        if self.do_plot:
            try:
                self.gnuplot = subprocess.Popen('gnuplot', stdin = subprocess.PIPE)
            except:
                print "Gnuplot must be installed to allow plotting of waveforms."
                sys.exit(1)
            atexit.register(self.gnuplot.kill)

    def add(self, trigger):
        self.triggers.append(trigger)

    def add_camera(self, cam):
        self.cameras.append(cam)
        self.add(cam)

    def set_projector(self, projector):
        self.projector = projector

    def clear(self):
        with self.mutex:
            for trig in self.triggers:
                trig.ready = False
            self.updated = False
    
    def update(self):
      with self.mutex:
        self.error = None
        n = len(self.triggers)
        for i in range(n):
            if not self.triggers[i].compute():
                if not self.silent:
                    self.set_error('No data for %s. (This is normal at startup.)'%self.triggers[i].name)
                return

        if self.do_plot:
            if self.error:
                self.empty_plot()
            else:
                self.plot()
        if self.projector: 
            for c in self.cameras:
                c.check(self.projector)
            if not self.silent:
                if not self.error:
                    rospy.loginfo("All checks passed.")
                else:
                    rospy.logerr("Some checks failed.")
            self.updated = True

    def set_error(self, msg):
        if not self.silent:
            rospy.logerr(msg)
        self.error = msg
        
    def plot(self):
        n = len(self.triggers)
        period = max(t.period for t in self.triggers)
        for i in range(n):
            if (period / self.triggers[i].period) % 1:
                self.set_error('Period for %s is %f, expected divisor of %f'%(self.triggers[i].name,
                        self.triggers[i].period, period))
                return
        style = 'with linespoints title "%s"'
        print >> self.gnuplot.stdin, 'plot "-"', style%self.triggers[0].name,
        for i in range(1, n):
            print >> self.gnuplot.stdin, ', "-"', style%self.triggers[i].name,
        print >> self.gnuplot.stdin 
        for i in range(n):
            t = self.triggers[i]
            reps = int(period / t.period)
            pts = t.sorted_pts()
            if len(pts) == 0:
                pts = [(0, 0)]
            def plot_pt(x, y):
                print >> self.gnuplot.stdin, x, (n - i - 1) * 1.1 + y%2
            plot_pt(0, pts[-1][1])
            for k in range(reps):
                xoffs = t.period * k
                for j in range(len(pts)):
                    plot_pt(pts[j][0] + xoffs, pts[j-1][1])
                    plot_pt(pts[j][0] + xoffs, pts[j][1])
            plot_pt(period, pts[-1][1])
            print >> self.gnuplot.stdin, "e"
            print >> self.gnuplot.stdin
            sys.stdout.flush()

    def empty_plot(self):
        print 'plot x, -x'
    
import unittest
class Test(unittest.TestCase):
    def setUp(self):
        global tp, reconfig_client
        tp.silent = True
        self.reconfig_client = reconfig_client
        tp.clear()
        self.reconfig_client.update_configuration({'projector_mode': 1})
    
    def wait_for_ready(self):
        global tp
        time.sleep(2)
        starttime = time.time()
        while not rospy.is_shutdown():
            if time.time() > starttime + 5:
                self.fail("Took too long to get responses.")
            if tp.updated:
                break
            time.sleep(0.1)
    
    def evaluate(self):
        global tp
        self.wait_for_ready()
        tp.silent = False
        tp.update()
        self.assertFalse(tp.error, tp.error)

    def test_with(self):
        self.reconfig_client.update_configuration({
                    'projector_pulse_shift': 0.0,
                    'wide_stereo_trig_mode': 3, 
                    'prosilica_projector_inhibit': False, 
                    'camera_reset': False, 
                    'forearm_r_rate': 30.0,
                    'projector_rate': 58.823529411764703, 
                    'stereo_rate': 29.411764705882351, 
                    'projector_pulse_length': 0.002,
                    'projector_mode': 2, 
                    'forearm_r_trig_mode': 3,
                    'projector_tweak': 0.0, 
                    'forearm_l_trig_mode': 3,
                    'forearm_l_rate': 30.0, 
                    'narrow_stereo_trig_mode': 3,
                })
        self.evaluate()
    
    def test_without(self):
        self.reconfig_client.update_configuration({
                    'projector_pulse_shift': 0.0,
                    'wide_stereo_trig_mode': 4, 
                    'prosilica_projector_inhibit': False, 
                    'camera_reset': False, 
                    'forearm_r_rate': 30.0,
                    'projector_rate': 58.823529411764703, 
                    'stereo_rate': 29.411764705882351, 
                    'projector_pulse_length': 0.002,
                    'projector_mode': 3, 
                    'forearm_r_trig_mode': 4,
                    'projector_tweak': 0.0, 
                    'forearm_l_trig_mode': 4,
                    'forearm_l_rate': 30.0, 
                    'narrow_stereo_trig_mode': 4,
                })
        self.evaluate()

    def test_alt(self):
        self.reconfig_client.update_configuration({
                    'projector_pulse_shift': 0.0,
                    'wide_stereo_trig_mode': 4, 
                    'prosilica_projector_inhibit': False, 
                    'camera_reset': False, 
                    'forearm_r_rate': 30.0,
                    'projector_rate': 58.823529411764703, 
                    'stereo_rate': 29.411764705882351, 
                    'projector_pulse_length': 0.002,
                    'projector_mode': 3, 
                    'forearm_r_trig_mode': 4,
                    'projector_tweak': 0.0, 
                    'forearm_l_trig_mode': 4,
                    'forearm_l_rate': 30.0, 
                    'narrow_stereo_trig_mode': 5,
                })
        self.evaluate()

    def test_slow(self):
        self.reconfig_client.update_configuration({
                    'projector_pulse_shift': 0.0,
                    'wide_stereo_trig_mode': 4, 
                    'prosilica_projector_inhibit': False, 
                    'camera_reset': False, 
                    'forearm_r_rate': 5.0,
                    'projector_rate': 58.823529411764703, 
                    'stereo_rate': 5.0,
                    'projector_pulse_length': 0.002,
                    'projector_mode': 3, 
                    'forearm_r_trig_mode': 4,
                    'projector_tweak': 0.0, 
                    'forearm_l_trig_mode': 3,
                    'forearm_l_rate': 5.0, 
                    'narrow_stereo_trig_mode': 5,
                })
        self.evaluate()

def main():    
    global tp, reconfig_client
    regression_test = rospy.get_param('~regression_test', False)
    tp = TriggerChecker(plot = not regression_test, silent = regression_test)
    head_trig = Trigger(tp, '/head_camera_trigger')
    l_forearm_trig = Trigger(tp, 'l_forearm_cam_trigger')
    r_forearm_trig = Trigger(tp, 'r_forearm_cam_trigger')
    Camera(tp, '/narrow_stereo_both', head_trig)
    tp.set_projector(Trigger(tp, '/projector_trigger'))
    Camera(tp, '/wide_stereo_both', head_trig)
    Camera(tp, '/l_forearm_cam', l_forearm_trig)
    Camera(tp, '/r_forearm_cam', r_forearm_trig)
    if regression_test:
         import rostest
         rospy.loginfo("Running in unit test mode")
         reconfig_client = dynamic_reconfigure.client.Client('synchronizer')
         rostest.rosrun(PKG, 'test_bare_bones', Test)
    else:
        rospy.loginfo("Running in plotting mode")
        while not rospy.is_shutdown():
            time.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('trigger_plotter', anonymous = True)
    main()
