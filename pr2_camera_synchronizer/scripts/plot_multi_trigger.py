#! /usr/bin/env python

from __future__ import with_statement

import roslib; roslib.load_manifest('pr2_camera_synchronizer')
import rospy
from ethercat_trigger_controllers.msg import MultiWaveform
import dynamic_reconfigure.client
import threading
import sys
import copy

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
    out = intervals
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

    #print >> sys.stderr, "Interval intersection", a, b, total
    return total

def interval_intersection_sum(one, many, period):
    return sum(interval_intersection(one, other, period) for other in many)

class Trigger(TriggerBase):
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
        self.parent.update()

class Camera(TriggerBase):
    def __init__(self, parent, node_name, trigger):
        self.name = node_name
        self.ready = False
        self.parent = parent
        parent.add_camera(self)
        self.trigger = trigger
        self.trigger_delay = 0.0001 
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
            projector_pulse_len = max(interval_lengths(projector_intervals))
    
            for s, e, v in camera_intervals:
                info = "camera %s exposing from %f to %f"%(self.name,s,e)
                if v == 0.75:
                    alt = True
                    interval = s+0.0005, e-0.0005, v # Conservative check that we don't lose texture light.
                elif v == 1:
                    alt = False
                    interval = s-0.0005, e+0.0005, v # Conservative check that we don't hit texture light.
                else:
                    raise Exception("Can't determine register set, %s.", info)

                red_light_time = interval_intersection_sum(interval, projector_intervals, self.period) 
                if not red_light_time or red_light_time < projector_pulse_len:
                    print >> sys.stderr, projector_intervals
                    print >> sys.stderr, projector.pts
                    raise Exception("Partial intersection with pulse (alt=%s), %s, %f s of %f."%(alt, info, red_light_time, projector_pulse_len))
                if alt and not red_light_time:
                    raise Exception("Alternate imager but no projector, %s."%info)
                if not alt and red_light_time:
                    raise Exception("Primary imager and has projector, %s."%info)
                if alt and e - s > projector_pulse_len + 0.001:
                    raise Exception("Too long exposure for textured image, %s."%info)
        except Exception, e:
            self.parent.set_error(repr(e))

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
            self.parent.set_error("Non-consistent triggering times for %s."%self.node_name)

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
                self.pts.append((exp_start_early - 1e-6, 0.5 + alternate_offset))
            self.pts.append((exp_start_late, 1 + alternate_offset))
            self.pts.append((exp_end + 1e-6, 0))
        return True
        
    def callback(self, config):
        #print >> sys.stderr, "Camera callback", self.name
        self.config = copy.deepcopy(config)
        self.ready = True
        self.parent.update()

class TriggerChecker:
    def __init__(self):
        self.triggers = []
        self.mutex = threading.Lock()
        self.error = None
        self.cameras = []

    def add(self, trigger):
        self.triggers.append(trigger)

    def add_camera(self, cam):
        self.cameras.append(cam)
        self.add(cam)

    def set_projector(self, projector):
        self.projector = projector
    
    def spin(self):
        rate = rospy.Duration(1)
        while not rospy.is_shutdown():
            rospy.sleep(rate)

    def update(self):
      with self.mutex:
        self.error = None
        n = len(self.triggers)
        for i in range(n):
            if not self.triggers[i].compute():
                self.set_error('No data for %s. (This is normal at startup.)'%self.triggers[i].name)
                return
        period = max(t.period for t in self.triggers)
        for i in range(n):
            if (period / self.triggers[i].period) % 1:
                self.set_error('Period for %s is %f, expected divisor of %f'%(self.triggers[i].name,
                        self.triggers[i].period, period))
                return

        if self.error:
            self.empty_plot()
        else:
            self.plot(period)
        if self.projector: 
            print >> sys.stderr
            for c in self.cameras:
                c.check(self.projector)

    def set_error(self, msg):
        rospy.logerr(msg)
        self.error = msg
        
    def plot(self, period):
        n = len(self.triggers)
        style = 'with linespoints title "%s"'
        print 'plot "-"', style%self.triggers[0].name,
        for i in range(1, n):
            print ', "-"', style%self.triggers[i].name,
        print
        for i in range(n):
            t = self.triggers[i]
            reps = int(period / t.period)
            pts = t.sorted_pts()
            if len(pts) == 0:
                pts = [(0, 0)]
            def plot_pt(x, y):
                print x, (n - i - 1) * 1.1 + y%2
            plot_pt(0, pts[-1][1])
            for k in range(reps):
                xoffs = t.period * k
                for j in range(len(pts)):
                    plot_pt(pts[j][0] + xoffs, pts[j-1][1])
                    plot_pt(pts[j][0] + xoffs, pts[j][1])
            plot_pt(period, pts[-1][1])
            print "e"
            print
            sys.stdout.flush()

    def empty_plot(self):
        print 'plot x, -x'
    
    def check(self, test):
        with self.tp.mutex:
            for camera in self.cameras:
                camera.check(test, self.projector)
            test.failIf(self.tp.error)

def main():    
    tp = TriggerChecker()
    head_trig = Trigger(tp, '/head_camera_trigger')
    l_forearm_trig = Trigger(tp, 'l_forearm_cam_trigger')
    r_forearm_trig = Trigger(tp, 'r_forearm_cam_trigger')
    Camera(tp, '/narrow_stereo_both', head_trig)
    tp.set_projector(Trigger(tp, '/projector_trigger'))
    Camera(tp, '/wide_stereo_both', head_trig)
    Camera(tp, '/l_forearm_cam', l_forearm_trig)
    Camera(tp, '/r_forearm_cam', r_forearm_trig)
    tp.spin()

if __name__ == "__main__":
    rospy.init_node('trigger_plotter', anonymous = True)
    main()
