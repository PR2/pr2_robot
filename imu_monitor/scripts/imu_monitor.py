#!/usr/bin/env python

import roslib; roslib.load_manifest('imu_monitor')
import rospy
import PyKDL
from sensor_msgs.msg import Imu
from pr2_mechanism_controllers.msg import Odometer
from threading import Lock
from math import fabs, pi
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

EPS = 0.0001

class ImuMonitor:
    def __init__(self):
        self.lock = Lock()

        # reset state
        self.dist = 0.0
        self.drift = -1.0
        self.last_angle = 0.0
        self.start_angle = 0.0
        self.start_time = rospy.Time.now()

        # subscribe to topics
        self.imu_sub = rospy.Subscriber('torso_lift_imu/data', Imu, self.imu_cb)
        self.imu_sub = rospy.Subscriber('base_odometry/odometer', Odometer, self.odom_cb)

        # diagnostics
        self.pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)


    def imu_cb(self, msg):
        with self.lock:
            rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                            msg.orientation.z, msg.orientation.w)
            (r, p, self.last_angle) = rot.GetRPY()
            #print self.last_angle*180/3.14

    def odom_cb(self, msg):
        with self.lock:
            dist = msg.distance + (msg.angle * 0.25)

            # check if base moved
            if dist > self.dist + EPS:
                print 'Reset imu monitor because base moved'
                self.start_time = rospy.Time.now()
                self.start_angle = self.last_angle
                self.dist = dist
        
            # do imu test if possible
            if rospy.Time.now() > self.start_time + rospy.Duration(10.0):
                self.drift = fabs(self.start_angle - self.last_angle)*180/(pi*10)
                self.start_time = rospy.Time.now()
                self.start_angle = self.last_angle
                self.last_measured = rospy.Time.now()
                
            # publish diagnostics
            d = DiagnosticArray()
            d.header.stamp = rospy.Time.now()
            ds = DiagnosticStatus()
            ds.name = "Imu Drift Monitor"
            if self.drift < 0.5:
                ds.level = DiagnosticStatus.OK
                ds.message = 'OK'
            elif self.drift < 1.0:
                ds.level = DiagnosticStatus.WARN
                ds.message = 'Drifting'
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = 'Drifting'
            if self.drift < 0:
                last_measured = 'No measurements yet, waiting for base to stop moving before measuring'
            else:
                last_measured = str(self.last_measured.to_sec())
            ds.values = [
                KeyValue('Last measured', last_measured),
                KeyValue('Drift (deg/sec)', str(self.drift)) ]
            d.status = [ds]
            self.pub_diag.publish(d)

def main():
    rospy.init_node('imu_monitor')
    rospy.sleep(0.001) # init time

    monitor = ImuMonitor()

    rospy.spin()


if __name__ == '__main__':
    main()
