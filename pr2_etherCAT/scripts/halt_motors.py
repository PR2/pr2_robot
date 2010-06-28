#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_etherCAT')
import rospy, sys
import std_srvs.srv
halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
halt()

