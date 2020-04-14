#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_ethercat')
import rospy, sys
import std_srvs.srv
reset = rospy.ServiceProxy("pr2_ethercat/reset_motors", std_srvs.srv.Empty)
reset()

