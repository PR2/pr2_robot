#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_etherCAT')
import rospy, sys
import std_srvs.srv
publish_trace = rospy.ServiceProxy("pr2_etherCAT/publish_trace", std_srvs.srv.Empty)
publish_trace()

