#! /usr/bin/env python

import roslib
roslib.load_manifest('pr2_ethercat')
import rospy, sys
import std_srvs.srv
publish_trace = rospy.ServiceProxy("pr2_ethercat/publish_trace", std_srvs.srv.Empty)
publish_trace()

