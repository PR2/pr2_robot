#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Kevin Watts

import roslib
roslib.load_manifest('pr2_computer_monitor')

import rospy

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

low_hd_level = 5
critical_hd_level = 1

hd_temp_warn = 50
hd_temp_error = 55

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }
temp_dict = { 0: 'OK', 1: 'Warm', 2: 'Hot' }
usage_dict = { 0: 'OK', 1: 'Low Disk Space', 2: 'Very Low Disk Space' }


## Connects to hddtemp daemon to get temp, HD make.
def get_hddtemp_data_socket(hostname = 'localhost', port = 7634):
    try:
        hd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        hd_sock.connect((hostname, port))
        sock_data = ''
        while True:
            newdat = hd_sock.recv(1024)
            if len(newdat) == 0:
                break
            sock_data = sock_data + newdat
        hd_sock.close()
        
        sock_vals = sock_data.split('|')

        # Format of output looks like ' | DRIVE | MAKE | TEMP | ' 
        idx = 0
        
        drives = []
        makes = []
        temps = []
        while idx + 5 < len(sock_vals):
            drives.append(sock_vals[idx + 1])
            makes.append(sock_vals[idx + 2])
            temps.append(sock_vals[idx + 3])
                        
            idx = idx + 5

        return True, drives, makes, temps
    except:
        rospy.logerr(traceback.format_exc())
        return False, [ 'Exception' ], [ traceback.format_exc() ], [ 100 ]

def update_status_stale(stat, last_update_time):
    time_since_update = rospy.get_time() - last_update_time

    level = stat.level
    stale_status = 'OK'
    if time_since_update > 20:
        stale_status = 'Lagging'
        level = max(level, 1)
    if time_since_update > 35:
        stale_status = 'Stale'
        level = max(level, 2)
        
    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
    stat.values.insert(1, KeyValue(key = 'Time Since Update', value = str(time_since_update)))

class hdMonitor():
    def __init__(self, hostname, home_dir = ''):
        rospy.init_node('hd_monitor_%s' % hostname)

        self._mutex = threading.Lock()
        
        self._hostname = hostname
        self._no_temp_warn =  rospy.get_param('no_hd_temp_warn', False)
        self._home_dir = home_dir

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = "%s HD Temperature" % hostname
        self._temp_stat.level = 2
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data'), 
                                   KeyValue(key = 'Time Since Last Update', value = str(100000) )]

        if self._home_dir != '':
            self._usage_stat = DiagnosticStatus()
            self._usage_stat.level = 2
            self._usage_stat.hardware_id = hostname
            self._usage_stat.name = '%s HD Usage' % hostname
            self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                        KeyValue(key = 'Time Since Last Update', value = str(100000)) ]
            self.check_disk_usage()

        self._last_temp_time = 0
        self._last_usage_time = 0
        self._last_publish_time = 0
        
        self.check_temps()

        self._temp_timer = None
        self._usage_timer = None
        #self._publish_timer = threading.Timer(1.0, self.publish_stats)
        #self._publish_timer.start()
        
    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temp_timer:
            self._temp_timer.cancel()
            self._temp_timer = None
 
        if self._usage_timer:
            self._usage_timer.cancel()
            self._usage_timer = None

    def check_temps(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return

        diag_strs = [ KeyValue(key = 'Update Status', value = 'OK' ) ,
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_level = 0
        diag_message = 'OK'
                
        temp_ok, drives, makes, temps = get_hddtemp_data_socket()

        for index in range(0, len(drives)):
            temp = temps[index]
            
            if not unicode(temp).isnumeric():
                temp_level = 2
            else:
                temp_level = 0
                if float(temp) > hd_temp_warn:
                    temp_level = 1
                if float(temp) > hd_temp_error:
                    temp_level = 2
                diag_level = max(diag_level, temp_level)
            
            diag_strs.append(KeyValue(key = 'Disk %d Temp Status' % index, value = temp_dict[temp_level]))
            diag_strs.append(KeyValue(key = 'Disk %d Mount Pt.' % index, value = drives[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Device ID' % index, value = makes[index]))
            diag_strs.append(KeyValue(key = 'Disk %d Temp' % index, value = temp))
        
        if not temp_ok:
            diag_level = 2

        self._mutex.acquire()
        self._last_temp_time = rospy.get_time()

        self._temp_stat.values = diag_strs
        
        self._temp_stat.level = diag_level

        # Set HW ID to makes
        self._temp_stat.hardware_id = makes[0]

        # Give No Data message if we have no reading
        self._temp_stat.message = temp_dict[diag_level]
        if not temp_ok:
            self._temp_stat.message = 'Error'

        if self._no_temp_warn and temp_ok:
            self._temp_stat.level = 0


        if not rospy.is_shutdown():
            self._temp_timer = threading.Timer(10.0, self.check_temps)
            self._temp_timer.start()
        else:
            self.cancel_timers()

        self._mutex.release()
        
    def check_disk_usage(self):
        if rospy.is_shutdown():
            self._mutex.acquire()
            self.cancel_timers()
            self._mutex.release()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_level = 0
        diag_message = 'OK'
        
        try:
            p = subprocess.Popen(["df", "-P", "--block-size=1G", self._home_dir], 
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            
            if (retcode == 0):
                
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'OK'))
                row_count = 0
                for row in stdout.split('\n'):
                    if len(row.split()) < 2:
                        continue
                    if not unicode(row.split()[1]).isnumeric() or float(row.split()[1]) < 10: # Ignore small drives
                        continue
                
                    row_count += 1
                    g_available = row.split()[-3]
                    name = row.split()[0]
                    size = row.split()[1]
                    mount_pt = row.split()[-1]
                    
                    if (float(g_available) > low_hd_level):
                        level = 0
                    elif (float(g_available) > critical_hd_level):
                        level = 1
                    else:
                        level = 2
                        
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Name' % row_count, value = name))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Available' % row_count, value = g_available))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Size' % row_count, value = size))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Status' % row_count, value = stat_dict[level]))
                    diag_vals.append(KeyValue(
                            key = 'Disk %d Mount Point' % row_count, value = mount_pt))
                    
                    diag_level = max(diag_level, level)
                    diag_message = usage_dict[diag_level]

            else:
                diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Failed'))
                diag_level = 2
                diag_message = stat_dict[diag_level]
            
                    
        except:
            rospy.logerr(traceback.format_exc())
            
            diag_vals.append(KeyValue(key = 'Disk Space Reading', value = 'Exception'))
            diag_vals.append(KeyValue(key = 'Disk Space Ex', value = traceback.format_exc()))

            diag_level = 2
            diag_message = stat_dict[diag_level]
            
        # Update status
        self._mutex.acquire()
        self._last_usage_time = rospy.get_time()
        self._usage_stat.level = diag_level
        self._usage_stat.values = diag_vals
        self._usage_stat.message = diag_message

        if not rospy.is_shutdown():
            self._usage_timer = threading.Timer(5.0, self.check_disk_usage)
            self._usage_timer.start()
        else:
            self.cancel_timers()

        self._mutex.release()

        
    def publish_stats(self):
        self._mutex.acquire()
        update_status_stale(self._temp_stat, self._last_temp_time)
        
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status.append(self._temp_stat)
        if self._home_dir != '':
            update_status_stale(self._usage_stat, self._last_usage_time)
            msg.status.append(self._usage_stat)
        
        if rospy.get_time() - self._last_publish_time > 0.5:
            self._diag_pub.publish(msg)
            self._last_publish_time = rospy.get_time()
            
        self._mutex.release()


        
# TODO: Need to check HD input/output too using iostat

if __name__ == '__main__':
    hostname = socket.gethostname()

    home_dir = ''
    if len(rospy.myargv()) > 1:
        home_dir = rospy.myargv()[1]

        
    hd_monitor = hdMonitor(hostname, home_dir)
    rate = rospy.Rate(1.0)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
            hd_monitor.publish_stats()
    #finally:
    except Exception, e:
        traceback.print_exc()

    print 'finishing'
    hd_monitor.cancel_timers()
    sys.exit(0)
    

            
