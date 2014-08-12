#!/usr/bin/env python
import os
import sys
import struct
import time

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header, Float64MultiArray
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

import roscopter.msg

px4_time = [0]      #px4 time (from ATTITUDE #30)
local_time = [0,0]  #local start time and end time


mavlink_dir = os.path.realpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', 'mavlink'))
sys.path.insert(0, mavlink_dir)

pymavlink_dir = os.path.join(mavlink_dir, 'pymavlink')
sys.path.insert(0, pymavlink_dir)


from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyUSB0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

from pymavlink import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)



if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

#this function does not work for px4
def send_rc(data):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
	#mavutil.mavlink.MAV_COMP_ID_ALL,
        data.channel[0],
        data.channel[1],
        data.channel[2],
        data.channel[3],
        data.channel[4],
        data.channel[5],
        data.channel[6],
        data.channel[7])
    print "sending rc: %s" % data

def set_arm(req):
    '''arm motors (arducopter only)'''
    master.mav.command_long_send(
         master.target_system,  # target_system
         mavutil.mavlink.MAV_COMP_ID_ALL, # target_component
         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
         0, # confirmation
         1, # param1 (1 to indicate arm)
         0, # param2 (all other params meaningless)
         0, # param3
         0, # param4
         0, # param5
         0, # param6
         0) # param7
    return []

def set_disarm(req):
    '''disarm motors (arducopter only)'''
    master.mav.command_long_send(
         master.target_system,  # target_system
         mavutil.mavlink.MAV_COMP_ID_ALL, # target_component
         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
         0, # confirmation
         0, # param1 (1 to indicate arm)
         0, # param2 (all other params meaningless)
         0, # param3
         0, # param4
         0, # param5
         0, # param6
         0) # param7
    return []


####################################### handy functions

# handle function for displaying received messages 
def pose_play(data):
   rospy.loginfo("x: %s, y: %s, z: %s", data.data[0], data.data[1], data.data[2])

# template function for sending test
def send_template(data):
    master.mav.local_position_ned_send(
        50,
        data.x,
        data.y,
        data.z,
        data.vx,
	data.vy,
	data.vz)
    rospy.logdebug("sending template message: %s" % data)

####################################### mavlink sending functions 

# function for sending #76 message
def send_cmd(data):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        data.channel[0],
        data.channel[1],
        data.channel[2],
        data.channel[3],
        data.channel[4],
        data.channel[5],
        data.channel[6],
        data.channel[7],
        data.channel[8])
    rospy.logdebug("sending #76 message: %s" % data)

#function for sending #32 message
def send_32(data):
    local_time[1] = time.time() # get local end time
    master.mav.local_position_ned_send(
        px4_time[0]*1000 + (local_time[1]-local_time[0])*1000000,
        data.data[0], #pos.x
        data.data[1], #pos.y
        data.data[2], #pos.z
        data.data[3], #pos.yaw (abused vx)
        1.0,
	1.0)
    rospy.logdebug("sending #32 message: %s" % data)

#function for sending abused #89 message(abused for pose_estimator)
def send_89(data):
    
    local_time[1] = time.time() # get the end time of local time
    rospy.logdebug(" #89 start: %s, end: %s, dt: %s" % (local_time[0],local_time[1],local_time[1]-local_time[0])) # #89 processing time info

    master.mav.local_position_ned_system_global_offset_send(
        px4_time[0]*1000 + (local_time[1]-local_time[0])*1000000,
        data.data[0], #pos.x
        data.data[1], #pos.y
        data.data[2], #pos.z
	1.0,
	1.0,
        data.data[3], #pos.yaw
	)
    rospy.logdebug("sending #89 message: %s" % data)

############################################################

pub_gps = rospy.Publisher('gps', NavSatFix)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC)
pub_state = rospy.Publisher('state', roscopter.msg.State)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU)

#subscribe pose from pose_estimator
rospy.Subscriber("simplePose", Float64MultiArray, send_89)
#rospy.Subscriber("send_template", roscopter.msg.template, send_template)

if opts.enable_control:
    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc)
    rospy.Subscriber("send_cmd", roscopter.msg.long_cmd, send_cmd)

#define service callbacks
arm_service = rospy.Service('arm', Empty, set_arm)
disarm_service = rospy.Service('disarm', Empty, set_disarm)


#state
gps_msg = NavSatFix()



def mainloop():

    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        #print msg.get_type()
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else: 
            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_RAW" :
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
            if msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            if msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)

            if msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix=NavSatStatus.STATUS_FIX
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS) 
                                          ))
            #pub.publish(String("MSG: %s"%msg))
            if msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
		
		px4_time[0] = msg.time_boot_ms # get px4 time

		local_time[0] = time.time() # get local start time

            if msg_type == "LOCAL_POSITION_NED" :
                rospy.loginfo("Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz))

            if msg_type == "RAW_IMU" :
                pub_raw_imu.publish (Header(), msg.time_usec, 
                                     msg.xacc, msg.yacc, msg.zacc, 
                                     msg.xgyro, msg.ygyro, msg.zgyro,
                                     msg.xmag, msg.ymag, msg.zmag)


wait_heartbeat(master)

print("Sending all stream request for rate %u" % opts.rate)
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    opts.rate,
    1)

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
