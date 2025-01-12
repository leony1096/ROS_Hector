#!/usr/bin/env python

import roslib, rospy, rospkg
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int8, Bool
from math import sqrt, cos, sin, pi, atan2
import sys

from hector_constants import *
from ee4308_bringup.msg import EE4308MsgMotion

# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.05

# =============================== SUBSCRIBERS =========================================  
def subscribe_target(msg):
    global msg_target
    msg_target = msg.point

def subscribe_stop(msg):
    global msg_stop
    msg_stop = msg.data
    
def subscribe_state(msg):
    global msg_state
    msg_state = msg.data
    
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg
       
# ================================ BEGIN ===========================================
def move():
    # ---------------------------------- INITS ----------------------------------------------
    # --- init node ---
    rospy.init_node('hector_move')
    
    if DISABLE_MOVE:
        return
        
    # --- cache global vars / constants ---
    global msg_stop, msg_motion, msg_state, msg_target
    msg_motion = None
    msg_target = None
    msg_state = None
    msg_stop = None
    
    # --- Service: Enable Motors ---
    enable_motors = rospy.ServiceProxy('/hector/enable_motors', EnableMotors)
    # Shutdown handler
    def shutdown_handler():
        # Messages cannot be published in topics
        # Disable motors   
        enable_motors(False)
        print('[HEC MOVE  ] Motors Disabled')
    rospy.on_shutdown(shutdown_handler)
    
    # --- Subscribers ---
    rospy.Subscriber('/hector/target', PointStamped, subscribe_target, queue_size=1)
    rospy.Subscriber('/hector/stop', Bool, subscribe_stop, queue_size=1)
    rospy.Subscriber('/hector/state', Int8, subscribe_state, queue_size=1)
    rospy.Subscriber('/hector/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    while (msg_motion is None or msg_stop is None or msg_state is None or msg_target is None or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
    if rospy.is_shutdown():
        return
        
    print('=== [HEC  MOVE ] Initialised ===')
    
    # --- Publishers ---
    pub_cmd = rospy.Publisher('/hector/cmd_vel', Twist, latch=True, queue_size=1)
    cmd_vel = Twist()
    cmd_lin = cmd_vel.linear
    cmd_ang = cmd_vel.angular
    
    # --- Enable motors ---
    enable_motors(True)
    print('[HEC  MOVE ] Enabled motors')
    
    # --- Inits ---
    prev_err_x = 0
    prev_err_y = 0
    prev_err_z = 0
    accum_err_x = 0.
    accum_err_y = 0.
    accum_err_z = 0.
    prev_tx = 0
    prev_ty = 0
    prev_tz = 0
    
    # ---------------------------------- LOOP ----------------------------------------------
    t = rospy.get_time();
    while not rospy.is_shutdown() and not msg_stop:
        if rospy.get_time() > t:
            # --- Get pose ---
            rx = msg_motion.x
            ry = msg_motion.y
            rz = msg_motion.z
            ro = msg_motion.o
            
            # --- Get target ---
            tx = msg_target.x
            ty = msg_target.y
            tz = msg_target.z
            
            # --- Anti windup ---
            if (prev_tx != tx):
                accum_err_x = 0.
                prev_tx = tx
            if (prev_ty != ty):
                accum_err_y = 0.
                prev_ty = ty
            if (prev_tz != tz):
                accum_err_z = 0.
                prev_tz = tz    
            
            # --- Get Errors ---
            Dx = tx - rx
            Dy = ty - ry
            err_o = atan2(Dy, Dx) - ro
            err_r = sqrt(Dx*Dx + Dy*Dy)
            err_x = err_r * cos(err_o)
            err_y = err_r * sin(err_o) 
            
            # --- PID for x ---
            accum_err_x += err_x
            cmd_x = KP_X * err_x + KD_X * (err_x - prev_err_x) + KI_X * accum_err_x
            prev_err_x = err_x
            
            # --- PID for y ---
            accum_err_y += err_y
            cmd_y = KP_Y * err_y + KD_Y * (err_y - prev_err_y) + KI_Y * accum_err_y
            prev_err_y = err_y
            
            # --- Constrain linear speed v ---
            cmd_v = sqrt(cmd_x*cmd_x + cmd_y*cmd_y)
            if (cmd_v > MAX_V):
                tmp = MAX_V / cmd_v
                cmd_x *= tmp
                cmd_y *= tmp
            
            # --- PID for z ---
            err_z = tz - rz
            accum_err_z += err_z
            cmd_z = KP_Z * err_z + KD_Z * (err_z - prev_err_z) + KI_Z * accum_err_z
            prev_err_z = err_z           
            
            # --- Constrain z ---
            if (cmd_z > MAX_ASCEND):
                cmd_z = MAX_ASCEND
            elif (cmd_z < MAX_DESCEND):
                cmd_z = MAX_DESCEND
            
            # --- Command ---
            cmd_lin.y = cmd_y
            cmd_lin.x = cmd_x
            cmd_lin.z = cmd_z
            if msg_state == STATE_TAKEOFF or msg_state == STATE_LAND:
                cmd_ang.z = 0
            else:
                cmd_ang.z = MAX_W
            pub_cmd.publish(cmd_vel)
            
            # --- Timing ---
            et = rospy.get_time() - t
            t += ITERATION_PERIOD
            if et > ITERATION_PERIOD:
                print('[HEC  MOVE ] {} OVERSHOOT'.format(int(et*1000)))
    
    
if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
        
    print('=== [HEC  MOVE ] Terminated ===')
