#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# Modifications Copyright (c) 2019, Couchmobile
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import time
from math import exp
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

NORMAL_MAX_LIN_VEL = 0.4
NORMAL_MAX_ANG_VEL = 2.0

SLOW_MAX_LIN_VEL = 0.2
SLOW_MAX_ANG_VEL = 1.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

# Returns string containing operational information
# e.g. Operating mode, speed
def getModeMsg():
    ret = ""
    ret = ret + "Operating mode: " + operating_mode + "\n"
    
    minSpeed, maxSpeed = 0,0
    minTurn, maxTurn = 0,0
    if operating_mode == "normal":
        minSpeed = -NORMAL_MAX_LIN_VEL
        maxSpeed = NORMAL_MAX_LIN_VEL
        minTurn = -NORMAL_MAX_ANG_VEL
        maxTurn = NORMAL_MAX_ANG_VEL
    else:
        minSpeed = -SLOW_MAX_LIN_VEL
        maxSpeed = SLOW_MAX_LIN_VEL
        minTurn = -SLOW_MAX_ANG_VEL
        maxTurn = SLOW_MAX_ANG_VEL
    
    ret = ret + "Speed: " + str(minSpeed) + " - " + str(maxSpeed) + "\n"
    ret = ret + "Angular speed: " + str(minTurn) + " - " + str(maxTurn) + "\n"
    return ret

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel, actual_linear_vel,
        actual_angular_vel):
    return "desired:\tlinear vel %s\t angular vel %s \nactual:\tlinear vel %s\tangular vel %s " % (target_linear_vel,target_angular_vel, actual_linear_vel, actual_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def exponentialProfile(currVal, desiredVal, dt, tau, maxStep):
    # Parses input through an exponential function to smooth steps
    # tau is time constant (s)
    # maxStep is equivalent step over 1s
    delta = 1.0 - exp(-dt/tau)
    delta = delta * (desiredVal - currVal)
    if delta < maxStep*dt:
        delta = -maxStep*dt
    elif delta > maxStep*dt:
        delta = maxStep*dt
    return currVal + delta

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if operating_mode == "normal":
      vel = constrain(vel, -NORMAL_MAX_LIN_VEL, NORMAL_MAX_LIN_VEL)
    elif operating_mode == "slow":
      vel = constrain(vel, -SLOW_MAX_LIN_VEL, SLOW_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -SLOW_MAX_LIN_VEL, SLOW_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if operating_mode == "normal":
      vel = constrain(vel, -NORMAL_MAX_ANG_VEL, NORMAL_MAX_ANG_VEL)
    elif operating_mode == "slow":
      vel = constrain(vel, -SLOW_MAX_ANG_VEL, SLOW_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -SLOW_MAX_ANG_VEL, SLOW_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    operating_mode = rospy.get_param("operating_mode", "normal")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    prevTime = time.time();

    try:
        print(msg)
        print(getModeMsg())
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel,control_linear_vel,control_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel,control_linear_vel,control_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel,control_linear_vel,control_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel,control_linear_vel,control_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel,control_linear_vel,control_angular_vel))
            else:
                if (key == '\x03'):
                    break

            print(vels(target_linear_vel, target_angular_vel,control_linear_vel,control_angular_vel))
            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()
            tau = 0.5

            dt = time.time() - prevTime
            prevTime = time.time()
            control_linear_vel = exponentialProfile(control_linear_vel,
                    target_linear_vel, dt, tau, LIN_VEL_STEP_SIZE/2.0)
            control_angular_vel = exponentialProfile(control_angular_vel,
                    target_angular_vel, dt, tau, ANG_VEL_STEP_SIZE/2.0)
            #control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            #control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except Exception as ex:
        print(e)
        print(ex)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

