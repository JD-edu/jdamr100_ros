#!/usr/bin/env python

import socket
import os
import sys
import select
import rclpy
import termios
import tty

from std_msgs.msg import Int64, String
from rclpy.qos import QoSProfile

LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 10

max_lin_vel = 250
min_lin_vel = 0

max_ang_vel = 250
min_ang_vel = -250

msg = """
Control
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

def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def check_linear_limit_velocity(velocity):
    return constrain(velocity, min_lin_vel, max_lin_vel)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, min_ang_vel, max_ang_vel)

def main():
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    #pub_linear = node.create_publisher(Int64, 'linear', qos)
    #pub_angular = node.create_publisher(Int64, 'angular', qos)
    pub_key = node.create_publisher(String, 'key', qos)

    '''
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    '''
    key_input = ' \n'
    try:
        while(1):
            
            key = get_key(settings)
            if key == 'w':
                key_input= 'w\n'
                print(key_input)
                #target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                #print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 's':
                key_input= 's\n'
                print(key_input)
                #target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                #print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                key_input= 'a\n'
                print(key_input)
                #target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                #print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                key_input= 'd\n'
                print(key_input)
                #target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                #print_vels(target_linear_velocity, target_angular_velocity)

            elif key == ' ' or key == 'p':
                key_input= ' \n'
                print(key_input)
                #target_linear_velocity = 0.0
                #control_linear_velocity = 0.0
                #target_angular_velocity = 0.0
                #control_angular_velocity = 0.0
                #print_vels(target_linear_velocity, target_angular_velocity)

            else:
                if (key == '\x03'):
                    break

            '''
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            linear_msg = Int64()
            linear_msg.data = int(target_linear_velocity)
            pub_linear.publish(linear_msg)

            angular_msg = Int64()
            angular_msg.data = int(target_angular_velocity)
            '''
            key_msg = String()
            key_msg.data = key_input
            #print(key_msg)
            pub_key.publish(key_msg)

    except Exception as e:
        print(e)

    finally:
        #linear_msg = Int64()
        #linear_msg.data = 0
        #pub_linear.publish(linear_msg)

        #angular_msg = Int64()
        #angular_msg.data = 0
        #pub_angular.publish(angular_msg)
        key_msg = String()
        key_msg.data = ' \n'
        #print(key_msg)
        pub_key.publish(key_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':#
    main()

