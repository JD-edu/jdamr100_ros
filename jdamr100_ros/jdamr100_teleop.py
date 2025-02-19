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


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub_key = node.create_publisher(String, 'key', qos)

    key_input = ' \n'
    try:
        while(1):
            
            key = get_key(settings)
            if key == 'w':
                key_input= 'w\n'
                print(key_input)
            elif key == 's':
                key_input= 's\n'
                print(key_input)
            elif key == 'a':
                key_input= 'a\n'
                print(key_input)
            elif key == 'd':
                key_input= 'd\n'
                print(key_input)
            elif key == ' ' or key == 'p':
                key_input= ' \n'
                print(key_input)
            else:
                if (key == '\x03'):
                    break
            key_msg = String()
            key_msg.data = key_input
            #print(key_msg)
            pub_key.publish(key_msg)

    except Exception as e:
        print(e)

    finally:
        key_msg = String()
        key_msg.data = ' \n'
        #print(key_msg)
        pub_key.publish(key_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':#
    main()

