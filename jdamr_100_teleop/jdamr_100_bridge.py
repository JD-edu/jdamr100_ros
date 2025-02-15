'''
MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.
'''

import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import Int64

class Jdamr100Bridge(Node):
    def __init__(self):
        super().__init__('jdamr_100_bridge')
        
        self.subscription_linear = self.create_subscription(Int64, 'linear', self.linear_callback, 10)
        self.subscription_angular = self.create_subscription(Int64, 'angular', self.angular_callback, 10)
        
        self.host = '172.30.1.39'
        self.port = 8080
        
        self.min_vel = 0
        self.max_vel = 250
        # esp32 connect 
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.host, self.port))
            self.get_logger().info(f'Connected to {self.host}:{self.port}')
        except socket.error as e:
            self.get_logger().error(f"Failed to con-nect to server: {e}")

        self.linear_velocity = 0
        self.angular_velocity = 0
        
        self.timer = self.create_timer(0.5, self.send_motor_control)

    def linear_callback(self, msg):
        self.linear_velocity = msg.data

    def angular_callback(self, msg):
        self.angular_velocity = msg.data

    def send_motor_control(self):
        try:
            motor_control = self.motor_speed_vel(self.linear_velocity, self.angular_velocity)
            self.client.send(motor_control.encode())
            self.get_logger().info(f'Sent data: {motor_control}')
        except socket.error as e:
            self.get_logger().error(f"Failed to send data: {e}")
    
    def motor_speed_vel(self, linear, angular):
        vel_R = linear + angular
        vel_L = linear - angular
        
        motor_R = constrain(vel_R, self.min_vel, self.max_vel)
        motor_L = constrain(vel_L, self.min_vel, self.max_vel)
        motor = f"1a{motor_R}b{motor_L}c\n"        
        
        self.get_logger().info(f'motor_R: {motor_R}, motor_L: {motor_L}')
        return motor

def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = Jdamr100Bridge()

    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        cmd_vel_subscriber.get_logger().info("Shutting down...")
    finally:
        cmd_vel_subscriber.linear_velocity = 0
        cmd_vel_subscriber.angular_velocity = 0
        motor_control = cmd_vel_subscriber.motor_speed_vel(0, 0)
        try:
            cmd_vel_subscriber.client.send(motor_control.encode())
            cmd_vel_subscriber.get_logger().info(f'Sent stop command: {motor_control}')
        except socket.error as e:
            cmd_vel_subscriber.get_logger().error(f"Failed to send stop command: {e}")

        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

