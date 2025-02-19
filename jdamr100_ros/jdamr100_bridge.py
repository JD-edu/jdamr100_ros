import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import Int64, String

class Jdamr100Bridge(Node):
    def __init__(self):
        super().__init__('jdamr100_bridge')
     
        self.subscription_keyinput = self.create_subscription(String, 'key', self.key_callback, 10)
       
        self.host = '172.30.1.38'
        self.port = 8080
        
        # esp32 connect 
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.host, self.port))
            self.get_logger().info(f'Connected to {self.host}:{self.port}')
        except socket.error as e:
            self.get_logger().error(f"Failed to con-nect to server: {e}")

    def key_callback(self, msg):
        print(msg.data)
        self.client.send(msg.data.encode())

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
        motor_control = cmd_vel_subscriber.motor_speed_vel(0, 0)
        try:
            cmd_vel_subscriber.client.send(' \n'.encode())
            cmd_vel_subscriber.get_logger().info(f'Sent stop command: {motor_control}')
        except socket.error as e:
            cmd_vel_subscriber.get_logger().error(f"Failed to send stop command: {e}")

        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

