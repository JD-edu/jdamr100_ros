# jdamr100_bridge.py
### **📌 ROS2 JDAMR100 Bridge Node Explanation (jdamr100_bridge.py)**  

This **ROS2 node (`jdamr100_bridge`)** acts as a **Wi-Fi TCP bridge** between the **ROS2 system** and the **ESP32 motor controller**.  

### **✅ Command Flow in the JDAMR100 System**  
```
ROS2 teleop node  →  ROS2 bridge node  →  Wi-Fi  →  ESP32  →  Serial  →  Arduino  →  Motor
```
1. The **teleop node** sends keyboard commands (`w`, `a`, `s`, `d`, `space`) to the `key` topic.  
2. This **bridge node (`jdamr100_bridge`)** subscribes to the `key` topic and sends commands to ESP32 over Wi-Fi (TCP).  
3. **ESP32 receives the command** and forwards it via Serial to **Arduino**.  
4. **Arduino executes the command** by controlling the motor driver (PWM signals).  

---

## **📌 Line-by-Line Explanation**

### **1️⃣ Import Required Libraries**
```python
import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import String
```
- **rclpy** → ROS2 client library for Python.  
- **socket** → Used to create a TCP connection to ESP32.  
- **Node** → Base class for ROS2 nodes.  
- **String** → ROS2 message type used to receive keyboard commands from the teleop node.  

---

### **2️⃣ Define the ROS2 Node**
```python
class Jdamr100Bridge(Node):
    def __init__(self):
        super().__init__('jdamr100_bridge')
```
- **Creates a ROS2 node named `jdamr100_bridge`**.  
- This node **subscribes to the `key` topic** and sends data to ESP32 over Wi-Fi.  

---

### **3️⃣ Subscribe to ROS2 Key Command Topic**
```python
        self.subscription_keyinput = self.create_subscription(String, 'key', self.key_callback, 10)
```
- **Subscribes to the ROS2 `key` topic** to receive keyboard control commands.  
- Calls the **`key_callback` function** when a message is received.  

---

### **4️⃣ Define ESP32 Wi-Fi Connection**
```python
        self.host = '172.30.1.38'
        self.port = 8080
```
- **ESP32's IP address (`172.30.1.38`) and TCP port (`8080`)**.  
- The ROS2 node will **connect as a client** to this ESP32 server.  

---

### **5️⃣ Create TCP Connection to ESP32**
```python
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.host, self.port))
            self.get_logger().info(f'Connected to {self.host}:{self.port}')
        except socket.error as e:
            self.get_logger().error(f"Failed to con-nect to server: {e}")
```
- **Creates a TCP socket (`AF_INET` = IPv4, `SOCK_STREAM` = TCP socket).**  
- **Attempts to connect to ESP32's Wi-Fi server**.  
- **If successful** → Logs "Connected to ESP32".  
- **If connection fails** → Logs an error message.  

---

### **6️⃣ Handle Incoming Key Commands**
```python
    def key_callback(self, msg):
        print(msg.data)
        self.client.send(msg.data.encode())
```
- **Called when a message is received on the `key` topic**.  
- **Sends the received command (keyboard key) as a TCP message to ESP32**.  
- **Example**: If user presses `"w"` (forward command), `"w"` is sent to ESP32 via Wi-Fi.  

---

### **7️⃣ Utility Function to Constrain Values (Unused Here)**
```python
def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)
```
- **Ensures values remain within a given range (`min_value` to `max_value`)**.  
- **⚠️ Note:** This function is not used in this code version.  

---

### **8️⃣ Main ROS2 Execution Loop**
```python
def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = Jdamr100Bridge()
```
- **Initializes ROS2** and creates an instance of `Jdamr100Bridge`.  
- The node starts listening for key commands and sending them to ESP32.  

```python
    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        cmd_vel_subscriber.get_logger().info("Shutting down...")
```
- **Keeps the node running** until manually stopped (`Ctrl + C`).  
- **If interrupted**, logs "Shutting down..."  

---

### **9️⃣ Send Stop Command on Shutdown**
```python
    finally:
        try:
            cmd_vel_subscriber.client.send(' \n'.encode())
            cmd_vel_subscriber.get_logger().info('Sent stop command')
        except socket.error as e:
            cmd_vel_subscriber.get_logger().error(f"Failed to send stop command: {e}")
```
- **Sends a space character `" "` (stop command) to ESP32 when shutting down.**  
- Ensures **the robot stops moving** when ROS2 is terminated.  

```python
        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()
```
- **Cleans up the ROS2 node and shuts down ROS2.**  

---

## **✅ Summary of Key Features**
✔ **Receives ROS2 keyboard commands (`w`, `s`, `a`, `d`, `space`).**  
✔ **Sends commands to ESP32 via Wi-Fi TCP socket.**  
✔ **ESP32 forwards the command to Arduino via Serial.**  
✔ **Ensures robot stops when ROS2 node is closed.**  

This **ROS2 bridge node (`jdamr100_bridge.py`)** is responsible for **converting ROS2 messages into Wi-Fi commands** for **ESP32, enabling wireless teleoperation of JDAMR100**. 🚀

# jdamr100_teleop.py

### **📌 ROS2 Teleop Keyboard Node Explanation (`teleop_keyboard.py`)**  

This **ROS2 Python script** allows a user to **control the JDAMR100 robot using keyboard input**.  
It **reads user key presses (`w`, `a`, `s`, `d`, `space`) and publishes them to the `key` topic**.  

---

## **✅ ROS2 System Configuration (Briefly)**
```
[User Keyboard Input]  →  [ROS2 Teleop Node]  →  [ROS2 Bridge Node]  →  [Wi-Fi (TCP)]  
                     →  [ESP32]  →  [Serial]  →  [Arduino]  →  [Motor Control]
```
1. **This node (`teleop_keyboard.py`) reads keyboard input and publishes it on the `key` topic.**  
2. **The bridge node (`jdamr100_bridge.py`) subscribes to `key` and sends commands to ESP32 over Wi-Fi.**  
3. **ESP32 forwards commands to Arduino, which controls the motors.**  

---

## **📌 Line-by-Line Code Explanation**

### **1️⃣ Import Required Libraries**
```python
#!/usr/bin/env python
import socket
import os
import sys
import select
import rclpy
import termios
import tty

from std_msgs.msg import String
from rclpy.qos import QoSProfile
```
- **`rclpy`** → ROS2 Python client library.  
- **`termios`, `tty`** → Used to handle terminal input (get single key press).  
- **`select`** → Monitors user input without blocking execution.  
- **`std_msgs.msg.String`** → Used to send keyboard input as a ROS2 message.  

---

### **2️⃣ Display User Instructions**
```python
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
```
- **`msg`** → Instructions displayed when the script starts.  
- **`e`** → Error message (not used in this version).  

---

### **3️⃣ Function: Get User Key Press**
```python
def get_key(settings):
    tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode (reads single key press)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Wait for key input (timeout = 0.1s)
    if rlist:
        key = sys.stdin.read(1)  # Read one character from the keyboard
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings
    return key
```
- **Reads a single key press** without requiring Enter.  
- Uses `select.select()` to **check if input is available** without blocking execution.  

---

### **4️⃣ Main Function: Initialize ROS2 and Listen for Key Input**
```python
def main():
    print(msg)
    settings = termios.tcgetattr(sys.stdin)  # Save terminal settings

    rclpy.init()  # Initialize ROS2

    qos = QoSProfile(depth=10)  # Quality of Service profile
    node = rclpy.create_node('teleop_keyboard')  # Create ROS2 node
    pub_key = node.create_publisher(String, 'key', qos)  # Create publisher for `key` topic
```
- **Displays movement instructions.**  
- **Initializes ROS2 and creates a publisher** for the `key` topic.  

---

### **5️⃣ Loop: Read Key Input and Publish to ROS2**
```python
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
                if (key == '\x03'):  # Ctrl+C pressed (exit)
                    break
            key_msg = String()
            key_msg.data = key_input
            pub_key.publish(key_msg)  # Publish the key press to the `key` topic
```
- **Continuously reads user key input.**  
- **Recognized keys (`w`, `a`, `s`, `d`, `space`) are stored in `key_input`.**  
- **Publishes the key command to the `key` topic.**  
- **Exits loop if Ctrl+C (`\x03`) is pressed.**  

---

### **6️⃣ Handle Exceptions & Ensure Robot Stops on Exit**
```python
    except Exception as e:
        print(e)
```
- **Handles unexpected errors gracefully.**  

```python
    finally:
        key_msg = String()
        key_msg.data = ' \n'  # Stop command (space key)
        pub_key.publish(key_msg)  # Send stop command before exiting

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings
```
- **Ensures the robot stops when the script exits** (publishes a space key `" "`).  
- **Restores terminal settings to normal.**  

---

### **7️⃣ Run the Main Function**
```python
if __name__ == '__main__':
    main()
```
- **Runs the `main()` function when the script starts.**  

---

## **✅ Summary of Key Features**
✔ **Reads user keyboard input (`w`, `a`, `s`, `d`, `space`).**  
✔ **Publishes commands to the `key` topic.**  
✔ **Ensures robot stops when exiting (Ctrl+C).**  
✔ **Allows real-time keyboard control of JDAMR100.**  

This **ROS2 teleop node (`teleop_keyboard.py`)** enables **wireless manual control of the JDAMR100 robot using ROS2 topics and ESP32 Wi-Fi communication**. 🚀
