# **JDAMR100 ROS2 Package**  
An **affordable and simple** ROS2-compatible **Arduino + ESP32** robot car for learning ROS2  

## **1. What is jdamr100_ros?**  
`jdamr100_ros` is a **ROS2 package** designed for the **JDAMR100**, a **two-wheel Arduino + ESP32-based robot car**.  

Most ROS2-compatible robot cars are built using **Raspberry Pi or Jetson Nano**, costing **$500 to $1000**. These expensive robots often include **high-level features**, making the software complex and difficult for beginners to learn.  

In contrast, **JDAMR100** is a **low-cost, easy-to-understand** robot that focuses on **fundamental ROS2 hardware integration**. This makes it **ideal for beginners** who want to learn **ROS2 motor control and communication without unnecessary complexity**.  

🔹 **GitHub Repository**: [jdamr100_ros](https://github.com/JD-edu/jdamr100_ros)  

---

## **2. Getting Started**  

To use the **jdamr100_ros** package, follow these steps:  

### **Step 1: Program ESP32 and Arduino**  
Before running the ROS2 package, you need to **flash the ESP32 and Arduino firmware**.  

1. **Flash ESP32** firmware using `110_esp32_socket_serial_bridge.ino`.  
2. **Flash Arduino** firmware using `109_arduino_motor_remote_control.ino`.  

### **Step 2: Check ESP32 to Arduino Communication**  
Before running ROS2 nodes, ensure that **ESP32 and Arduino communicate properly** over the **serial port**.  

- **Arduino uses SoftwareSerial (Pins 7 and 8)** for communication.  
- **ESP32 uses Serial2** for communication.  

#### **Debugging the Communication**  
You can use the **Serial Monitor** in **Arduino IDE** to check the communication:  

1. **Open two Arduino IDE instances** – one for **Arduino**, one for **ESP32**.  
2. **Connect each board separately and open the Serial Monitor** for debugging.  
3. **Verify that commands sent from ESP32 to Arduino as following**.
4. **Send Commands to Move the Motors**  
To move the motors, the command follows this **communication pipeline**:

```
PC (Python Code) -> ESP32 -> Arduino -> Motor Driver -> Motors
```

To **send motor control commands**, you can use the Python script **`110_esp32_socket_comm.py`**:

```bash
python3 110_esp32_socket_comm.py
```  

### **Step 3: Install and Build jdamr100_ros Package**  
Once ESP32 and Arduino communicate successfully, proceed to install the `jdamr100_ros` package.  

#### **1) Clone the repository**
```bash
cd ~/ros2_ws/src
git clone https://github.com/JD-edu/jdamr100_ros.git
```

#### **2) Build the package**
```bash
cd ~/ros2_ws
colcon build --packages-select jdamr100_ros
source install/setup.bash
```

### **Step 4: Run jdamr100_bridge Node**  
The **jdamr100_bridge** node is responsible for **sending motor control commands** to the ESP32 via **WiFi**.  

```bash
ros2 run jdamr100_ros jdamr100_bridge
```

### **Step 5: Run jdamr100_teleop Node**  
The **jdamr100_teleop** node allows users to **control the robot using keyboard commands**.  

```bash
ros2 run jdamr100_ros jdamr100_teleop
```

#### **Keyboard Controls:**
- **W** → Move forward  
- **A** → Turn left  
- **S** → Move backward  
- **D** → Turn right  
- **Space** → Stop  

### **Step 7: Communication Flow with ROS2 Nodes**  
Once both nodes are running, the **motor control commands follow this pipeline**:

```
User Keyboard Input (jdamr100_teleop)  
        ↓  
ROS2 Topic (jdamr100_bridge)  
        ↓  
WiFi Communication (ESP32)  
        ↓  
Serial Communication (ESP32 -> Arduino)  
        ↓  
Motor Control Execution (Arduino)
```

---

## **3. Pros and Cons of jdamr100_ros**  

### ✅ **Pros (Advantages)**  
- **Low Cost** → No need for expensive Raspberry Pi or Jetson Nano.  
- **Beginner-Friendly** → Ideal for learning **basic ROS2 hardware communication**.  
- **Simple Teleoperation** → Easily control the robot using a keyboard.  
- **Hardware Integration** → Demonstrates **WiFi (ESP32) & Serial (Arduino) communication**.  

### ❌ **Cons (Limitations)**  
- **Limited Functionality** → No **advanced features** like SLAM, LiDAR, or computer vision.  
- **Only Supports Teleoperation** → No **autonomous navigation** (manual control required).  
- **Not Scalable for Advanced ROS2** → For **advanced robotics**, a **Raspberry Pi-based** robot is recommended.  

---

Let me know if you need any further modifications! 🚀😊
