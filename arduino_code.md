# 109_arduino_motor_remote_control.ino 
### **Arduino Motor Control Code Explanation (Line-by-Line)**  

This code controls the motors of the **JDAMR100** robot car based on serial commands received from **ESP32 or PC**. Below is a short explanation of each section.  

---

### **1️⃣ Include Library & Define Pins**  
```cpp
#include <SoftwareSerial.h>  // Library for software serial communication

#define motor_A_enable 12  // Enable pin for Motor A
#define motor_B_enable 13  // Enable pin for Motor B
#define motor_A 10         // PWM pin for Motor A
#define motor_B 11         // PWM pin for Motor B
```
- Defines motor control pins. For motor control driver IC pin setting refer jdAMR100 code.
- Uses **PWM** to control speed via `analogWrite()`.  

---

### **2️⃣ Initialize SoftwareSerial for ESP32 Communication**  
```cpp
SoftwareSerial mySerial(7, 8); // RX = 7, TX = 8
```
- **ESP32 communicates with Arduino using SoftwareSerial (pins 7 and 8).**  

---

### **3️⃣ Setup Function (Runs Once at Start)**  
```cpp
void setup() {
  mySerial.begin(9600);      // Initialize SoftwareSerial (ESP32)
  Serial.begin(115200);      // Initialize Serial Monitor (PC)
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- Starts **serial communication** with both **ESP32** and **PC**.  
- Configures motor control pins as **outputs**.  

---

### **4️⃣ Motor Control Functions**  
```cpp
void forward(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- Moves **both motors forward** by setting enable pins to **LOW** and applying **PWM signals**.  

```cpp
void backward(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- Moves **both motors backward** by setting enable pins to **HIGH**.  

```cpp
void turnLeft(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- **Left turn:** One motor moves forward while the other moves backward.  

```cpp
void turnRight(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- **Right turn:** Opposite motor movement compared to `turnLeft()`.  

```cpp
void stopAll() {
  digitalWrite(motor_A_enable, 0);
  digitalWrite(motor_B_enable, 0);
  analogWrite(motor_A, LOW);
  analogWrite(motor_B, LOW);
}
```
- Stops both motors by setting **enable pins to LOW** and **PWM to 0**.  

---

### **5️⃣ Loop Function (Runs Continuously)**  

#### **🔹 5.1 Check for Commands from ESP32 (Software Serial)**
```cpp
if (mySerial.available()) {
  String incomingStr2 = mySerial.readStringUntil('\n');  
  if(incomingStr2[0] == 'w') forward(255, 255);
  else if(incomingStr2[0] == 's') backward(255, 255);
  else if(incomingStr2[0] == ' ') stopAll();
  else if(incomingStr2[0] == 'a') turnLeft(255, 255);
  else if(incomingStr2[0] == 'd') turnRight(255, 255);
}
```
- **Reads commands from ESP32** via **SoftwareSerial**.  
- **Executes movement functions** based on received character (`w`, `s`, `a`, `d`, `space`).  

#### **🔹 5.2 Check for Commands from PC (USB Serial Monitor)**
```cpp
if (Serial.available()) {
  String incomingStr = Serial.readStringUntil('\n');
  if(incomingStr[0] == 'w') forward(255, 255);
  else if(incomingStr[0] == 's') backward(255, 255);
  else if(incomingStr[0] == ' ') stopAll();
  else if(incomingStr[0] == 'a') turnLeft(255, 255);
  else if(incomingStr[0] == 'd') turnRight(255, 255);
}
```
- **Reads commands from PC** via **Serial Monitor** (USB).  
- Allows direct control **without using ESP32** for debugging.  

#### **🔹 5.3 Add a Small Delay to Prevent Overloading**
```cpp
delay(100);
```
- Prevents **excessive CPU usage** by adding a **100ms delay**.  

---

## **📌 Summary of Key Features**
✅ **Receives commands from ESP32 (SoftwareSerial) and PC (USB Serial Monitor).**  
✅ **Processes movement commands (`w`, `s`, `a`, `d`, `space`).**  
✅ **Controls motor movement using PWM signals.**  
✅ **Supports both remote and direct debugging.**  

This **Arduino motor control code** is the **final execution layer** of **ROS2 motor control** in **JDAMR100**. 🚀


# 110_esp32_socket_serial_bridge.ino
### **ESP32 Wi-Fi to Serial Bridge Code Explanation (110_esp32_socket_serial_bridge.ino)**  

This **ESP32 code** serves as a **Wi-Fi to Serial bridge** for the **JDAMR100 ROS2** robot. It enables the ESP32 to receive motor control commands from a **Wi-Fi-connected ROS2 node** and forward them to **Arduino via Serial2 (pins 16 & 17)**.  

This acts as a **network interface** that allows **remote control of the robot using ROS2 over Wi-Fi**.  

---

## **📌 Role in the Entire ROS2 Package (jdamr100_ros)**
This code plays a **critical role** in enabling **Wi-Fi communication** between **ROS2 and the Arduino motor controller**.  

### **ROS2 Communication Flow**
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

## **📌 Line-by-Line Explanation**  

### **1️⃣ Include Wi-Fi Library & Define Wi-Fi Credentials**
```cpp
#include <WiFi.h>  // ESP32 Wi-Fi library

// Wi-Fi 설정
const char* ssid = "ConnectValue_A403_2G";   // Wi-Fi SSID (Network Name)
const char* password = "CVA403!@#$";         // Wi-Fi Password
```
- **Enables ESP32 Wi-Fi functionality.**  
- Connects to a **specified Wi-Fi network** using `ssid` and `password`.  

---

### **2️⃣ Define Serial Communication Pins**
```cpp
#define RXD2 16   // RX2 pin for Serial2 communication (ESP32 -> Arduino)
#define TXD2 17   // TX2 pin for Serial2 communication (ESP32 -> Arduino)
```
- **ESP32 communicates with Arduino via Serial2 on GPIO 16 (RX2) and GPIO 17 (TX2).**  

---

### **3️⃣ Set Up Wi-Fi and Serial Connections**
```cpp
void setup() {
  Serial.begin(115200);  // Initialize Serial Monitor for debugging
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Initialize Serial2 for Arduino communication
  delay(1000);
```
- Starts **Serial communication** for both **debugging (USB Serial) and Arduino (Serial2)**.  

```cpp
  Serial.println("Wi-Fi 연결 중...");
  WiFi.begin(ssid, password);
```
- Connects ESP32 to the specified **Wi-Fi network**.  

```cpp
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("연결 중...");
  }
```
- **Waits until Wi-Fi connection is established** before proceeding.  

```cpp
  Serial.println("Wi-Fi 연결 완료!");
  Serial.print("ESP32 IP 주소: ");
  Serial.println(WiFi.localIP());
```
- Prints **ESP32’s local IP address**, allowing **ROS2 nodes to communicate with it** over Wi-Fi.  

---

### **4️⃣ Start TCP Server on ESP32**
```cpp
  server.begin();
  Serial.println("서버 시작됨, 클라이언트 대기 중...");
```
- Starts a **TCP server on ESP32 (Port 8080)** to accept **Wi-Fi clients (ROS2 nodes).**  

```cpp
  pinMode(2, OUTPUT);
```
- Configures **GPIO2 as an output pin**, but it's unused in this code.  

---

### **5️⃣ Main Loop: Handling Client Connections**  
```cpp
void loop() {
  WiFiClient client = server.available();  // Check for incoming Wi-Fi clients
```
- Waits for **ROS2 nodes** (or any Wi-Fi clients) to connect to ESP32.  

```cpp
  if (client) {
    Serial.println("클라이언트가 연결되었습니다.");
```
- If a **client (ROS2 bridge node) connects**, print a log message.  

```cpp
    while (client.connected()) {
      if (client.available()) {
```
- Continuously listens for **incoming commands** from ROS2 while the client is connected.  

```cpp
        String message = client.readStringUntil('\n');  // Read incoming message
        Serial.println(message);  // Print received message
        Serial2.println(message);  // Forward message to Arduino (Motor Controller)
```
- **Reads motor control commands from ROS2 via Wi-Fi**.  
- **Sends the same message to Arduino via Serial2**, controlling the motors.  

```cpp
        client.println("데이터 수신 완료");
```
- **Sends acknowledgment** back to the ROS2 client.  

---

### **6️⃣ Close Connection When Client Disconnects**
```cpp
    client.stop();
    Serial.println("클라이언트 연결 종료.");
  }
}
```
- **Terminates connection** when the ROS2 node disconnects.  

---

## **📌 Summary of Key Features**
✅ **Creates a TCP Server on ESP32 to receive commands from ROS2 over Wi-Fi**.  
✅ **Receives motor control commands from ROS2 and forwards them to Arduino via Serial2**.  
✅ **Acts as a network bridge, allowing JDAMR100 to be controlled wirelessly**.  
✅ **Prints debugging logs for easy troubleshooting**.  

This **ESP32 Wi-Fi to Serial Bridge** is the **core network interface** for **controlling the JDAMR100 robot remotely using ROS2**. 🚀
