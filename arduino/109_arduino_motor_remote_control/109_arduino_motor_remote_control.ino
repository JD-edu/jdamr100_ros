#include <SoftwareSerial.h>

#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX

void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}

void backward(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void forward(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void turnLeft(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void turnRight(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void stopAll() {
  digitalWrite(motor_A_enable, 0);
  digitalWrite(motor_B_enable, 0);
  analogWrite(motor_A, LOW);
  analogWrite(motor_B, LOW);
}

void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr2 = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    if(incomingStr2[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr2[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr2[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr2[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr2[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }

  if(Serial.available()){
    String incomingStr = Serial.readStringUntil('\n');
    if(incomingStr[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }

  // 100ms 지연
  delay(100);
}
