/*
하드웨어 연결 
arduino uno     ESP32 Devkit V1

GND             GND 
7               TX2(17)
8               RX2(16)

*/

#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (예: RX = 10, TX = 11)
SoftwareSerial mySerial(7, 8); // RX, TX

void setup() {
  // 기본 하드웨어 시리얼 포트 (USB 시리얼 모니터)
  Serial.begin(115200);

  // 소프트웨어 시리얼 포트
  mySerial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Serial.println("Software Serial에서 데이터 수신 준비 완료");
}

int linear = 0, angle = 0;
void loop() {
  // SoftwareSerial에서 데이터가 있을 경우 처리
  if (mySerial.available()) {
    // 수신된 데이터 읽기
    String inString = mySerial.readStringUntil('\n');  // '\n' 문자를 만날 때까지 읽기
    linear = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
    angle = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();
         
    
    // 읽은 데이터를 시리얼 모니터에 출력
    //Serial.print("Software Serial에서 받은 데이터: ");
    // 'a'기 압력되면 13 LED on
    //if(incomingData[0] == 'a')
    //  digitalWrite(13, HIGH);
    // 'b'가 입력되면 13 LED off
    //else if(incomingData[0] == 'b')
    //  digitalWrite(13, LOW);
    Serial.print(linear);
    Serial.print("  ");
    Serial.println(angle);
    digitalWrite(5, LOW);
    if(linear == 0)
      digitalWrite(6, LOW);
    else
      analogWrite(6, linear);

  }
}
