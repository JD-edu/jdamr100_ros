/*
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
*/

#include <WiFi.h>

// Wi-Fi 설정
const char* ssid = "SK_WiFiGIGA73BA_2.4G";       // Wi-Fi 이름
const char* password = "1609043407";  // Wi-Fi 비밀번호

#define RXD2 16   // 기본 RX2 핀 (GPIO16)
#define TXD2 17   // 기본 TX2 핀 (GPIO17)



// 서버 포트 설정
WiFiServer server(8080);

void setup() {
  Serial.begin(115200);  // 시리얼 모니터 시작
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  
  // Wi-Fi 연결
  Serial.println("Wi-Fi 연결 중...");
  WiFi.begin(ssid, password);

  // Wi-Fi 연결이 완료될 때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("연결 중...");
  }

  Serial.println("Wi-Fi 연결 완료!");
  Serial.print("ESP32 IP 주소: ");
  Serial.println(WiFi.localIP());

  // TCP 서버 시작
  server.begin();
  Serial.println("서버 시작됨, 클라이언트 대기 중...");

  pinMode(2, OUTPUT);
}

int linear = 0, angle = 0;
void loop() {
  // 클라이언트가 연결되면 처리
  WiFiClient client = server.available();

  if (client) {
    Serial.println("클라이언트가 연결되었습니다.");

    while (client.connected()) {
      if (client.available()) {
        // 클라이언트로부터 데이터 수신
        String message = client.readStringUntil('\n');
        Serial.println(message);
        Serial2.println(message);

        // 클라이언트에 응답 보내기
        client.println("데이터 수신 완료");
      }
    }

    // 클라이언트 연결 종료
    client.stop();
    Serial.println("클라이언트 연결 종료.");
  }
}
