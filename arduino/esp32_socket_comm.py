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

import socket
import time 

# ESP32 서버의 IP 주소 및 포트 번호 설정
server_ip = '172.30.1.90'  # ESP32의 IP 주소 (Wi-Fi 연결 시 확인)
port = 8080                  # ESP32에서 설정한 포트 번호

# 소켓 생성 (IPv4, TCP)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버에 연결
client_socket.connect((server_ip, port))
print(f"서버 {server_ip}:{port}에 연결 성공")


for i in range(4):
    # 메시지 전송
    message = 'a'
    client_socket.send((message + "\n").encode())  # 줄바꿈 문자 추가
    # 서버로부터의 응답 수신
    response = client_socket.recv(1024)
    print(f"서버 응답: {response.decode()}")
    time.sleep(1)

    message = 'b'
    client_socket.send((message + "\n").encode())  # 줄바꿈 문자 추가
    # 서버로부터의 응답 수신
    response = client_socket.recv(1024)
    print(f"서버 응답: {response.decode()}")
    time.sleep(1)


# 소켓 종료
client_socket.close()
