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
