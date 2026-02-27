import cv2
import socket
import struct
import time # 시계 도구 (타이머용)
import board # 핀 번호 도구
import adafruit_dht # 파란색 습도 센서 통역사

# 1. 습도 센서 개통 (17번 핀)
# 라즈베리파이 5의 버그를 피하기 위해 use_pulseio=False 를 꼭 넣어줍니다.
dhtDevice = adafruit_dht.DHT11(board.D17, use_pulseio=False)

# 2. A에게 전화 걸기
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ip = '10.42.0.1' # A 라즈베리파이의 IP 주소 (핫스팟 주소)
port = 8080
client_socket.connect((server_ip, port))

# 3. 카메라 켜기
cap = cv2.VideoCapture(0)
print("B(클라이언트): 🎥 조용히 감시를 시작합니다. (습도 45% 이상일 때만 A에게 알림)")

# 스톱워치를 누르고 현재 시간을 기억해둡니다.
last_check_time = time.time()
is_emergency = False # 지금 비상 상황인가요? (처음엔 False)

while True:
    ret, frame = cap.read()
    if not ret: break
    
    current_time = time.time() # 지금 몇 시인지 시계를 봅니다.
    
    # 4. 2초마다 한 번씩 몰래 습도 확인하기
    if current_time - last_check_time >= 2.0:
        last_check_time = current_time # 다음 2초를 위해 스톱워치를 다시 누릅니다.
        
        try:
            humidity = dhtDevice.humidity # 센서에게 습도 값을 물어봅니다.
            
            if humidity is not None:
                # 💡 오직 습도가 45 이상일 때만 반응합니다!
                if humidity >= 45:
                    if not is_emergency: # 처음 45%를 넘었을 때 딱 한 번만 화면에 경고를 띄웁니다.
                        print("🚨 [비상] 습도 45% 돌파! A서버 송장에 빨간 스티커를 붙입니다!")
                    is_emergency = True # A에게 보낼 송장에 "비상(True)!!" 표시를 합니다.
                else:
                    is_emergency = False # 습도가 45 미만이면 다시 평화 상태(False)로 돌립니다.
        except Exception as e:
            # 센서가 가끔 값을 못 읽어도, 카메라 영상은 끊기면 안 되므로 쿨하게 무시하고 넘어갑니다.
            pass 
    
    # 5. 영상 압축하기 (데이터 다이어트)
    frame = cv2.resize(frame, (320, 240))
    result, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    data = encoded_frame.tobytes()
    
    # 6. 택배 보내기
    # 9바이트짜리 특수 송장에 (사진 용량, 비상상황True/False) 두 가지를 함께 포장합니다.
    header = struct.pack("<Q?", len(data), is_emergency)
    
    # A에게 송장(header)을 먼저 보내고, 이어서 사진(data)을 와다다다 보냅니다.
    client_socket.sendall(header)
    client_socket.sendall(data)

# 뒷정리
cap.release()
client_socket.close()
dhtDevice.exit() # 센서도 깔끔하게 끕니다.
