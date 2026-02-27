# server_video_multi.py
import cv2
import socket
import struct
import numpy as np
from collections import deque
import subprocess
import os
import time
import threading

# -----------------------------
# [1] 설정
# -----------------------------
FPS = 10
SECONDS_BEFORE = 10
SECONDS_AFTER = 10
MAX_FRAMES_BEFORE = FPS * SECONDS_BEFORE
MAX_FRAMES_AFTER  = FPS * SECONDS_AFTER

HOST = "0.0.0.0"
PORT = 8080

OUT_DIR = "/home/pi/events"
os.makedirs(OUT_DIR, exist_ok=True)

TARGET_W, TARGET_H = 640, 480
USE_MP4 = True

payload_size = struct.calcsize("<Q?")  # (uint64 size, bool emergency)

def play_video(path: str):
    # 로컬 GUI(HDMI)에서 재생
    os.environ.setdefault("DISPLAY", ":0")
    subprocess.Popen(["cvlc", "--play-and-exit", "--fullscreen", path])

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf

# -----------------------------
# [2] 클라이언트 처리 스레드
# -----------------------------
def handle_client(client_socket: socket.socket, addr):
    client_id = f"{addr[0].replace('.', '_')}_{addr[1]}"
    print(f"[{client_id}] 연결됨")

    before_frames = deque(maxlen=MAX_FRAMES_BEFORE)
    after_frames = []

    is_recording_emergency = False
    after_count = 0
    wait_for_reset = False

    try:
        while True:
            header = recv_exact(client_socket, payload_size)
            if not header:
                print(f"[{client_id}] 연결 종료")
                break

            msg_size, is_emergency_signal = struct.unpack("<Q?", header)

            frame_data = recv_exact(client_socket, msg_size)
            if not frame_data:
                print(f"[{client_id}] 프레임 수신 중 종료")
                break

            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            if (frame.shape[1], frame.shape[0]) != (TARGET_W, TARGET_H):
                frame = cv2.resize(frame, (TARGET_W, TARGET_H))

            # 방패 해제 (신호 False)
            if not is_emergency_signal:
                wait_for_reset = False

            # 비상 시작
            if is_emergency_signal and (not is_recording_emergency) and (not wait_for_reset):
                print(f"[{client_id}] 🚨 비상 감지! 앞/뒤 녹화 시작")
                is_recording_emergency = True
                after_frames.clear()
                after_count = 0

            # 버퍼 저장
            if not is_recording_emergency:
                before_frames.append(frame)
            else:
                after_frames.append(frame)
                after_count += 1

                if after_count >= MAX_FRAMES_AFTER:
                    ts = time.strftime("%Y%m%d_%H%M%S")

                    if USE_MP4:
                        out_name = f"emergency_{client_id}_{ts}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    else:
                        out_name = f"emergency_{client_id}_{ts}.avi"
                        fourcc = cv2.VideoWriter_fourcc(*"XVID")

                    out_path = os.path.join(OUT_DIR, out_name)
                    print(f"[{client_id}] 저장 중... {out_path}")

                    out = cv2.VideoWriter(out_path, fourcc, FPS, (TARGET_W, TARGET_H))
                    for f in before_frames:
                        out.write(f)
                    for f in after_frames:
                        out.write(f)
                    out.release()

                    print(f"[{client_id}] ✅ 저장 완료: {out_name}")

                    # 자동 재생(원하면 유지, 여러 클라이언트면 동시에 재생될 수 있음)
                    play_video(out_path)

                    # 초기화
                    is_recording_emergency = False
                    after_frames.clear()
                    after_count = 0
                    before_frames.clear()

                    wait_for_reset = True
                    print(f"[{client_id}] 연속 저장 차단(Reset 대기)")

    except Exception as e:
        print(f"[{client_id}] 에러: {e}")

    finally:
        client_socket.close()
        print(f"[{client_id}] 스레드 종료")

# -----------------------------
# [3] 서버 메인 (항상 accept)
# -----------------------------
def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(10)  # ✅ 여기! 여러 클라이언트 대기열
    print(f"A(서버): {HOST}:{PORT} 멀티클라이언트 대기 중...")

    try:
        while True:
            client_socket, addr = server_socket.accept()
            t = threading.Thread(target=handle_client, args=(client_socket, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\nA(서버): 종료합니다...")
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()
