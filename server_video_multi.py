import os
import cv2
import json
import time
import socket
import struct
import subprocess
import threading
import queue
import numpy as np
from collections import deque
from dataclasses import dataclass, field

# -----------------------------
# [1] 설정
# -----------------------------
HOST = "0.0.0.0"
PORT = 8080

FPS = 10
SECONDS_BEFORE = 10
SECONDS_AFTER = 10
MAX_FRAMES_BEFORE = FPS * SECONDS_BEFORE
MAX_FRAMES_AFTER = FPS * SECONDS_AFTER

TARGET_W, TARGET_H = 640, 480
# header: (uint64 frame_size, bool is_emergency, uint32 sensor_json_size)
JPEG_HEADER_SIZE = struct.calcsize("<Q?I")

OUT_DIR = "/home/pi/events"
os.makedirs(OUT_DIR, exist_ok=True)
SENSOR_LOG_PATH = os.path.join(OUT_DIR, "sensor_data.log")

USE_MP4 = True

sensor_log_lock = threading.Lock()


# -----------------------------
# [2] 공용 유틸
# -----------------------------
def recv_exact(sock: socket.socket, n: int) -> bytes:
    """소켓에서 정확히 n바이트를 읽는다. 끊기면 b''를 반환."""
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return b""
        data.extend(chunk)
    return bytes(data)


def normalize_frame(frame):
    if frame is None:
        return None
    if (frame.shape[1], frame.shape[0]) != (TARGET_W, TARGET_H):
        frame = cv2.resize(frame, (TARGET_W, TARGET_H))
    return frame


def build_output_path(client_id: str) -> str:
    ts = time.strftime("%Y%m%d_%H%M%S")
    ext = "mp4" if USE_MP4 else "avi"
    return os.path.join(OUT_DIR, f"emergency_{client_id}_{ts}.{ext}")


def save_clip(path: str, before_frames, after_frames):
    if USE_MP4:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    else:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")

    out = cv2.VideoWriter(path, fourcc, FPS, (TARGET_W, TARGET_H))
    for frame in before_frames:
        out.write(frame)
    for frame in after_frames:
        out.write(frame)
    out.release()


def play_video(path: str):
    # 로컬 GUI(HDMI)에서 자동 재생
    # worker가 순차 실행되도록 재생이 끝날 때까지 대기한다.
    os.environ.setdefault("DISPLAY", ":0")
    subprocess.run(["cvlc", "--play-and-exit", "--fullscreen", path], check=False)


def append_sensor_log(client_id: str, is_emergency: bool, sensor_data: dict):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"{ts}\t{client_id}\temergency={is_emergency}\t{json.dumps(sensor_data, ensure_ascii=False)}\n"
    with sensor_log_lock:
        with open(SENSOR_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)


# -----------------------------
# [3] 재생 워커 (동시 재생 난립 방지)
# -----------------------------
play_queue: "queue.Queue[str]" = queue.Queue()


def player_worker():
    while True:
        path = play_queue.get()
        try:
            play_video(path)
        except Exception as e:
            print(f"[PLAYER] 자동 재생 실패: {e}")
        finally:
            play_queue.task_done()


# -----------------------------
# [4] 클라이언트 상태
# -----------------------------
@dataclass
class ClientState:
    before_frames: deque = field(default_factory=lambda: deque(maxlen=MAX_FRAMES_BEFORE))
    after_frames: list = field(default_factory=list)
    recording: bool = False
    after_count: int = 0
    wait_for_reset: bool = False

    def reset_after_save(self):
        self.before_frames.clear()
        self.after_frames.clear()
        self.recording = False
        self.after_count = 0
        self.wait_for_reset = True


# -----------------------------
# [5] 클라이언트 처리
# -----------------------------
def handle_client(client_socket: socket.socket, addr):
    client_id = f"{addr[0].replace('.', '_')}_{addr[1]}"
    state = ClientState()

    print(f"[{client_id}] 연결됨")

    try:
        while True:
            header = recv_exact(client_socket, JPEG_HEADER_SIZE)
            if not header:
                print(f"[{client_id}] 연결 종료")
                break

            msg_size, is_emergency, sensor_size = struct.unpack("<Q?I", header)

            sensor_data = {}
            if sensor_size > 0:
                sensor_raw = recv_exact(client_socket, sensor_size)
                if not sensor_raw:
                    print(f"[{client_id}] 센서데이터 수신 중 연 종료")
                    break
                try:
                    sensor_data = json.loads(sensor_raw.decode("utf-8"))
                except Exception:
                    sensor_data = {"raw": sensor_raw.decode("utf-8", errors="replace")}

            append_sensor_log(client_id, is_emergency, sensor_data)

            frame_data = recv_exact(client_socket, msg_size)
            if not frame_data:
                print(f"[{client_id}] 프레임 수신 중 연결 종료")
                break

            frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
            frame = normalize_frame(frame)
            if frame is None:
                continue

            # 신호가 False로 내려가야 다음 이벤트를 다시 트리거할 수 있음
            if not is_emergency:
                state.wait_for_reset = False

            # 비상 시작: before 버퍼는 이미 쌓여 있음
            if is_emergency and (not state.recording) and (not state.wait_for_reset):
                state.recording = True
                state.after_frames.clear()
                state.after_count = 0
                print(f"[{client_id}] 🚨 비상 감지: 10초 전/후 영상 저장 시작, sensor={sensor_data}")

            if not state.recording:
                state.before_frames.append(frame)
                continue

            state.after_frames.append(frame)
            state.after_count += 1

            if state.after_count >= MAX_FRAMES_AFTER:
                out_path = build_output_path(client_id)
                print(f"[{client_id}] 저장 중... {out_path}")
                save_clip(out_path, state.before_frames, state.after_frames)
                print(f"[{client_id}] ✅ 저장 완료")

                play_queue.put(out_path)
                state.reset_after_save()
                print(f"[{client_id}] 다음 이벤트 대기(Reset 필요)")

    except Exception as e:
        print(f"[{client_id}] 에러: {e}")
    finally:
        client_socket.close()
        print(f"[{client_id}] 스레드 종료")


# -----------------------------
# [6] 서버 메인
# -----------------------------
def main():
    threading.Thread(target=player_worker, daemon=True).start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(32)

    print(f"[SERVER] {HOST}:{PORT} 멀티클라이언트 대기 중")
    print(f"[SERVER] 센서 로그 파일: {SENSOR_LOG_PATH}")

    try:
        while True:
            client_socket, addr = server_socket.accept()
            t = threading.Thread(target=handle_client, args=(client_socket, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n[SERVER] 종료")
    finally:
        server_socket.close()


if __name__ == "__main__":
    main()
