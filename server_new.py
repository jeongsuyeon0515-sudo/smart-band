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

OUT_DIR = "/home/pi/events"
os.makedirs(OUT_DIR, exist_ok=True)
SENSOR_LOG_PATH = os.path.join(OUT_DIR, "sensor_data.log")

DEFAULT_FPS = 30
TARGET_W, TARGET_H = 640, 480
USE_MP4 = True

# 새 클라이언트(클립 전송) 메시지 타입
MSG_CLIP_START = 1
MSG_FRAME = 2
MSG_CLIP_END = 3

# 구형/스트리밍 클라이언트 헤더: (uint64 frame_size, bool is_emergency, uint32 sensor_json_size)
LEGACY_HEADER_FMT = "<Q?I"
LEGACY_HEADER_SIZE = struct.calcsize(LEGACY_HEADER_FMT)
LEGACY_SECONDS_BEFORE = 10
LEGACY_SECONDS_AFTER = 10
LEGACY_MAX_FRAMES_BEFORE = DEFAULT_FPS * LEGACY_SECONDS_BEFORE
LEGACY_MAX_FRAMES_AFTER = DEFAULT_FPS * LEGACY_SECONDS_AFTER

sensor_log_lock = threading.Lock()
play_queue: "queue.Queue[str]" = queue.Queue()


# -----------------------------
# [2] 공용 유틸
# -----------------------------
def recv_exact(sock: socket.socket, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return b""
        data.extend(chunk)
    return bytes(data)


def play_video(path: str):
    os.environ.setdefault("DISPLAY", ":0")
    subprocess.run(["cvlc", "--play-and-exit", "--fullscreen", path], check=False)


def player_worker():
    while True:
        path = play_queue.get()
        try:
            play_video(path)
        except Exception as e:
            print(f"[PLAYER] 자동 재생 실패: {e}")
        finally:
            play_queue.task_done()


def append_sensor_log(client_id: str, payload: dict):
    """센서 필드 구조를 가정하지 않고 payload 전체를 로그로 남긴다."""
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"{ts}\t{client_id}\t{json.dumps(payload, ensure_ascii=False)}\n"
    with sensor_log_lock:
        with open(SENSOR_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)


def build_output_path(client_id: str) -> str:
    ts = time.strftime("%Y%m%d_%H%M%S")
    ext = "mp4" if USE_MP4 else "avi"
    return os.path.join(OUT_DIR, f"emergency_{client_id}_{ts}.{ext}")


def save_clip(path: str, fps: int, frames: list):
    fourcc = cv2.VideoWriter_fourcc(*("mp4v" if USE_MP4 else "XVID"))
    out = cv2.VideoWriter(path, fourcc, fps, (TARGET_W, TARGET_H))
    for frame in frames:
        out.write(frame)
    out.release()


def decode_jpeg_to_frame(jpg_bytes: bytes):
    frame = cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    if (frame.shape[1], frame.shape[0]) != (TARGET_W, TARGET_H):
        frame = cv2.resize(frame, (TARGET_W, TARGET_H))
    return frame


@dataclass
class LegacyState:
    before_frames: deque = field(default_factory=lambda: deque(maxlen=LEGACY_MAX_FRAMES_BEFORE))
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
# [3] 새 프로토콜(클립 업로드) 핸들러
# -----------------------------
def handle_clip_protocol(client_socket: socket.socket, client_id: str, first_byte: bytes):
    clip_meta = None
    clip_frames = []

    msg_type_raw = first_byte
    while True:
        if not msg_type_raw:
            print(f"[{client_id}] 연결 종료")
            break

        msg_type = struct.unpack("<B", msg_type_raw)[0]

        if msg_type == MSG_CLIP_START:
            size_raw = recv_exact(client_socket, 4)
            if not size_raw:
                break
            meta_size = struct.unpack("<I", size_raw)[0]
            meta_raw = recv_exact(client_socket, meta_size)
            if not meta_raw:
                break

            try:
                clip_meta = json.loads(meta_raw.decode("utf-8"))
            except Exception:
                clip_meta = {"raw": meta_raw.decode("utf-8", errors="replace")}

            clip_frames = []
            append_sensor_log(client_id, {"protocol": "clip", "meta": clip_meta})
            print(f"[{client_id}] [clip] 이벤트 시작 / target={clip_meta.get('total_frames')}")

        elif msg_type == MSG_FRAME:
            size_raw = recv_exact(client_socket, 4)
            if not size_raw:
                break
            jpg_size = struct.unpack("<I", size_raw)[0]
            jpg = recv_exact(client_socket, jpg_size)
            if not jpg:
                break

            frame = decode_jpeg_to_frame(jpg)
            if frame is not None:
                clip_frames.append(frame)

        elif msg_type == MSG_CLIP_END:
            if not clip_frames:
                print(f"[{client_id}] [clip] clip_end 수신했지만 프레임 없음")
            else:
                fps = int((clip_meta or {}).get("fps", DEFAULT_FPS))
                out_path = build_output_path(client_id)
                print(f"[{client_id}] [clip] 저장 중... {out_path} (frames={len(clip_frames)})")
                save_clip(out_path, fps, clip_frames)
                print(f"[{client_id}] [clip] ✅ 저장 완료")
                play_queue.put(out_path)

            clip_meta = None
            clip_frames = []

        else:
            print(f"[{client_id}] [clip] 알 수 없는 메시지 타입: {msg_type}")
            break

        msg_type_raw = recv_exact(client_socket, 1)


# -----------------------------
# [4] 구형 프로토콜(<Q?I per-frame) 핸들러
# -----------------------------
def handle_legacy_protocol(client_socket: socket.socket, client_id: str, initial_bytes: bytes):
    state = LegacyState()
    pending = bytearray(initial_bytes)

    def read_legacy_header():
        while len(pending) < LEGACY_HEADER_SIZE:
            chunk = client_socket.recv(LEGACY_HEADER_SIZE - len(pending))
            if not chunk:
                return b""
            pending.extend(chunk)
        header = bytes(pending[:LEGACY_HEADER_SIZE])
        del pending[:LEGACY_HEADER_SIZE]
        return header

    while True:
        header = read_legacy_header()
        if not header:
            print(f"[{client_id}] [legacy] 연결 종료")
            break

        msg_size, is_emergency, sensor_size = struct.unpack(LEGACY_HEADER_FMT, header)

        sensor_data = {}
        if sensor_size > 0:
            sensor_raw = recv_exact(client_socket, sensor_size)
            if not sensor_raw:
                print(f"[{client_id}] [legacy] 센서데이터 수신 중 종료")
                break
            try:
                sensor_data = json.loads(sensor_raw.decode("utf-8"))
            except Exception:
                sensor_data = {"raw": sensor_raw.decode("utf-8", errors="replace")}

        append_sensor_log(
            client_id,
            {
                "protocol": "legacy",
                "is_emergency": bool(is_emergency),
                "sensor": sensor_data,
            },
        )

        frame_data = recv_exact(client_socket, msg_size)
        if not frame_data:
            print(f"[{client_id}] [legacy] 프레임 수신 중 종료")
            break

        frame = decode_jpeg_to_frame(frame_data)
        if frame is None:
            continue

        if not is_emergency:
            state.wait_for_reset = False

        if is_emergency and (not state.recording) and (not state.wait_for_reset):
            state.recording = True
            state.after_frames.clear()
            state.after_count = 0
            print(f"[{client_id}] [legacy] 🚨 비상 감지: 10초 전/후 클립 생성")

        if not state.recording:
            state.before_frames.append(frame)
            continue

        state.after_frames.append(frame)
        state.after_count += 1

        if state.after_count >= LEGACY_MAX_FRAMES_AFTER:
            out_path = build_output_path(client_id)
            print(f"[{client_id}] [legacy] 저장 중... {out_path}")
            save_clip(out_path, DEFAULT_FPS, list(state.before_frames) + state.after_frames)
            print(f"[{client_id}] [legacy] ✅ 저장 완료")
            play_queue.put(out_path)
            state.reset_after_save()


# -----------------------------
# [5] 클라이언트 처리(프로토콜 자동 판별)
# -----------------------------
def handle_client(client_socket: socket.socket, addr):
    client_id = f"{addr[0].replace('.', '_')}_{addr[1]}"
    print(f"[{client_id}] 연결됨")

    try:
        # 첫 1바이트를 보고 새/구형 프로토콜 선택
        first = recv_exact(client_socket, 1)
        if not first:
            print(f"[{client_id}] 연결 직후 종료")
            return

        first_val = first[0]
        if first_val in (MSG_CLIP_START, MSG_FRAME, MSG_CLIP_END):
            print(f"[{client_id}] 프로토콜: clip")
            handle_clip_protocol(client_socket, client_id, first)
        else:
            print(f"[{client_id}] 프로토콜: legacy")
            handle_legacy_protocol(client_socket, client_id, first)

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
