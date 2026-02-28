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

# -----------------------------
# [1] 설정
# -----------------------------
HOST = "0.0.0.0"
PORT = 8080

OUT_DIR = "/home/pi/events"
os.makedirs(OUT_DIR, exist_ok=True)
SENSOR_LOG_PATH = os.path.join(OUT_DIR, "sensor_data.log")

DEFAULT_FPS = 10
TARGET_W, TARGET_H = 640, 480
USE_MP4 = True

# 메시지 타입
MSG_CLIP_START = 1
MSG_FRAME = 2
MSG_CLIP_END = 3

sensor_log_lock = threading.Lock()
play_queue: "queue.Queue[str]" = queue.Queue()


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


def append_sensor_log(client_id: str, meta: dict):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"{ts}\t{client_id}\t{json.dumps(meta, ensure_ascii=False)}\n"
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


def handle_client(client_socket: socket.socket, addr):
    client_id = f"{addr[0].replace('.', '_')}_{addr[1]}"
    print(f"[{client_id}] 연결됨")

    clip_meta = None
    clip_frames = []

    try:
        while True:
            msg_type_raw = recv_exact(client_socket, 1)
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
                append_sensor_log(client_id, clip_meta)
                print(f"[{client_id}] 이벤트 수신 시작: {clip_meta.get('event')} / target={clip_meta.get('total_frames')}")

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
                    print(f"[{client_id}] clip_end 수신했지만 프레임 없음")
                    continue

                fps = int((clip_meta or {}).get("fps", DEFAULT_FPS))
                out_path = build_output_path(client_id)

                print(f"[{client_id}] 저장 중... {out_path} (frames={len(clip_frames)})")
                save_clip(out_path, fps, clip_frames)
                print(f"[{client_id}] ✅ 저장 완료")

                play_queue.put(out_path)

                clip_meta = None
                clip_frames = []

            else:
                print(f"[{client_id}] 알 수 없는 메시지 타입: {msg_type}")
                break

    except Exception as e:
        print(f"[{client_id}] 에러: {e}")
    finally:
        client_socket.close()
        print(f"[{client_id}] 스레드 종료")


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
