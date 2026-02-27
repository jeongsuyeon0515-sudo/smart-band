import cv2
import json
import time
import socket
import struct
import threading
import re
from collections import deque

import serial

# -----------------------------
# [1] 설정
# -----------------------------
SERVER_IP = "10.42.0.1"
SERVER_PORT = 8080

SERIAL_PORT = "/dev/ttyUSB0"  # 블루투스 시리얼 포트에 맞게 변경
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

CAM_INDEX = 0
FPS = 10
SECONDS_BEFORE = 10
SECONDS_AFTER = 10
MAX_FRAMES_BEFORE = FPS * SECONDS_BEFORE
MAX_FRAMES_AFTER = FPS * SECONDS_AFTER

FRAME_W, FRAME_H = 640, 480
JPEG_QUALITY = 85

# 메시지 타입
MSG_CLIP_START = 1
MSG_FRAME = 2
MSG_CLIP_END = 3


# -----------------------------
# [2] 공유 상태 (STM32 센서)
# -----------------------------
sensor_lock = threading.Lock()
last_sensor = {
    "source": "stm32",
    "message": None,
    "accel_raw": None,
    "gyro_raw": None,
    "svm": None,
    "ts": None,
}

# 낙하 감지 이벤트 큐(중복 트리거 방지용 timestamp)
pending_event = threading.Event()


def parse_stm32_line(line: str):
    now = time.time()

    with sensor_lock:
        last_sensor["message"] = line
        last_sensor["ts"] = now

        m = re.search(
            r"Acc:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\|\s*Gyro:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)",
            line,
        )
        if m:
            ax, ay, az, gx, gy, gz = map(int, m.groups())
            last_sensor["accel_raw"] = {"x": ax, "y": ay, "z": az}
            last_sensor["gyro_raw"] = {"x": gx, "y": gy, "z": gz}
            fx, fy, fz = ax / 16384.0, ay / 16384.0, az / 16384.0
            last_sensor["svm"] = (fx * fx + fy * fy + fz * fz) ** 0.5

        # STM32 코드에서 보내는 비상 문자열
        if "추락감지" in line:
            pending_event.set()
            print(f"[CLIENT] 🚨 낙하감지 수신: {line}")


def stm32_reader_thread():
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            print(f"[CLIENT] STM32 시리얼 연결됨: {SERIAL_PORT} @ {SERIAL_BAUD}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                parse_stm32_line(line)

        except Exception as e:
            print(f"[CLIENT] STM32 연결 오류: {e} (1초 후 재시도)")
            time.sleep(1)


def connect_server() -> socket.socket:
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((SERVER_IP, SERVER_PORT))
            print(f"[CLIENT] 서버 연결됨: {SERVER_IP}:{SERVER_PORT}")
            return sock
        except Exception as e:
            print(f"[CLIENT] 서버 연결 실패: {e} (1초 후 재시도)")
            time.sleep(1)


def send_clip(sock: socket.socket, frames: list[bytes], sensor_snapshot: dict):
    meta = {
        "event": "fall_detected",
        "fps": FPS,
        "width": FRAME_W,
        "height": FRAME_H,
        "seconds_before": SECONDS_BEFORE,
        "seconds_after": SECONDS_AFTER,
        "total_frames": len(frames),
        "sensor": sensor_snapshot,
        "client_ts": time.time(),
    }
    meta_bytes = json.dumps(meta, ensure_ascii=False).encode("utf-8")

    sock.sendall(struct.pack("<BI", MSG_CLIP_START, len(meta_bytes)))
    sock.sendall(meta_bytes)

    for jpg in frames:
        sock.sendall(struct.pack("<BI", MSG_FRAME, len(jpg)))
        sock.sendall(jpg)

    sock.sendall(struct.pack("<B", MSG_CLIP_END))


def main():
    threading.Thread(target=stm32_reader_thread, daemon=True).start()

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("[CLIENT] 카메라 열기 실패")
        return

    sock = connect_server()

    before_buffer = deque(maxlen=MAX_FRAMES_BEFORE)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[CLIENT] 카메라 프레임 읽기 실패")
                time.sleep(0.1)
                continue

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            )
            if not ok:
                continue

            jpg = encoded.tobytes()
            before_buffer.append(jpg)

            # 이벤트 발생 시점: 10초 전 버퍼 + 10초 후 수집 후 전송
            if pending_event.is_set():
                pending_event.clear()

                with sensor_lock:
                    sensor_snapshot = dict(last_sensor)

                print("[CLIENT] 이벤트 처리 시작: 10초 전 + 10초 후 클립 생성")
                clip_frames = list(before_buffer)

                after_collected = 0
                while after_collected < MAX_FRAMES_AFTER:
                    ret2, frame2 = cap.read()
                    if not ret2:
                        time.sleep(0.05)
                        continue

                    frame2 = cv2.resize(frame2, (FRAME_W, FRAME_H))
                    ok2, encoded2 = cv2.imencode(
                        ".jpg", frame2, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                    )
                    if not ok2:
                        continue

                    jpg2 = encoded2.tobytes()
                    clip_frames.append(jpg2)
                    before_buffer.append(jpg2)
                    after_collected += 1

                # 클립 전송 (실패시 재연결 후 재시도 1회)
                sent = False
                for _ in range(2):
                    try:
                        send_clip(sock, clip_frames, sensor_snapshot)
                        sent = True
                        break
                    except Exception as e:
                        print(f"[CLIENT] 클립 전송 실패: {e} -> 재연결")
                        try:
                            sock.close()
                        except Exception:
                            pass
                        sock = connect_server()

                if sent:
                    print(f"[CLIENT] ✅ 클립 전송 완료 (frames={len(clip_frames)})")
                else:
                    print("[CLIENT] ❌ 클립 전송 최종 실패")

            # 너무 빠른 루프 방지
            time.sleep(1.0 / FPS)

    finally:
        cap.release()
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
