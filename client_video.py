import cv2
import json
import time
import socket
import struct
import threading
import re

import serial

# -----------------------------
# [1] 설정
# -----------------------------
SERVER_IP = "10.42.0.1"   # 서버용 라즈베리파이 IP
SERVER_PORT = 8080

CAM_INDEX = 0
FRAME_W, FRAME_H = 320, 240
JPEG_QUALITY = 80

# STM32 -> Raspberry Pi (UART)
SERIAL_PORT = "/dev/ttyUSB0"  # 환경에 따라 /dev/serial0, /dev/ttyAMA0 등으로 변경
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# STM32에서 "🚨 추락감지 🚨" 수신 시 이 시간 동안 emergency=True 전송
EMERGENCY_SIGNAL_HOLD_SEC = 1.5

# header: (uint64 frame_size, bool is_emergency, uint32 sensor_json_size)
HEADER_FMT = "<Q?I"


# -----------------------------
# [2] UART 데이터 공유 상태
# -----------------------------
sensor_lock = threading.Lock()
latest_sensor = {
    "source": "stm32",
    "accel_raw": None,
    "gyro_raw": None,
    "svm": None,
    "message": None,
    "line_ts": None,
}
emergency_until = 0.0


def parse_stm32_line(line: str):
    """STM32에서 온 한 줄 문자열을 파싱해서 센서 상태를 업데이트한다."""
    global emergency_until

    now = time.time()

    with sensor_lock:
        latest_sensor["message"] = line
        latest_sensor["line_ts"] = now

        if "추락감지" in line:
            emergency_until = max(emergency_until, now + EMERGENCY_SIGNAL_HOLD_SEC)
            return

        # 예: Acc: 123, -45, 16000 | Gyro: 12, -5, 30
        m = re.search(
            r"Acc:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\|\s*Gyro:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)",
            line,
        )
        if m:
            ax, ay, az, gx, gy, gz = map(int, m.groups())
            latest_sensor["accel_raw"] = {"x": ax, "y": ay, "z": az}
            latest_sensor["gyro_raw"] = {"x": gx, "y": gy, "z": gz}

            # STM32 코드와 동일한 계산식(svm = sqrt((ax/16384)^2 + ...))
            f_ax, f_ay, f_az = ax / 16384.0, ay / 16384.0, az / 16384.0
            latest_sensor["svm"] = (f_ax * f_ax + f_ay * f_ay + f_az * f_az) ** 0.5


# -----------------------------
# [3] UART 수신 스레드
# -----------------------------
def stm32_reader_thread():
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            print(f"[CLIENT] STM32 UART 연결됨: {SERIAL_PORT} @ {SERIAL_BAUD}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                parse_stm32_line(line)

        except Exception as e:
            print(f"[CLIENT] STM32 UART 오류: {e} (1초 후 재시도)")
            time.sleep(1)


# -----------------------------
# [4] 서버 연결 유틸
# -----------------------------
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


# -----------------------------
# [5] 메인: 영상 + 센서데이터 전송
# -----------------------------
def main():
    threading.Thread(target=stm32_reader_thread, daemon=True).start()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[CLIENT] 카메라 열기 실패")
        return

    sock = connect_server()

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
            frame_bytes = encoded.tobytes()

            now = time.time()
            with sensor_lock:
                is_emergency = now < emergency_until
                sensor_payload = {
                    "source": "stm32",
                    "is_emergency_by_stm32": is_emergency,
                    "accel_raw": latest_sensor["accel_raw"],
                    "gyro_raw": latest_sensor["gyro_raw"],
                    "svm": latest_sensor["svm"],
                    "message": latest_sensor["message"],
                    "line_ts": latest_sensor["line_ts"],
                    "send_ts": now,
                }

            sensor_bytes = json.dumps(sensor_payload, ensure_ascii=False).encode("utf-8")
            header = struct.pack(HEADER_FMT, len(frame_bytes), is_emergency, len(sensor_bytes))

            try:
                sock.sendall(header)
                sock.sendall(sensor_bytes)
                sock.sendall(frame_bytes)
            except Exception:
                print("[CLIENT] 서버 송신 실패 -> 재연결")
                try:
                    sock.close()
                except Exception:
                    pass
                sock = connect_server()

    finally:
        cap.release()
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
