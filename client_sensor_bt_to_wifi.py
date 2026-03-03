import json
import socket
import threading
import time
from typing import Optional

import serial

# -----------------------------
# [1] 설정 (헬멧 라즈베리파이)
# -----------------------------
SERVER_HOST = "10.42.0.1"   # 서버 라즈베리파이 IP
SERVER_PORT = 9090

SERIAL_PORT = "/dev/rfcomm0"  # STM32 블루투스 시리얼 포트
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.2

DEVICE_ID = "helmet-rpi-01"
RETRY_DELAY_SEC = 1.0
SOCKET_TIMEOUT_SEC = 5.0

send_lock = threading.Lock()


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def connect_server() -> socket.socket:
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(SOCKET_TIMEOUT_SEC)
            sock.connect((SERVER_HOST, SERVER_PORT))
            sock.settimeout(None)
            print(f"[CLIENT] 서버 연결 성공: {SERVER_HOST}:{SERVER_PORT}")
            return sock
        except Exception as exc:
            print(f"[CLIENT] 서버 연결 실패: {exc} (재시도 {RETRY_DELAY_SEC}s)")
            time.sleep(RETRY_DELAY_SEC)


def open_serial() -> serial.Serial:
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            print(f"[CLIENT] STM32 블루투스 시리얼 연결: {SERIAL_PORT} @ {SERIAL_BAUD}")
            return ser
        except Exception as exc:
            print(f"[CLIENT] 시리얼 연결 실패: {exc} (재시도 {RETRY_DELAY_SEC}s)")
            time.sleep(RETRY_DELAY_SEC)


def parse_sensor_line(raw_line: str) -> dict:
    """
    STM32에서 받은 한 줄을 서버 전송용 JSON으로 감싼다.
    - 권장 STM32 포맷 예: "MAG,-12,34,56"
    - 포맷이 달라도 raw 필드에 보존된다.
    """
    msg = {
        "device_id": DEVICE_ID,
        "source": "stm32_bluetooth",
        "sensor_type": "magnetometer",
        "ts_client": now_iso(),
        "raw": raw_line,
    }

    parts = [p.strip() for p in raw_line.split(",")]
    if len(parts) == 4 and parts[0].upper() == "MAG":
        try:
            mx, my, mz = int(parts[1]), int(parts[2]), int(parts[3])
            msg["mag"] = {"x": mx, "y": my, "z": mz}
            msg["format"] = "MAG,x,y,z"
        except ValueError:
            msg["parse_error"] = "invalid MAG numeric values"
    else:
        msg["format"] = "unknown"

    return msg


def send_json_line(sock: socket.socket, payload: dict) -> None:
    data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    with send_lock:
        sock.sendall(data)


def main() -> None:
    ser: Optional[serial.Serial] = None
    sock: Optional[socket.socket] = None
    seq = 0

    try:
        ser = open_serial()
        sock = connect_server()

        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            payload = parse_sensor_line(line)
            payload["seq"] = seq
            seq += 1

            # 소켓 에러 시 재연결 후 1회 재전송
            try:
                send_json_line(sock, payload)
                print(f"[CLIENT] 전송 완료 seq={payload['seq']} raw={line}")
            except Exception as exc:
                print(f"[CLIENT] 전송 실패: {exc} -> 재연결")
                try:
                    sock.close()
                except Exception:
                    pass
                sock = connect_server()
                send_json_line(sock, payload)
                print(f"[CLIENT] 재전송 완료 seq={payload['seq']}")

    except KeyboardInterrupt:
        print("\n[CLIENT] 종료")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
