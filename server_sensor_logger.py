import json
import os
import socket
import threading
import time
from typing import Optional

# -----------------------------
# [1] 설정 (서버 라즈베리파이)
# -----------------------------
HOST = "0.0.0.0"
PORT = 9090

LOG_DIR = "./sensor_logs"
os.makedirs(LOG_DIR, exist_ok=True)
LOG_PATH = os.path.join(LOG_DIR, "mag_sensor_data.jsonl")

log_lock = threading.Lock()


def append_log(record: dict) -> None:
    with log_lock:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")


def recv_lines(conn: socket.socket):
    buffer = b""
    while True:
        chunk = conn.recv(4096)
        if not chunk:
            break
        buffer += chunk
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            yield line


def handle_client(conn: socket.socket, addr) -> None:
    client_id = f"{addr[0]}:{addr[1]}"
    print(f"[SERVER] 연결됨: {client_id}")

    try:
        for line in recv_lines(conn):
            if not line:
                continue

            try:
                payload = json.loads(line.decode("utf-8"))
            except Exception:
                payload = {
                    "decode_error": True,
                    "raw": line.decode("utf-8", errors="replace"),
                }

            record = {
                "ts_server": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
                "client_addr": client_id,
                "payload": payload,
            }
            append_log(record)
            seq: Optional[int] = payload.get("seq") if isinstance(payload, dict) else None
            print(f"[SERVER] 수신/저장 완료 client={client_id} seq={seq}")

    except Exception as exc:
        print(f"[SERVER] 클라이언트 처리 오류({client_id}): {exc}")
    finally:
        conn.close()
        print(f"[SERVER] 연결 종료: {client_id}")


def main() -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(16)

    print(f"[SERVER] 대기 중: {HOST}:{PORT}")
    print(f"[SERVER] 로그 파일: {LOG_PATH}")

    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n[SERVER] 종료")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
