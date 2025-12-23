import json
import time
import threading
import serial
from collections import deque
from queue import Queue, Empty, Full

import numpy as np
import joblib
from keras.models import load_model
from websocket_server import WebsocketServer

# ===============================
# CONFIG
# ===============================
SERIAL_PORT = "COM9"
BAUDRATE = 115200
WS_PORT = 3001

SENSOR_INTERVAL = 0.03  # 30ms (참고용)
SEQ_SIZE = 30

FEATURE_NAMES = ["AcX", "AcY", "AcZ", "GyX", "GyY", "GyZ"]

# ✅ anomaly 판단: 시간 기준 (1초)
ANOMALY_TIME_SEC = 1.0

MODEL_PATH = "LSTM_AutoEncoder_MPU6050.h5"
SCALER_PATH = "mpu6050_scaler.pkl"
THRESHOLD_PATH = "mpu6050_threshold.npy"

# 큐 적체 방지 (최신 우선)
SENSOR_QUEUE_MAX = 10
sensor_queue = Queue(maxsize=SENSOR_QUEUE_MAX)

ai_event_queue = Queue()

mpu_buffer = deque(maxlen=SEQ_SIZE)

latest_sensor = None
connected = False
seq = 0

anomaly_duration = 0.0
lock = threading.Lock()

# ===============================
# LOAD AI
# ===============================
print("[AI] Loading model...")
model = load_model(MODEL_PATH)
scaler = joblib.load(SCALER_PATH)
threshold = float(np.load(THRESHOLD_PATH))
print(f"[AI] Threshold = {threshold:.6f}")

# ===============================
# EMERGENCY (stub)
# ===============================
def is_emergency():
    return False

# ===============================
# SERIAL PARSE
# ===============================
def parse_mpu_line(line: str):
    parts = line.split(",")
    if len(parts) != 7:
        return None
    try:
        return {
            "t_ms": int(parts[0]),
            "pc_ts": time.time(),
            "AcX": int(parts[1]),
            "AcY": int(parts[2]),
            "AcZ": int(parts[3]),
            "GyX": int(parts[4]),
            "GyY": int(parts[5]),
            "GyZ": int(parts[6]),
        }
    except ValueError:
        return None

# ===============================
# THREAD 1: SERIAL COLLECTOR
# ===============================
def serial_thread():
    global connected, latest_sensor

    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    connected = True
    print("[SERIAL] Connected")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line or line.startswith("t,AcX"):
                continue

            data = parse_mpu_line(line)
            if data is None:
                continue

            with lock:
                latest_sensor = data

            try:
                sensor_queue.put_nowait(data)
            except Full:
                try:
                    sensor_queue.get_nowait()
                except Empty:
                    pass
                try:
                    sensor_queue.put_nowait(data)
                except Full:
                    pass

        except Exception as e:
            print("[SERIAL ERROR]", e)
            connected = False
            time.sleep(1)

# ===============================
# THREAD 2: AI INFERENCE
# ===============================
AI_HZ = 10
AI_PERIOD = 1.0 / AI_HZ

def ai_thread():
    global anomaly_duration

    last_run = 0.0

    while True:
        sensor = sensor_queue.get()

        # backlog flush → 최신 센서만 사용
        while True:
            try:
                sensor = sensor_queue.get_nowait()
            except Empty:
                break

        now = time.time()
        if now - last_run < AI_PERIOD:
            continue
        last_run = now

        if is_emergency():
            mpu_buffer.clear()
            anomaly_duration = 0.0
            continue

        mpu_buffer.append([sensor[k] for k in FEATURE_NAMES])

        if len(mpu_buffer) < SEQ_SIZE:
            continue

        seq_np = np.array(mpu_buffer, dtype=np.float32).reshape(1, SEQ_SIZE, 6)
        seq_scaled = scaler.transform(
            seq_np.reshape(-1, 6)
        ).reshape(1, SEQ_SIZE, 6)

        recon = model.predict(seq_scaled, verbose=0)
        mae = float(np.mean(np.abs(recon - seq_scaled)))

        over = mae > threshold
        if over:
            anomaly_duration += AI_PERIOD
        else:
            anomaly_duration = 0.0

        ai_event_queue.put({
            "ts": time.time(),
            "mae": mae,
            "over_threshold": over,
            "anomaly_duration": round(anomaly_duration, 2),
            "anomaly": anomaly_duration >= ANOMALY_TIME_SEC,
            "lag_ms": int((time.time() - sensor["pc_ts"]) * 1000),
        })

# ===============================
# THREAD 3: WEBSOCKET SENDER
# ===============================
SEND_HZ = 30

def ws_thread(server):
    global seq
    period = 1.0 / SEND_HZ

    while True:
        time.sleep(period)

        payload = {
            "type": "tick",
            "seq": seq,
            "connected": connected,
            "mpu": None,
            "ai": None,
        }

        with lock:
            payload["mpu"] = latest_sensor

        last_ai = None
        while not ai_event_queue.empty():
            last_ai = ai_event_queue.get()
        if last_ai is not None:
            payload["ai"] = last_ai

        seq += 1

        if server.clients:
            server.send_message_to_all(json.dumps(payload))

# ===============================
# MAIN
# ===============================
def main():
    server = WebsocketServer(port=WS_PORT)
    print(f"[WS] ws://localhost:{WS_PORT}")

    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=ai_thread, daemon=True).start()
    threading.Thread(target=ws_thread, args=(server,), daemon=True).start()

    server.run_forever()

if __name__ == "__main__":
    main()