import json
import time
import math
import threading
from websocket_server import WebsocketServer

WS_PORT = 3001
SEND_HZ = 20

seq = 0
t0 = time.time()

print("### FILE START ###")

def make_mpu(t):
    return {
        "ax": 0.5 * math.sin(t),
        "ay": 0.5 * math.cos(t),
        "az": 0.2 * math.sin(t * 0.5),
        "gx": 0.8 * math.sin(t * 1.5),
        "gy": 0.8 * math.cos(t * 1.5),
        "gz": 0.3 * math.sin(t),
    }

def make_ai(t):
    return "normal" if (int(t) // 3) % 2 == 0 else "abnormal"

def tick_loop(server):
    global seq
    while True:
        t = time.time() - t0
        seq += 1

        payload = {
            "type": "tick",
            "seq": seq,
            "connected": True,
            "mpu": make_mpu(t),
            "ai": make_ai(t),
        }

        server.send_message_to_all(json.dumps(payload))
        time.sleep(1.0 / SEND_HZ)

def on_new_client(client, server):
    print("Client connected:", client["id"])
    server.send_message(client, json.dumps({
        "type": "snapshot",
        "seq": seq,
        "connected": True,
        "mpu": make_mpu(time.time() - t0),
        "ai": make_ai(time.time() - t0),
    }))

if __name__ == "__main__":
    print("### MAIN ENTERED ###")

    server = WebsocketServer(
        host="0.0.0.0",
        port=WS_PORT,
        loglevel=1
    )

    server.set_fn_new_client(on_new_client)

    th = threading.Thread(
        target=tick_loop,
        args=(server,),
        daemon=True
    )
    th.start()

    print(f"✅ WS running on ws://127.0.0.1:{WS_PORT}")
    server.run_forever()   # ❗ 여기서 프로그램이 살아있음
