import json
import time
import threading
import serial
from websocket_server import WebsocketServer

# ex)
SERIAL_PORT = "COM5"
BAUDRATE = 115200
WS_PORT = 3001
SEND_HZ = 20

# 자물쇠
latest_lock = threading.Lock()
# 가장 최근 시리얼 센서 값
latest_mpu = None
# 가장 최근 ai 추론 결과 값
latest_ai = None
# 웹으로 보낸 메시지 번호
seq = 0
# 시리얼 연결 상태 표시
connected = False

# 시리얼로 값 읽기
def serial_thread():
    global latest_mpu, connected
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    connected = True

    # while True:
        # 파싱 함수 넣어서 변수에 넣기

        # ---- 파싱 함수 필요 ----

        #----------------

        # Thread Lock 걸어서 latest_mpu 값 받아주세요. 형식은 자유롭게 받아주시고 제가 파싱 따로 하겠습니다.
        # with latest_lock:
        #     latest_mpu = {"ts": int(time.time()*1000), **mpu}

# ai 추론과 ws으로 값 보내기 함수
def ai_ws_thread(server):
    global latest_ai, seq
    # 20Hz 사용 - ai 추론 및 웹으로 보내는 주기
    # 0.005s, 50ms
    interval = 1.0 / SEND_HZ
    while True:
        time.sleep(interval)

        # 안정성 위해 Thread Lock
        with latest_lock:
            if latest_mpu is None:
                continue
            mpu = latest_mpu
            
            # ai 추론 함수 정의 필요
            # ai = infer(mpu)
            # latest_ai = ai
            
            # 웹으로 데이터 보내기 위한 데이터 번호
            seq += 1
            payload = {
                # tick -> 주기적으로 날아오는 메시지라는 뜻
                "type": "tick",
                "seq": seq,
                "connected": connected,
                "mpu": mpu,
                # ai 추론 결과도 같이 보내면 좋겠음
                # "ai": ai
            }

        # 락 밖에서 전송
        if server.clients:
            server.send_message_to_all(json.dumps(payload))


def main():
    server = WebsocketServer(port=WS_PORT, loglevel=None)
    threading.Thread(target=serial_thread, daemon=True).start()
    threading.Thread(target=ai_ws_thread, args=(server,), daemon=True).start()
    print(f"WS on ws://localhost:{WS_PORT}")
    server.run_forever()

if __name__ == "__main__":
    main()