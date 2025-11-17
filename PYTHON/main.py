from uart.uart import uart
import time

a = uart('COM7', 115200)

while True:
    receive_data = a.receive()
    print(f"데이터 수신을 대기합니다.")
    if (receive_data):
        print(f"데이터를 수신받았습니다.")
        print(f"{receive_data}")
        break
    