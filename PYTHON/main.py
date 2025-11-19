from uart.uart import uart
import time

a = uart('COM4', 9600)

# while True:
#     a.send('001')
#     time.sleep(1)
#     a.send('000')

while True:
    receive_data = a.receive()
    print(f"데이터 수신을 대기합니다.")
    if (receive_data):
        print(f"데이터를 수신받았습니다.")
        print(f"{receive_data}")
        break
    
    
# while True:
    