import threading
import time
from uart.uart import uart

u = uart('COM4', 9600)
emergency = threading.Event()

def monitor():
    while True:
        received_data = u.receive()
        if (received_data == "111"):
            emergency.set() #비상상태 ON
        
        elif (received_data == "000"):
            emergency.clear() #비상상태 OFF

threading.Thread(target=monitor, daemon=True).start()

while True:
    #비상상태라면
    if (emergency.is_set()):
        print(f"EMERGENCY")
    
    else:
        print(f"WORKING")
    
    time.sleep(1)