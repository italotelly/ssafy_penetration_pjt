from robot.robot import dobot
from uart.uart import uart
from vision.vision import RealSenseColorDetector
from modbus.client import ModbusTCPClient
import cv2
import threading
import time

#################### MEMO ######################
'''
0. 컨베이어 벨트, Turtlebot3 ROS2 추가
1. 모드버스 IP, PORT 바꿔야함
2. 스레드 부분, receive_data 상태 갱신하고 None으로 바뀌는지 확인
3. 모드버스 구조 생각해서, 모드버스 write도 구현하자
4. PICK_POSITION 실제로 측정해서 고치자
5. SORT_POSITION 실제로 측정해서 고치자
6. 현재 COMPLETE_TASK → DETECT_OBJECT로 돌아오고 있는 상황인데 STM32가 컨베이어 벨트를 반드시 ON했다는 보장이 없다.
'''
################################################

#################### DEFINE ####################
NOW_STATE = "WAIT_START"

START_PROCESS_FLAG = False
FINISH_PROCESS_FLAG = False
CLASSIFY_OBJECT_FLAG = False
EMERGENCY_FLAG = False
COLOR_CODE = {
    "RED": "001",
    "GREEN": "010",
    "BLUE": "011",
    "YELLOW": "100",
}
PICK_POSITION_1 = [141.87, -233.48, 87.7, -53.06]
PICK_POSITION_2 = [121.78, -257.6, 20.84, -64.7]
SORT_POSITION = {
    "RED":     [251.84, 66.75, -18.93, 14.84],
    "GREEN":   [256.83, -7.62, -17.89, -1.7],
    "BLUE":    [241.76, -86.3, -18.49, -19.65],
    "YELLOW":  [197.07, -176.54, -28.39, -41.86],
}
HOME_POSITION = [209.67, 0.18, 99.97, 0.05]
last_detected_color = None
step = 0
temp = None
cnt = 0
move_sent = False
TOLERANCE = 1.0
'''''''''''''''''''''''''''''''''''''''''''''''''''
시작 대기 : WAIT_START
공정 시작 : START_PROCESS
공정 종료 : FINISH_PROCESS
물체 탐지 : DETECT_OBJECT
분류 대기 : WAIT_CLASSIFY
물체 분류 : CLASSIFY_OBJECT
작업 완료 : COMPLETE_TASK
비상 시작 : EMERGENCY_ON
비상 종료 : EMERGENCY_OFF
'''''''''''''''''''''''''''''''''''''''''''''''''''
def is_reached(current_pose: dict, target_pose: list, tol=TOLERANCE):
    cx = current_pose["x"]
    cy = current_pose["y"]
    cz = current_pose["z"]
    
    tx = target_pose[0]
    ty = target_pose[1]
    tz = target_pose[2]
    
    dx = abs(cx - tx)
    dy = abs(cy - ty)
    dz = abs(cz - tz)
    
    return dx < tol and dy < tol and dz < tol
###################################################
#################### SETUP ####################
# Modbus 연결
client = ModbusTCPClient('192.168.110.101', 20000)
client.connect()
client.write_log("[MODBUS] Client connected successfully.")
print("[MODBUS] Client connected successfully.")

# Dobot 초기화
robot = dobot('COM6')
robot.connect()
robot.home()
client.write_log("[DOBOT] Device connected successfully.")
print("[DOBOT] Device connected successfully.")

# UART 연결
comm = uart('COM4', 9600)
client.write_log("[UART] Device connected successfully.")
print("[UART] Device connected successfully.")

# D435i 연결
vision = RealSenseColorDetector(roi_area=(230, 280, 425, 475))
client.write_log("[D435i] Device connected successfully.")
print("[D435i] Device connected successfully.")
###################################################
################## STATE_FUNCTIONS ################
def wait_start_func():
    global START_PROCESS_FLAG
    
    while True:
        if START_PROCESS_FLAG:
            client.write_log("[SYSTEM] Start Process")
            print("[SYSTEM] Start Process")
            START_PROCESS_FLAG = False
            return "START_PROCESS"
                
        yield

def start_process_func():
    client.write_coil(True)
    yield
    return "DETECT_OBJECT"

def detect_object_func():
    global last_detected_color
    
    while True:
        view, detected_color = vision.detect_one_frame()
        yield
        
        if view is not None:
            cv2.imshow("D435i", view)
            cv2.waitKey(1)
        yield

        if detected_color:
            client.write_coil(False)
            client.write_log(f"[D435i] Detected: {detected_color}")
            print(f"[D435i] Detected: {detected_color}")
            last_detected_color = detected_color
            comm.send(COLOR_CODE[detected_color])
            cv2.destroyWindow("D435i")
            return "WAIT_CLASSIFY"
        
        yield

def wait_classify_func():
    global CLASSIFY_OBJECT_FLAG
    
    while True:
        if CLASSIFY_OBJECT_FLAG:
            CLASSIFY_OBJECT_FLAG = False
            return "CLASSIFY_OBJECT"
        
        yield
      
def classify_object_func():
    global step
    global move_sent
    step = 0
    move_sent = False
    client.write_log("[DOBOT] Start PICK & SORT")
    print("[DOBOT] Start PICK & SORT")
    
    while True:
        if step == 0:
            if not move_sent:
                robot.move(*PICK_POSITION_1)
                move_sent = True
            
            pose = robot.get_pose()
            if is_reached(pose, PICK_POSITION_1):
                step = 1
                move_sent = False
            yield
            
        if step == 1:
            if not move_sent:
                robot.move(*PICK_POSITION_2)
                move_sent = True
                
            pose = robot.get_pose()
            if is_reached(pose, PICK_POSITION_2):
                step = 2
                move_sent = False 
            yield
        
        if step == 2:
            robot.suction(1)
            time.sleep(0.5)
            step = 3
            yield
            
        if step == 3:
            if not move_sent:
                robot.move(*PICK_POSITION_1)
                move_sent = True
            
            pose = robot.get_pose()
            if is_reached(pose, PICK_POSITION_1):
                step = 4
                move_sent = False
            yield
            
        if step == 4:
            if not move_sent:
                sort_pos = SORT_POSITION[last_detected_color]
                robot.move(*sort_pos)
                move_sent = True
            
            pose = robot.get_pose()
            if is_reached(pose, sort_pos):
                step = 5
                move_sent = False
            yield
            
        if step == 5:
            robot.suction(0)
            step = 6
            yield
        
        if step == 6:
            return "COMPLETE_TASK"
            
def complete_task_func():
    global FINISH_PROCESS_FLAG
    client.write_log("[DOBOT] Task Completed")
    print(f"[DOBOT] Task Completed")
    if not FINISH_PROCESS_FLAG:
        client.write_coil(True)
        comm.send("000")
    
    yield
    
    if FINISH_PROCESS_FLAG:
        FINISH_PROCESS_FLAG = False
        return "FINISH_PROCESS"
    
    else:   
        return "DETECT_OBJECT"

def finish_process_func():
    global step
    global move_sent
    step = 0
    move_sent = False
    
    client.write_log("[SYSTEM] Finish Process")
    print("[SYSTEM] Finish Process")
    
    while True:
        if step == 0:
            if not move_sent:
                robot.home()
                move_sent = True
                
            pose = robot.get_pose()
            if is_reached(pose, HOME_POSITION):
                step = 1
                move_sent = False 
            yield
            
        if step == 1:
            client.export_logs()
            return "WAIT_START"
###################################################
################## THREAD ##################
def stm32_listener():
    print("[UART] stm32_listener thread started!")
    
    global START_PROCESS_FLAG
    global FINISH_PROCESS_FLAG
    global CLASSIFY_OBJECT_FLAG
    global EMERGENCY_FLAG
    global step
    global temp
    global cnt
    
    while True:
        receive_data = comm.receive()
        
        if receive_data:
            if receive_data == "110":
                if NOW_STATE == "WAIT_START":
                    client.write_log("[UART] Received: START_PROCESS(110) from STM32.")
                    print("[UART] Received: START_PROCESS(110) from STM32.")
                    START_PROCESS_FLAG = True
            
            elif receive_data == "100":
                if NOW_STATE != "WAIT_START":
                    client.write_log("[UART] Received: FINISH_PROCESS(100) from STM32.")
                    print("[UART] Received: FINISH_PROCESS(100) from STM32.")
                    FINISH_PROCESS_FLAG = True
                    
            elif receive_data == "101":
                client.write_log("[UART] Received: CLASSIFY_OBJECT(101) from STM32.")
                print("[UART] Received: CLASSIFY_OBJECT(101) from STM32.")
                CLASSIFY_OBJECT_FLAG = True
            
            elif receive_data == "111":
                client.write_log("[UART] Received: EMERGENCY_ON(111) from STM32.")
                print("[UART] Received: EMERGENCY_ON(111) from STM32.")
                temp = step
                EMERGENCY_FLAG = True
            
            elif receive_data == "000":
                client.write_log("[UART] Received: EMERGENCY_OFF(000) from STM32.")
                print("[UART] Received: EMERGENCY_OFF(000) from STM32.")
                step = temp
                cnt = 0
                EMERGENCY_FLAG = False
                    
        time.sleep(0.01)
####################################################
################## MAIN ##################
t = threading.Thread(target=stm32_listener)
t.start()

STATE_FUNCTIONS = {
    "WAIT_START": wait_start_func,
    "START_PROCESS": start_process_func,
    "FINISH_PROCESS": finish_process_func,
    "DETECT_OBJECT": detect_object_func,
    "WAIT_CLASSIFY": wait_classify_func,
    "CLASSIFY_OBJECT": classify_object_func,
    "COMPLETE_TASK": complete_task_func,
}

current = STATE_FUNCTIONS[NOW_STATE]()

while True:
    if EMERGENCY_FLAG and cnt == 0:
        robot.stop()
        robot.clear()
        cnt += 1
        move_sent = False
        time.sleep(0.01)
        continue
    
    if EMERGENCY_FLAG and cnt >= 1:
        robot.start()
        time.sleep(0.01)
        continue
    
    try:
        next(current)
    
    except StopIteration as e:
        NOW_STATE = e.value
        current = STATE_FUNCTIONS[NOW_STATE]()
        
    time.sleep(0.01)
##########################################




