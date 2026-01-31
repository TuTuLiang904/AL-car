import cv2
import time
import serial
import serial.tools.list_ports
import numpy as np
import struct
import threading
from flask import Flask, Response

# ================= 1. 硬件与PID配置 (使用急弯优化后的参数) =================
BAUD_RATE = 115200
BASE_SPEED = 5.0      
Kp = -0.02           # 急弯优化
MAX_TURN_VAL = 5   # 急弯优化

# ================= 2. 双模型配置 (CPU ONNX) =================
# 模型1: 避障
MODEL_PATH_AVOID = '/root/car_vision/best_escape.onnx'   
INPUT_SIZE_AVOID = (640, 640)
OBSTACLE_WIDTH_RATIO = 0.225 

# 模型2: 物块识别 (恢复这个模型)
MODEL_PATH_BLOCK = '/root/car_vision/yolo_block.onnx'
INPUT_SIZE_BLOCK = (640, 640)

# ================= 3. 动作参数 =================
AVOID_SPEED_OUT_L = 1;  AVOID_SPEED_OUT_R = 6    
AVOID_SPEED_PASS_L = 5; AVOID_SPEED_PASS_R = 5
AVOID_SPEED_BACK_L = 6; AVOID_SPEED_BACK_R = 1  
AVOID_TIME_OUT = 0.8     
AVOID_TIME_PASS = 1.6    
AVOID_TIME_BACK = 1.6    

# ================= 4. 全局变量 =================
app = Flask(__name__)
lock = threading.Lock()
is_running = True
ser = None 

# 状态机: 0=巡线+避障, 1=原地搜寻物块, 2=视觉对准抓取
MISSION_STAGE = 0 

global_obstacle_flag = False   
global_block_center = None  # 存储物块中心 (cx, cy)

# 逻辑控制
red_line_count = 0          
ignore_obstacle_until = 0   

# ================= 5. 串口与通信 =================
def auto_connect_serial():
    global ser
    ports = list(serial.tools.list_ports.comports())
    target_dev = None
    for p in ports:
        if "USB" in p.device: target_dev = p.device; break
    if target_dev is None:
        for i in range(4):
            try: serial.Serial(f"/dev/ttyUSB{i}", BAUD_RATE).close(); target_dev = f"/dev/ttyUSB{i}"; break
            except: pass
    if target_dev:
        try: ser = serial.Serial(target_dev, BAUD_RATE, timeout=0.1); print(f"✅ 串口: {target_dev}")
        except: print("❌ 失败")
    else: print("⚠️ 无串口")

auto_connect_serial()

def send_motor_packet(left, right, cmd=0):
    if ser is None: return
    left = int(max(min(left, 100), -100))
    right = int(max(min(right, 100), -100))
    try:
        packet = struct.pack('BbbBB', 0xFF, right, left, cmd, 0xFE)
        ser.write(packet)
    except: pass

def execute_move(left, right, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        send_motor_packet(left, right, 0)
        time.sleep(0.1) 

def action_stop(): send_motor_packet(0, 0)
def action_turn_left(): send_motor_packet(-1, 1) 
def action_turn_right(): send_motor_packet(1, -1)

# [辅助] 盲走 (用于卸货或离开红线)
def move_forward_blind():
    print(">>> ⬆️ 盲走离开/对准...")
    send_motor_packet(5, 5)
    time.sleep(1.5)
    action_stop()

# [抓取] CMD=2
def execute_grab_action():
    print(">>> 🤖 执行抓取 (CMD=2)...")
    for _ in range(3): send_motor_packet(0, 0, 2); time.sleep(0.1)
    print(">>> 等待机械臂 (8s)...")
    time.sleep(8.0)
    print(">>> ✅ 抓取完成")

# [卸货] CMD=3
def execute_unload_action():
    print(">>> 🚚 执行卸货 (CMD=3)...")
    for _ in range(3): send_motor_packet(0, 0, 3); time.sleep(0.1)
    print(">>> 等待机械臂 (8s)...")
    time.sleep(8.0)
    print(">>> ✅ 卸货完成")
def move_forward_blind():
    print(">>> ⬆️ 盲走10cm (时长1.5秒)...")
    
    # 目标时长
    TOTAL_DURATION = 1.2  
    # 每次发送间隔 (必须小于下位机的看门狗超时时间)
    INTERVAL = 0.2        
    
    start_time = time.time()
    
    # 循环发送指令，直到时间耗尽
    while time.time() - start_time < TOTAL_DURATION:
        send_motor_packet(5, 5) # 持续发送前进指令
        time.sleep(INTERVAL)    # 短暂休眠
        
    action_stop() # 最后才停车
    time.sleep(0.5)
# ================= 6. 摄像头 =================
class MJPEGLoader:
    def __init__(self):
        index_list = [8, 0, 1]
        self.cap = None
        for idx in index_list:
            tmp = cv2.VideoCapture(idx)
            tmp.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            tmp.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            tmp.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if tmp.isOpened():
                ret, _ = tmp.read()
                if ret: self.cap = tmp; break
                else: tmp.release()
        self.frame = None; self.ret = False; self.running = True
        if self.cap: threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with lock: self.frame = frame; self.ret = ret
            else: time.sleep(0.01)

    def get_frame(self):
        with lock: return self.frame.copy() if self.ret else None

cam = MJPEGLoader()

# ================= 7. 红线检测 =================
def detect_red_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 70, 50]); upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50]); upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    h, w = mask.shape
    roi = mask[int(h*0.5):, :] # 保持优化后的视野
    return (cv2.countNonZero(roi) / (roi.shape[0] * roi.shape[1])) > 0.05

# ================= 8. AI 线程 (双模型分时复用) =================
def ai_thread_func():
    global global_obstacle_flag, global_block_center, MISSION_STAGE
    
    # 加载双模型
    try: 
        net_avoid = cv2.dnn.readNetFromONNX(MODEL_PATH_AVOID)
        net_block = cv2.dnn.readNetFromONNX(MODEL_PATH_BLOCK)
        print(f"✅ 双模型加载成功")
    except Exception as e: 
        print(f"❌ 模型加载失败: {e}"); return

    print(">>> 🔍 AI 线程启动")
    
    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.01); continue

        # ---------------------------------------------------------
        # 场景 0: 巡线模式 -> 运行【避障模型】
        # ---------------------------------------------------------
        if MISSION_STAGE == 0:
            # 如果处于无敌时间，直接跳过避障
            if time.time() < ignore_obstacle_until:
                global_obstacle_flag = False
                time.sleep(0.05)
                continue

            blob = cv2.dnn.blobFromImage(frame, 1/255.0, INPUT_SIZE_AVOID, swapRB=True, crop=False)
            net_avoid.setInput(blob)
            outputs = np.transpose(net_avoid.forward()[0])
            
            max_w_ratio = 0
            for i in range(outputs.shape[0]):
                confidence = outputs[i][4]
                if confidence > 0.8:
                    print(f"confidence:{confidence:.2f}") 
                    w = outputs[i][2]
                    ratio = w / INPUT_SIZE_AVOID[0]
                    if ratio > max_w_ratio: max_w_ratio = ratio
            
            if max_w_ratio > OBSTACLE_WIDTH_RATIO:
                if not global_obstacle_flag: print(f">>> ⚠️ 障碍物: {max_w_ratio:.2f}")
                global_obstacle_flag = True
            else:
                global_obstacle_flag = False
            time.sleep(0.03)
        # ---------------------------------------------------------
        # 场景 1 & 2: 抓取模式 -> 运行【物块识别模型】
        # ---------------------------------------------------------
        elif MISSION_STAGE == 1 or MISSION_STAGE == 2:
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, INPUT_SIZE_BLOCK, swapRB=True, crop=False)
            net_block.setInput(blob)
            outputs = np.transpose(net_block.forward()[0])
            
            best_conf = 0
            found_center = None
            
            for i in range(outputs.shape[0]):
                confidence = outputs[i][4]
                # 假设 Class 0 是蓝色物块 (根据你之前的代码)
                # 你的 ONNX 输出格式是 [x, y, w, h, conf, cls0, cls1...]
                # 这里简化处理，直接取 conf 做判断，假设模型只训练了物块
                if confidence > 0.6:
                     # 如果有多类别，这里需要加 class_id 判断
                     # class_scores = outputs[i][5:]
                     # class_id = np.argmax(class_scores)
                     # if class_id == 1: ...
                     
                     if confidence > best_conf:
                        best_conf = confidence
                        # 坐标还原
                        cx = outputs[i][0]
                        cy = outputs[i][1]
                        # 你的模型可能是归一化的，也可能是像素级的
                        # 如果是归一化 (0~1)，需要 * 640
                        if cx < 1.5: cx *= 640; cy *= 640
                        found_center = (cx, cy)
            
            global_block_center = found_center
        
        time.sleep(0.03)

# ================= 9. 主控循环 (包含视觉搜寻+卸货逻辑) =================
def control_loop():
    global global_obstacle_flag, MISSION_STAGE
    global red_line_count, ignore_obstacle_until
    
    avoid_cd = 0
    print("🚀 3秒后出发..."); time.sleep(3); print("GO!")

    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.01); continue
        
        # ==========================================================
        # 🟢 Stage 0: 正常巡线 + 红线检测
        # ==========================================================
        if MISSION_STAGE == 0:
            
            # --- 红线检测 ---
            if detect_red_line(frame) and red_line_count < 2:
                # 如果刚离开红线(无敌时间内)，忽略红线
                if time.time() < ignore_obstacle_until:
                    pass
                else:
                    print(">>> 🔴 红线触发！停车！")
                    action_stop()
                    time.sleep(1.0)
                    
                    red_line_count += 1
                    
                    # [事件1] 第一次红线 -> 切换到视觉抓取模式
                    if red_line_count == 1:
                        print(">>> [任务1] 切换 Stage 1 (视觉搜寻)...")
                        # 往前稍微走一点，让物块进入视野
                        execute_move(5, 5, 0.5) 
                        MISSION_STAGE = 1
                        continue # 跳出当前循环，让 Stage 1 接管
                    
                    # [事件2] 第二次红线 -> 执行盲卸货 (手里有货，没法视觉搜)
                    elif red_line_count == 2:
                        print(">>> [任务2] 执行卸货流程...")
                        move_forward_blind()    # 盲走10cm
                        execute_unload_action() # 卸货
                        
                        # 卸货完，继续走
                        print(">>> 🛡️ 卸货完成，开启无敌模式继续走...")
                        ignore_obstacle_until = time.time() + 20.0
                        execute_move(5, 1, 1.5) # 离开红线区
                    
            # --- 避障逻辑 ---
            if global_obstacle_flag and (time.time() > avoid_cd) and (time.time() > ignore_obstacle_until):
                print(">>> 🔄 避障动作...")
                execute_move(AVOID_SPEED_OUT_L, AVOID_SPEED_OUT_R, AVOID_TIME_OUT)
                execute_move(AVOID_SPEED_PASS_L, AVOID_SPEED_PASS_R, AVOID_TIME_PASS)
                execute_move(AVOID_SPEED_BACK_L, AVOID_SPEED_BACK_R, AVOID_TIME_BACK)
                avoid_cd = time.time() + 30.0; global_obstacle_flag = False
                continue

                # --- 3. 正常巡线 (PID) ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)
            ret, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            h, w = binary.shape
            roi = binary[int(h*0.7):, :]
            M = cv2.moments(roi)
            target_L = 0; target_R = 0
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                offset = cx - 320 # 你的原版是 320
                turn = -offset * Kp
                turn = max(min(turn, MAX_TURN_VAL), -MAX_TURN_VAL)
                target_L = BASE_SPEED + turn
                target_R = BASE_SPEED - turn
            else:
                target_L = 6; target_R = 2
            send_motor_packet(target_L, target_R)

        # ==========================================================
        # 🟡 Stage 1: 原地搜寻物块 (final_mission_red 算法)
        # ==========================================================
        elif MISSION_STAGE == 1:
            if global_block_center is None:
                print(">>> 🔍 没看到物块，右转搜索...")
                action_turn_right()
                time.sleep(0.2)   # 恢复正常步长
                action_stop() 
                time.sleep(0.5)
            else:
                print(">>> 🎉 发现物块！切换 Stage 2 (视觉对准)...")
                MISSION_STAGE = 2
        
        # ==========================================================
        # 🔵 Stage 2: 视觉对准 + 抓取
        # ==========================================================
        elif MISSION_STAGE == 2:
            if global_block_center is not None:
                bx, by = global_block_center
                
                # 简单的视觉伺服逻辑 (保持原版)
                # 目标是在画面中心偏下
                TARGET_X = 277.5
                err_x = bx - TARGET_X
                
                print(f">>> 🎯 瞄准中: err_x={err_x:.1f} y={by:.1f}")
                print(bx)
                # 对准 X 轴
                if abs(err_x) < 20:
                    # 如果 Y 轴足够大 (说明离得近了)
           
                    print(">>> ✅ 距离合适，抓取！")
                    action_stop()
                    time.sleep(0.5)
                    move_forward_blind()
                    time.sleep(0.5)
                    execute_grab_action()
                        
                        # [关键修改] 抓取完，切回 Stage 0 继续走！
                    print(">>> 🛡️ 抓取完成，切回巡线模式，开启无敌时间...")
                    MISSION_STAGE = 0
                    ignore_obstacle_until = time.time() + 20.0
                    execute_move(1, 1, 1.5) # 往前走，防止还在红线区
                        
 
                elif err_x > 0:
                    action_turn_right()
                    time.sleep(0.2)   # 恢复正常步长
                    action_stop() 
                    time.sleep(0.5)
                else:
                    action_turn_left()
                    time.sleep(0.2)   # 恢复正常步长
                    action_stop() 
                    time.sleep(0.5)
                
                time.sleep(0.1)
            else:
                print(">>> ⚠️ 丢失目标，回 Stage 1...")
                MISSION_STAGE = 1

# ================= 10. 网页视频流 =================
def generate_frames():
    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.05); continue
        ret, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if ret: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + bytearray(buf) + b'\r\n')
        time.sleep(0.04)

@app.route('/')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    t1 = threading.Thread(target=control_loop); t1.daemon = True; t1.start()
    t2 = threading.Thread(target=ai_thread_func); t2.daemon = True; t2.start()
    print("✅ 系统已启动 (双模型 + 视觉抓取 + 卸货 + 连续执行)")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)