import cv2
import time
import serial
import serial.tools.list_ports
import numpy as np
import struct
import threading
from flask import Flask, Response

# ================= 1. 硬件与PID配置 (完全不动) =================
BAUD_RATE = 115200
BASE_SPEED = 5.0      
Kp = -0.02           
MAX_TURN_VAL = 5.0    

# ================= 2. 模型配置 (只用 CPU 跑 ONNX) =================
# [注意] 这里只保留了避障模型，物块识别已移除
MODEL_PATH_AVOID = '/root/car_vision/best_escape.onnx'   
INPUT_SIZE_AVOID = (640, 640)
OBSTACLE_WIDTH_RATIO = 0.225  # 沿用您文件里的参数

# ================= 3. 动作参数 (完全不动) =================
AVOID_SPEED_OUT_L = 1;  AVOID_SPEED_OUT_R = 7    
AVOID_SPEED_PASS_L = 5; AVOID_SPEED_PASS_R = 5
AVOID_SPEED_BACK_L = 7; AVOID_SPEED_BACK_R = 1  
AVOID_TIME_OUT = 0.8     
AVOID_TIME_PASS = 1.0    
AVOID_TIME_BACK = 0.8    

# ================= 4. 全局变量 =================
app = Flask(__name__)
lock = threading.Lock()
is_running = True
ser = None 

global_obstacle_flag = False   

# [新增] 逻辑控制变量
red_line_count = 0          # 红线计数器
ignore_obstacle_until = 0   # 避障屏蔽截止时间

# ================= 5. 串口与通信 (完全不动) =================
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

# [新增] 盲走动作 (模拟走10cm)
def move_forward_blind():
    print(">>> ⬆️ 盲走10cm (时长4秒)...")
    
    # 目标时长
    TOTAL_DURATION = 1.5  
    # 每次发送间隔 (必须小于下位机的看门狗超时时间)
    INTERVAL = 0.2        
    
    start_time = time.time()
    
    # 循环发送指令，直到时间耗尽
    while time.time() - start_time < TOTAL_DURATION:
        send_motor_packet(5, 5) # 持续发送前进指令
        time.sleep(INTERVAL)    # 短暂休眠
        
    action_stop() # 最后才停车
    time.sleep(0.5)

# [保留] 抓取 (CMD=2)
def execute_grab_action():
    print(">>> 🤖 执行盲抓 (CMD=2)...")
    for _ in range(3): send_motor_packet(0, 0, 2); time.sleep(0.1)
    print(">>> 等待机械臂动作 (8秒)...")
    time.sleep(8.0)
    print(">>> ✅ 抓取完成")

# [新增] 卸货 (CMD=3)
def execute_unload_action():
    print(">>> 🚚 执行卸货 (CMD=3)...")
    for _ in range(3): send_motor_packet(0, 0, 3); time.sleep(0.1)
    print(">>> 等待机械臂动作 (8秒)...")
    time.sleep(8.0)
    print(">>> ✅ 卸货完成")

# ================= 6. 摄像头 (保留原版逻辑) =================
class MJPEGLoader:
    def __init__(self):
        index_list = [8, 0, 1]
        self.cap = None
        for idx in index_list:
            tmp = cv2.VideoCapture(idx)
            # 这里的参数保持你原来的不动
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

# ================= 7. 红线检测 (完全不动) =================
def detect_red_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 70, 50]); upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50]); upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    h, w = mask.shape
    roi = mask[int(h*0.5):, :] 
    return (cv2.countNonZero(roi) / (roi.shape[0] * roi.shape[1])) > 0.05

# ================= 8. AI 线程 (只留避障 ONNX) =================
def ai_thread_func():
    global global_obstacle_flag
    
    # 1. 加载避障模型 (标准 OpenCV CPU 加载)
    try: 
        net_avoid = cv2.dnn.readNetFromONNX(MODEL_PATH_AVOID)
        print(f"✅ 避障模型加载成功: {MODEL_PATH_AVOID}")
    except Exception as e: 
        print(f"❌ 模型加载失败: {e}")
        return

    print(">>> 🔍 AI 线程启动 (仅避障)")
    
    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.01); continue

        # [修改] 如果当前时间在"无敌时间"内，强制关闭避障检测
        if time.time() < ignore_obstacle_until:
            global_obstacle_flag = False
            time.sleep(0.1) # 休息一下，省点CPU
            continue

        # === 避障推理 (保持你原来的逻辑，不用 NV12) ===
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, INPUT_SIZE_AVOID, swapRB=True, crop=False)
        net_avoid.setInput(blob)
        outputs = np.transpose(net_avoid.forward()[0])
        
        max_w_ratio = 0
        
        for i in range(outputs.shape[0]):
            confidence = outputs[i][4]
            # 你的原始阈值是 0.8，我保持不动，或者你可以改成 0.5
            if confidence > 0.8: 
                w = outputs[i][2]
                # 你的原始模型输出可能是归一化的或者像素级的
                # 根据你之前的代码，它是直接取值的，这里保持不动
                ratio = w / INPUT_SIZE_AVOID[0]
                if ratio > max_w_ratio:
                    max_w_ratio = ratio
        
        if max_w_ratio > OBSTACLE_WIDTH_RATIO:
            if not global_obstacle_flag:
                print(f">>> ⚠️ 障碍物触发! 占比:{max_w_ratio:.2f}")
            global_obstacle_flag = True
        else:
            global_obstacle_flag = False
        
        time.sleep(0.03)

# ================= 9. 主控循环 (大改：固定逻辑) =================
def control_loop():
    global global_obstacle_flag
    global red_line_count, ignore_obstacle_until
    
    avoid_cd = 0
    print("🚀 3秒后出发..."); time.sleep(3); print("GO!")

    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.01); continue
        
        # 始终处于巡线模式，直到遇到红线
        
        # --- 1. 红线检测 ---
        if detect_red_line(frame):
            print(">>> 🔴 红线触发！停车！")
            action_stop()
            time.sleep(1.0)
            
            red_line_count += 1
            
            # [第一次红线]：抓取
            if red_line_count == 1:
                print(">>> [任务1] 执行固定抓取流程")
                move_forward_blind()
                time.sleep(1.0)
                execute_move(1, -1, 3)  
                execute_grab_action()   # 盲抓
                
                # 开启20秒无敌模式
                print(">>> 🛡️ 开启10秒无敌模式 (屏蔽避障)")
                ignore_obstacle_until = time.time() + 10.0
                
                # 往前稍微走一点离开红线，防止重复触发
                # execute_move(5, 1, 1.5)
                
            # [第二次红线]：卸货
            elif red_line_count == 2:
                print(">>> [任务2] 执行固定卸货流程")
                move_forward_blind()  
                time.sleep(1.0)
                execute_move(1, -1, 3)  
                time.sleep(1.0)
                execute_unload_action() # 卸货 (CMD=3)
                
                print(">>> 🎉 全部完成，程序结束")
                while True:
                    action_stop()
                    time.sleep(1)
            
            continue # 跳过本次循环剩余部分，防止一边抓取一边避障

        # --- 2. 避障逻辑 (受 ignore_obstacle_until 控制) ---
        # 只有在非无敌时间，且避障标志为真时触发
        if global_obstacle_flag and (time.time() > avoid_cd) and (time.time() > ignore_obstacle_until):
            print(">>> 🔄 执行避障动作...")
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

# ================= 10. 网页视频流 (完全不动) =================
def generate_frames():
    while is_running:
        frame = cam.get_frame()
        if frame is None: time.sleep(0.05); continue
        # 这里保留你的 50 质量设置
        ret, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if ret: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + bytearray(buf) + b'\r\n')
        time.sleep(0.04)

@app.route('/')
def video_feed(): return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    t1 = threading.Thread(target=control_loop); t1.daemon = True; t1.start()
    t2 = threading.Thread(target=ai_thread_func); t2.daemon = True; t2.start()
    print("✅ 系统已启动 (逻辑固定版 - ONNX)")
    app.run(host='0.0.0.0', port=5050, debug=False, threaded=True)