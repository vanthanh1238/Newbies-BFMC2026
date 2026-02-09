import serial
import time
import sys
from src.Perception.perception_system import BFMC_Vision 

# --- CẤU HÌNH THÔNG SỐ ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Tốc độ
SPEED_NORMAL = 150      
SPEED_HIGHWAY = 250     
SPEED_REVERSE = -150    
SPEED_TURN = 250        

# Góc lái
STEER_STRAIGHT = -70
STEER_LEFT = -210
STEER_RIGHT = 120

# Thời gian
TIME_STOP_WAIT = 3.0    
TIME_REVERSE = 5.0      
TIME_TURN = 5.0       
COOLDOWN_TIME = 5.0     

# --- KẾT NỐI SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(" Serial OK!")
except Exception as e:
    print(" Lỗi Serial. Nhớ chạy: sudo chmod 666 /dev/ttyACM0")
    sys.exit()

def send_command(action, value):
    msg = f"#{action}:{value};;\r\n"
    ser.write(msg.encode('utf-8'))

# --- KHỞI ĐỘNG VISION ---
eye = BFMC_Vision()
eye.start() 
time.sleep(3) 

# --- TRẠNG THÁI XE ---
STATE = "NORMAL" 
cooldown_timer = 0 

try:
    print(" XE BẮT ĐẦU CHẠY!")
    send_command("kl", 30) 
    time.sleep(1)

    # 2. Trả lái về thẳng ngay lập tức
    print(" Cân chỉnh bánh xe thẳng tắp!")
    send_command("steer", STEER_STRAIGHT)
    time.sleep(1) 
    
    while True:
        objects = eye.get_objects()
        current_time = time.time()
        is_cooldown = current_time < cooldown_timer

        # --- PHẦN 1: QUYẾT ĐỊNH ---
        if STATE in ["NORMAL", "HIGHWAY"] and not is_cooldown:
            
            # 1. Ưu tiên: ĐÈN GIAO THÔNG (Đèn đỏ là phải dừng ngay)
            if "trafficlight_red" in objects:
                 print(" Gặp đèn ĐỎ! -> TRAFFIC LIGHT MODE")
                 STATE = "MANEUVER_TRAFFIC_LIGHT"

            # 2. Ưu tiên: Đường cấm
            elif "no_entry" in objects:
                print(" Gặp đường cấm! -> NO ENTRY MODE")
                STATE = "MANEUVER_NO_ENTRY"

            # 3. Ưu tiên: Stop/Crosswalk
            elif "stop" in objects or "crosswalk" in objects:
                print(" Gặp Stop/Crosswalk! -> STOP MODE")
                STATE = "MANEUVER_STOP"

            # 4. Cao tốc
            elif "highway_entry" in objects and STATE == "NORMAL":
                print("Vào cao tốc -> Tăng tốc!")
                STATE = "HIGHWAY"
                cooldown_timer = current_time + 2
            
            elif "highway_exit" in objects and STATE == "HIGHWAY":
                print(" Hết cao tốc -> Giảm tốc!")
                STATE = "NORMAL"
                cooldown_timer = current_time + 2

        # --- PHẦN 2: THỰC THI (Action) ---

        if STATE == "NORMAL":
            send_command("speed", SPEED_NORMAL)
            send_command("steer", STEER_STRAIGHT)

        elif STATE == "HIGHWAY":
            send_command("speed", SPEED_HIGHWAY)
            send_command("steer", STEER_STRAIGHT)

        # >>> XỬ LÝ ĐÈN GIAO THÔNG (MỚI)
        elif STATE == "MANEUVER_TRAFFIC_LIGHT":
            # B1: Dừng xe ngay lập tức
            send_command("speed", 0)
            print(" Đang dừng chờ đèn XANH...")
            
            # B2: Vòng lặp chờ (Waiting Loop)
            # Xe sẽ kẹt ở đây mãi mãi cho đến khi thấy đèn xanh
            start_wait = time.time()
            while True:
                fresh_objects = eye.get_objects() # Cập nhật ảnh mới nhất
                
                # Nếu thấy đèn xanh -> Thoát vòng lặp chờ
                if "trafficlight_green" in fresh_objects:
                    print(" Đã thấy đèn XANH! Go go go!")
                    break
                
                # (Tùy chọn) Timeout an toàn: Nếu chờ quá 30s mà không thấy xanh thì cứ đi (phòng khi đèn hỏng)
                if time.time() - start_wait > 30:
                    print(" Chờ quá lâu (30s)! Tự động đi tiếp.")
                    break
                
                time.sleep(0.1) # Kiểm tra 10 lần/giây

            # B3: Đèn xanh rồi -> Rẽ trái (như yêu cầu)
            print(" Đang rẽ trái (Traffic Light)...")
            send_command("steer", STEER_RIGHT)
            send_command("speed", SPEED_TURN)
            time.sleep(TIME_TURN)

            # B4: Trả lái
            print(" Trả lái thẳng!")
            send_command("steer", STEER_STRAIGHT)
            time.sleep(0.5)

            cooldown_timer = time.time() + COOLDOWN_TIME
            STATE = "NORMAL"

        # >>> XỬ LÝ NO ENTRY
        elif STATE == "MANEUVER_NO_ENTRY":
            send_command("speed", 0)
            print(" Dừng xe (No Entry)...")
            time.sleep(2)

            print(" Lùi xe...")
            send_command("steer", STEER_STRAIGHT)
            send_command("speed", SPEED_REVERSE)
            time.sleep(TIME_REVERSE)

            print(" Rẽ phải để né...")
            send_command("steer", STEER_RIGHT)  
            send_command("speed", SPEED_TURN)
            time.sleep(TIME_TURN)
            
            print(" Trả lái thẳng!")
            send_command("steer", STEER_STRAIGHT)
            time.sleep(0.5)

            cooldown_timer = time.time() + COOLDOWN_TIME
            STATE = "NORMAL"

        # >>> XỬ LÝ STOP / CROSSWALK
        elif STATE == "MANEUVER_STOP":
            send_command("speed", 0)
            print(" Dừng xe (Stop/Crosswalk)...")
            time.sleep(TIME_STOP_WAIT)

            print(" Rẽ trái...")
            send_command("steer", STEER_RIGHT)
            send_command("speed", SPEED_TURN)
            time.sleep(TIME_TURN)

            print(" Trả lái thẳng!")
            send_command("steer", STEER_STRAIGHT) 
            time.sleep(0.5)

            cooldown_timer = time.time() + COOLDOWN_TIME
            STATE = "NORMAL"

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n Dừng khẩn cấp!")
    send_command("speed", 0)
    send_command("brake", 0)
    eye.running = False

finally:
    ser.close()