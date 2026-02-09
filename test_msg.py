import time
import multiprocessing
import sys
import os

# 1. Đoạn này cực kỳ quan trọng: giúp Python tìm thấy thư mục 'src' của Bosch
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 2. Import các công cụ gửi tin nhắn từ chính dự án của em
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import ObstacleDetection

def run_test():
    # Khởi tạo danh sách Queue giống như bộ não thật vận hành
    queueList = {"General": multiprocessing.Queue()}

    # Khởi tạo bộ gửi cho ObstacleDetection (cái msgID = 6 mà em vừa thêm)
    sender = messageHandlerSender(queueList, ObstacleDetection)

    print("--- Đang bắt đầu gửi dữ liệu giả lập lên Dashboard ---")
    print("Mẹo: Hãy mở tab 'JSON Table' trên trình duyệt để xem kết quả")

    try:
        while True:
            # Gửi giá trị True (Báo có vật cản)
            print("Đang gửi: True")
            sender.send(True)
            time.sleep(2) # Đợi 2 giây

            # Gửi giá trị False (Báo đường trống)
            print("Đang gửi: False")
            sender.send(False)
            time.sleep(2) # Đợi 2 giây
            
    except KeyboardInterrupt:
        print("\nĐã dừng chạy thử nghiệm.")

if __name__ == "__main__":
    run_test()