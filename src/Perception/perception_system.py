import os
import cv2
import subprocess as sp
import numpy as np
import onnxruntime as ort
import threading
import time

class BFMC_Vision:
    def __init__(self):
        # --- CẤU HÌNH ---
        self.WIDTH, self.HEIGHT = 640, 480
        self.FPS = 25
        os.environ["DISPLAY"] = ":0"
        self.running = False
        self.latest_objects = [] 
        
        # --- NẠP MODEL ---
        print("🧠 Đang nạp bộ não YOLOv8 + NMS...")
        self.session = ort.InferenceSession("yolov8n_BFMC_V2.onnx", providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.names = ['car', 'crosswalk', 'highway_entry', 'highway_exit', 'intersection', 
                      'no_entry', 'onewayroad', 'parking', 'pedestrian', 'priority', 
                      'roundabout', 'stop', 'trafficlight_green', 'trafficlight_red', 'trafficlight_yellow']

    def preprocess(self, img_bgr):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_res = cv2.resize(img_rgb, (640, 640))
        img_input = img_res.astype(np.float32) / 255.0
        img_input = img_input.transpose(2, 0, 1)
        img_input = np.expand_dims(img_input, axis=0)
        return img_input

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        print("📷 Vision System (NMS Activated) đã khởi động!")

    def update(self):
        cmd = [
            "rpicam-vid", "--inline", "--nopreview", "--width", str(self.WIDTH), 
            "--height", str(self.HEIGHT), "--framerate", str(self.FPS), "--codec", "yuv420", 
            "--timeout", "0", "-o", "-"
        ]
        process = sp.Popen(cmd, stdout=sp.PIPE, bufsize=self.WIDTH*self.HEIGHT*2)
        frame_size = int(self.WIDTH * self.HEIGHT * 1.5)

        while self.running:
            try:
                raw_data = process.stdout.read(frame_size)
                if len(raw_data) != frame_size: continue
                
                yuv_image = np.frombuffer(raw_data, dtype=np.uint8).reshape((int(self.HEIGHT * 1.5), self.WIDTH))
                frame_bgr = cv2.cvtColor(yuv_image, cv2.COLOR_YUV420p2RGB)

                # --- AI DETECT ---
                blob = self.preprocess(frame_bgr)
                outputs = self.session.run(None, {self.input_name: blob})
                output = outputs[0][0].transpose()

                # --- BƯỚC 1: GOM DỮ LIỆU THÔ ---
                boxes = []
                confidences = []
                class_ids = []
                
                # Tỷ lệ scale để vẽ lại lên ảnh gốc
                x_factor = self.WIDTH / 640
                y_factor = self.HEIGHT / 640

                for row in output:
                    confidence = row[4:].max()
                    
                    # Lọc sơ bộ: Chỉ lấy cái nào tin tưởng > 50%
                    if confidence > 0.5:
                        class_id = row[4:].argmax()
                        
                        # Lấy tọa độ (x_center, y_center, width, height) từ YOLO
                        xc, yc, w, h = row[0], row[1], row[2], row[3]
                        
                        # Chuyển về tọa độ góc trái trên (left, top, width, height)
                        left = int((xc - w/2) * x_factor)
                        top = int((yc - h/2) * y_factor)
                        width = int(w * x_factor)
                        height = int(h * y_factor)

                        boxes.append([left, top, width, height])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)

                # --- BƯỚC 2: ÁP DỤNG NMS (LOẠI BỎ HỘP TRÙNG) ---
                # score_threshold: Ngưỡng điểm (đã lọc ở trên rồi nhưng khai báo lại cho chắc)
                # nms_threshold: 0.4 (Nếu 2 hộp chồng lên nhau quá 40% diện tích -> Xóa hộp điểm thấp hơn)
                indices = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=0.5, nms_threshold=0.4)

                # --- BƯỚC 3: XUẤT KẾT QUẢ CUỐI CÙNG ---
                final_objects = []
                
                if len(indices) > 0:
                    for i in indices.flatten():
                        # Lấy thông tin từ index đã lọc
                        x, y, w, h = boxes[i]
                        label = self.names[class_ids[i]]
                        conf = confidences[i]
                        
                        final_objects.append(label)

                        # Vẽ khung hình chữ nhật chuẩn
                        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        text = f"{label}: {conf:.2f}"
                        cv2.putText(frame_bgr, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.latest_objects = final_objects 

                cv2.imshow("Vision View (NMS)", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
            
            except Exception as e:
                print(f"Lỗi Vision: {e}")

        process.terminate()
        cv2.destroyAllWindows()

    def get_objects(self):
        return self.latest_objects