[README.md](https://github.com/user-attachments/files/28204862/README.md)
# Tài Liệu Giới Thiệu Dự Án: App-CCTV-AI - Hệ Thống Giám Sát An Toàn Lao Động

## 📋 Tổng Quan Dự Án

**App-CCTV-AI** là một hệ thống giám sát thời gian thực dựa trên AI (Artificial Intelligence) và Computer Vision, được thiết kế để:
- Phát hiện các vi phạm an toàn lao động
- Giám sát hành vi và tuân thủ an toàn tại các khu vực làm việc
- Ghi nhận sự kiện bất thường và cảnh báo qua API
- Theo dõi và lưu trữ video xử lý

Hệ thống tích hợp **PyQt6 GUI**, **YOLOv10 Object Detection**, và **Centroid Tracking** để cung cấp giải pháp giám sát toàn diện.

---

## 🏗️ Cấu Trúc Dự Án

```
App-CCTV-AI/
├── Application.py              # Ứng dụng GUI chính (PyQt6)
├── check.py                    # Module hỗ trợ kiểm tra
├── CameraCaptureWorker.py      # Thread capture video từ camera
├── CentroidTracker.py          # Theo dõi đối tượng bằng centroid
├── App-CCTV-AI.bat             # Script chạy ứng dụng
│
├── SSGLogic/                   # Module cho Camera 1 (Logic Area)
│   ├── cam1.py                 # Khởi tạo và cấu hình camera 1
│   ├── processing_Video.py     # Xử lý video chính (YOLO + Detection)
│   ├── CameraCaptureWorker.py  # Worker capture cho camera 1
│   ├── CentroidTracker.py      # Tracker cho camera 1
│   ├── CentroidTrackerHistory.py # Tracker có lịch sử cho camera 1
│   ├── config.yaml             # File cấu hình (tham số detection, ROI)
│   ├── ModelAll/               # Model YOLO chính
│   └── ModelKeo/               # Model YOLO chuyên biệt (Scissor detection)
│
├── CBInside/                   # Module cho Camera 2 (Ngoài - Inside)
│   ├── cam2.py
│   ├── processing_Video.py
│   ├── CameraCaptureWorker.py
│   ├── CentroidTracker.py
│   ├── config.yaml
│   └── ModelAll/
│
└── CBOutside/                  # Module cho Camera 3 (Ngoài - Outside)
    ├── cam3.py
    ├── processing_Video.py
    ├── CameraCaptureWorker.py
    ├── CentroidTracker.py
    ├── CentroidTrackerHistory.py
    ├── config.yaml
    └── ModelAll/
```

---

## 🎯 Các Chức Năng Chính

### 1. **Capture Video từ Camera (CameraCaptureWorker.py)**

**Lớp: `FreshestFrame`** - Thread-based video capture

```python
class FreshestFrame(threading.Thread):
```

**Chức năng:**
- Chạy trên một thread riêng để capture frame video liên tục
- Luôn giữ frame mới nhất (freshest frame) để giảm delay
- Sử dụng thread synchronization (Condition) để thread-safe

**Phương thức chính:**
- `__init__()` - Khởi tạo capture device và thread
- `run()` - Vòng lặp capture video liên tục
- `release()` - Dừng capture và giải phóng resource
- `read()` - Đọc frame mới nhất (hỗ trợ wait blocking hoặc polling)

**Tham số:**
- `sleep_interval=0.04` - Thời gian chờ giữa các frame (giảm CPU)
- Hỗ trợ callback khi có frame mới

---

### 2. **Xử Lý Video & Phát Hiện Vi Phạm (processing_Video.py)**

**Lớp: `SSGVision`** - Core vision processing engine

**Chức năng chính:**

#### **A. Khởi tạo và Cấu hình**
- Load file `config.yaml` chứa tham số detection
- Khởi tạo model YOLO (YOLOv10 / YOLO models)
- Thiết lập các bounding box (ROI) cho các vùng cần kiểm tra

**Các hằng số phát hiện:**
```python
GLOVE_CONSTANT = 0          # Phát hiện đeo găng
NOGLOVE_CONSTANT = 1        # Không đeo găng (vi phạm)
HAND_CONSTANT = 2           # Phát hiện tay
PEOPLE_CONSTANT = 3         # Phát hiện người
ROLL_CONSTANT = 4           # Phát hiện cuộn vật liệu
SCISSOR_LABEL = 3           # Phát hiện kéo
MIN_SCORE_CONSTANT = 0.5    # Ngưỡng confidence
```

#### **B. Các Loại Vi Phạm Được Phát Hiện**

1. **Không Đeo Găng (No Glove Detection)**
   - Phát hiện khi người dùng không đeo găng bảo vệ
   - Lưu ảnh xác nhận: `noglove_img_check`
   - Kiểm tra tối thiểu 10 frame liên tiếp

2. **Cuộn Vật Liệu Trên Sàn (Roll Detection)**
   - Phát hiện cuộn vật liệu nằm trên sàn
   - Kiểm tra trong vùng `accept_roll` từ config
   - Lưu ảnh: `roll_img_check`

3. **Tay Chạm Bảng Điều Khiển (Touch Panel)**
   - Phát hiện khi tay cảm ứng bảng điều khiển
   - Kiểm tra vùng `panel` từ config
   - Lưu ảnh: `touch_panel_img_check`

4. **Sử Dụng Kéo (Scissor Detection)**
   - Phát hiện khi người dùng sử dụng kéo
   - Kiểm tra trong vùng `scissorBox`
   - Model chuyên biệt: `ModelKeo`

5. **Chỉ Báo Chiếu Sáng (Light Indicator Check)**
   - Giám sát đèn chỉ báo (đỏ, vàng, xanh)
   - Kiểm tra từ vùng `lightBox` trong config
   - Phát hiện cờ (flashing) đèn

6. **Tập Hợp Nhân Sự (People Gathering)**
   - Phát hiện khi có quá nhiều người tập hợp gần nhau
   - Khoảng cách gần: `CLOSE_PEOPLE = 700` pixel
   - Thời gian giám sát: `TIME_CLOSE_PEOPLE = 120` giây
   - Số người kiểm tra: `NUM_PEOPLE_CHECK = 3`

#### **C. Các Công Cụ Theo Dõi**

- **`CentroidTrackerHistory`** - Tracker người dùng (lịch sử 30 frame)
- **`CentroidTracker`** - Tracker tay (10 frame)
- **`RollTracker`** - Tracker cuộn vật liệu (15 frame)

---

### 3. **Theo Dõi Đối Tượng (CentroidTracker.py)**

**Lớp: `CentroidTracker`**

**Chức năng:**
- Theo dõi chuyển động của các đối tượng qua các frame
- Dùng centroid (tâm) của bounding box để match đối tượng
- Lưu giữ ID các đối tượng khi chúng di chuyển

**Phương thức:**
- `update()` - Cập nhật vị trí đối tượng từ frame mới
- `register()` - Đăng ký đối tượng mới
- `deregister()` - Xóa đối tượng khỏi tracking

---

### 4. **Giao Diện Người Dùng (Application.py)**

**Lớp: `CameraWorker`** - Worker xử lý camera trong GUI

**Chức năng:**
- Quản lý 3 camera qua PyQt6 GUI
- Hiển thị live video feed từ các camera
- Điều khiển start/stop camera
- Cấu hình tham số camera
- Hiển thị log và thông báo sự kiện

**Các Component GUI:**
- `QGraphicsView` - Hiển thị video frame
- `QTableView` - Hiển thị event log
- `QStatusBar` - Thanh trạng thái
- `QLabel` - Hiển thị thông tin camera

**Kết nối Camera:**
- Lấy cấu hình từ API `CAMERA_SETTING`
- Cấu hình thiết bị thông qua Modbus TCP

---

## ⚙️ File Cấu Hình (config.yaml)

Mỗi camera có file `config.yaml` riêng chứa:

```yaml
general:
  source: "//192.168.10.200/..."       # URL camera hoặc video file
  autoRestart: true                    # Khởi động lại tự động khi lỗi
  ThreadCap: False                     # Sử dụng thread capture
  MaxRetry: 10                         # Số lần retry tối đa
  device: 0                            # GPU device ID (0=GPU, CPU=other)

model_All:
  weights: "SSGLogic/ModelAll/..."     # Đường dẫn model YOLO chính
  weights_Keo: "SSGLogic/ModelKeo/..." # Model chuyên biệt cho kéo
  imgsz: 640                           # Kích thước input model
  conf: 0.5                            # Confidence threshold
  iou: 0.5                             # IOU threshold

# Vùng phát hiện (ROI - Region of Interest)
accept_roll:                           # Vùng kiểm tra cuộn vật liệu
  Count: 13
  point1-13: (x, y)                    # Tọa độ các điểm đa giác

Scissor_warning:                       # Vùng kiểm tra kéo
  Count: 7
  point1-7: (x, y)

lightBox:                              # Vùng kiểm tra đèn chỉ báo
  thresh: 30
  RectRed: (x1, y1, x2, y2)
  RectYellow: (x1, y1, x2, y2)
  RectGreen: (x1, y1, x2, y2)

cabin:                                 # Vùng cabin
  Count: 7
  point1-7: (x, y)

panel:                                 # Bảng điều khiển
  Count: ...
  point1-N: (x, y)
```

---

## 🔌 Tích Hợp API

Hệ thống tích hợp API để:
- **Gửi dữ liệu bất thường**: POST sự kiện vi phạm
- **Lấy cấu hình camera**: GET cài đặt camera từ server
- **Gửi hình ảnh**: Upload ảnh vi phạm lên server

**Endpoint API:**
```yaml
api:
  API: "http://api.server.com/"
  CAMERA_SETTING: "camera/setting/"
  ABNORMAL: "abnormal/report/"
  MEDIA: "media/upload/"
```

---

## 📊 Luồng Xử Lý Chính

```
1. CAPTURE FRAME
   ↓
   Camera → FreshestFrame (thread) → Frame Queue
   
2. YOLO DETECTION
   ↓
   Frame → YOLOv10 Model → Bounding Boxes + Classes + Confidence
   
3. TRACKING
   ↓
   Bounding Boxes → CentroidTracker → Object IDs + Positions
   
4. VIOLATION CHECK
   ↓
   Object IDs + Classes + Positions → Violation Logic → Detected Violations
   
5. API & LOGGING
   ↓
   Violations → API POST → Server Log + Alert
   Violations → Logger → Local Log File
   
6. GUI DISPLAY
   ↓
   Frame + Detections → GUI → Live Display + Event List
```

---

## 🛠️ Quy Trình Chạy Ứng Dụng

### **1. Khởi động**
```bash
# Chạy từ batch file
App-CCTV-AI.bat

# Hoặc chạy Python trực tiếp
python Application.py
```

### **2. Khởi tạo**
- Load 3 camera config từ `config.yaml`
- Khởi tạo YOLO models
- Kết nối với camera sources
- Khởi động GUI PyQt6

### **3. Hoạt động**
- Thread capture liên tục grab frame
- Xử lý YOLO detection trên mỗi frame
- Tracking đối tượng giữa các frame
- Kiểm tra vi phạm
- Gửi cảnh báo qua API (nếu cần)
- Ghi log sự kiện
- Hiển thị GUI với live video

### **4. Dừng**
- Stop capture threads
- Close camera connections
- Release YOLO models (GPU memory)
- Save logs

---

## 📝 Logging & Monitoring

**Hệ thống Logging:**
- File log hàng ngày: `log/{YYYY-MM-DD}.log`
- Log rotation: Tự động tạo file mới mỗi ngày lúc nửa đêm
- Giữ lại log: 10 ngày

**Nội dung Log:**
- Thời gian capture
- Kết quả detection (class, confidence)
- Sự kiện vi phạm phát hiện
- Thông báo API response
- Lỗi exception

---

## 🎨 Màu Sắc Hiển Thị (config.yaml)

```python
NG: (0, 0, 255)              # Đỏ - Vi phạm
OK: (0, 255, 0)              # Xanh - Bình thường
OBJECT: (255, 0, 0)          # Xanh dương - Đối tượng
CABIN: (0, 255, 255)         # Vàng - Vùng cabin
PANEL: (0, 255, 0)           # Xanh - Bảng điều khiển
LIGHTRED: (0, 0, 255)        # Đỏ - Đèn đỏ
LIGHTYELLOW: (0, 255, 255)   # Vàng - Đèn vàng
LIGHTGREEN: (0, 255, 0)      # Xanh - Đèn xanh
FLOOR: (42, 42, 128)         # Nâu - Sàn
SCISSORCHECK: (255, 0, 255)  # Tím - Kéo
GATHER: (255, 255, 255)      # Trắng - Tập hợp người
```

---

## 📦 Dependencies

```
PyQt6                 # GUI framework
OpenCV (cv2)          # Video processing
PyTorch               # Deep learning
Shapely              # Polygon geometry
YOLOv10 / YOLO       # Object detection
ultralytics          # YOLO integration
requests             # HTTP requests
PyYAML               # YAML config
python-dateutil      # Date parsing
PyModbus             # Modbus TCP communication
```

---

## 🔄 Cấu Trúc Thư Mục Module

Mỗi camera module (SSGLogic, CBInside, CBOutside) có cấu trúc giống nhau:

```
CameraX/
├── cam{N}.py                      # Lớp chính SSGVision - khởi tạo
├── processing_Video.py            # Xử lý video, detection, violation check
├── CameraCaptureWorker.py         # FreshestFrame - thread capture
├── CentroidTracker.py             # Basic tracker
├── CentroidTrackerHistory.py      # Tracker với lịch sử (nếu có)
├── config.yaml                    # Cấu hình detection & ROI
├── ModelAll/                      # Thư mục chứa model YOLO chính
│   └── All_V8_{date}_best.pt      # Model weights
└── ModelKeo/ (optional)           # Model chuyên biệt (ví dụ: scissor detection)
    └── All_v8_{date}_best.pt      # Model weights
```

---

## 🚀 Các Tính Năng Nâng Cao

### **1. Multi-Camera Support**
- Xử lý 3 camera độc lập cùng lúc
- Mỗi camera chạy logic riêng

### **2. Threading & Performance**
- Capture video trên thread riêng
- GUI không bị block khi xử lý video
- Tuning `sleep_interval` để giảm CPU

### **3. Polygon-based ROI**
- Dùng Shapely Point-in-Polygon để check nếu object nằm trong vùng
- Hỗ trợ đa giác phức tạp (không chỉ hình chữ nhật)

### **4. Centroid Tracking**
- Theo dõi object giữa các frame bằng centroid matching
- Lưu ID để phát hiện vi phạm liên tục

### **5. Auto-Restart**
- Nếu camera disconnect, tự động restart connection
- Config: `autoRestart: true`

### **6. API Integration**
- Gửi violation events tới server
- Lấy config camera từ API
- Upload ảnh chứng cứ

---

## 📞 Contacts & Notes

- **Project Type**: Real-time Safety Monitoring System
- **AI Framework**: YOLOv10 / YOLO Object Detection
- **GUI**: PyQt6
- **Status**: In Development/Production

---

**Tài liệu này được tạo để giúp các nhà phát triển và người dùng hiểu rõ cấu trúc, chức năng và hoạt động của hệ thống App-CCTV-AI.**
