from urllib import response
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt, QTimer, QStringListModel
from PyQt6.QtWidgets import QGraphicsScene, QGraphicsPixmapItem, QStatusBar, QLabel, QTableWidgetItem
from PyQt6.QtGui import QImage, QPixmap
import cv2
from CBInside import cam2
from CBOutside import cam3
from SSGLogic import cam1
import time
import yaml
import threading
import os
import logging
import datetime as dt   
import requests
from pymodbus.client import ModbusTcpClient
from datetime import datetime, date
import sys
import requests
import re
import configparser
import json
from logging.handlers import TimedRotatingFileHandler
from dateutil import parser  
cbinside = cam2.SSGVision(config_path="CBInside/config.yaml")
cboutside = cam3.SSGVision(config_path="CBOutside/config.yaml")
logic = cam1.SSGVision(config_path="SSGLogic/config.yaml")

# def setup_daily_logger(log_folder, logger_name):
#     """Cấu hình logger theo ngày với xử lý lỗi."""
#     try:
#         # Tạo thư mục log nếu chưa tồn tại
#         log_dir = os.path.join(log_folder, logger_name)
#         if not os.path.exists(log_dir):
#             os.makedirs(log_dir)

#         # Tạo file log theo ngày
#         log_file = os.path.join(log_dir, f"{date.today().strftime('%Y-%m-%d')}.log")

#         # Lấy hoặc khởi tạo logger
#         logger = logging.getLogger(logger_name)
#         if not logger.handlers:  # Đảm bảo không thêm handler lặp lại
#             handler = logging.FileHandler(log_file, mode='a')
#             formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
#             handler.setFormatter(formatter)
#             logger.addHandler(handler)
#             logger.setLevel(logging.INFO)
#             logger.info("Logger initialized successfully")

#         # Đảm bảo logger ghi vào đúng file theo ngày
#         current_log_file = logger.handlers[0].baseFilename
#         if current_log_file != log_file:
#             # Xóa handler cũ và thêm handler mới
#             logger.handlers = []
#             handler = logging.FileHandler(log_file, mode='a')
#             formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
#             handler.setFormatter(formatter)
#             logger.addHandler(handler)
#             logger.info("Logger rotated to new date")

#         return logger

#     except PermissionError as e:
#         print(f"Permission error while setting up logger: {e}")
#     except Exception as e:
#         print(f"Error setting up logger: {e}")

class CameraWorker(QObject):
    frameCaptured = pyqtSignal(object)  # Emit frame data

    def __init__(self, camera_index=0, log_folder="logs", logger_name="CameraWorker"):
        super().__init__()
        self.camera_index = camera_index
        self.running = False

        self.main_ui = Ui_MainWindow()
        self.chooseCam_ = None
        self.stopCam_ = None

        self.camSettting_ = None
        # self.iconRun = None
        # self.iconStop = None

        self.iconRun = QtGui.QIcon("images/run.png")
        self.iconStop = QtGui.QIcon("images/stop.png")

        self.Cam = None
        self.baseDir=  os.path.abspath(os.path.dirname(sys.argv[0]))
        if not os.path.exists(self.baseDir+"/log"):
            os.makedirs(self.baseDir+"/log")
        self.configure_logging()
        # Khởi tạo logger riêng cho CameraWorker
        # self.logger = setup_daily_logger(log_folder, logger_name)
    def configure_logging(self):
        logger = logging.getLogger()
        # Remove all handlers associated with the logger object
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)

        # Set logging file with TimedRotatingFileHandler
        sLog = self.baseDir + '/log/' + str(dt.datetime.now().date()) + '.log'
        logHandler = TimedRotatingFileHandler(sLog, when="midnight", interval=1)
        logHandler.suffix = "%Y-%m-%d.log"
        logHandler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
        logger.setLevel(logging.DEBUG)
        logger.addHandler(logHandler)
        #self.CheckDeleteLog(self.baseDir)

    def CheckDeleteLog(self, baseDir):  
        try:
            mypath = baseDir+'/log/'
            nDayStore= 10
            t2= (dt.datetime.now()  + dt.timedelta(days=1)).date()
            t1= (dt.datetime.now()  - dt.timedelta(days=nDayStore)).date()
            for root, dirs, files in os.walk(mypath):
                for file in files: 
                    if file.endswith(".log"): #list all log file in log folder
                        #print(file)
                        sFileName = os.path.basename(file).split('.')[0] #get base filename
                        try:
                            t3 = parser.parse(sFileName).date()
                            if t1 < t3 and t3 < t2:
                                print('keep:' + file)
                            else:
                                print('delete:' + file)
                                logging.info('delete:' + file)
                                os.remove(mypath+ file)#delte file
                        except:
                            logging.exception('Got exception when delete' + file)
                            os.remove(mypath+ file)
                    else:
                        os.remove(mypath+ file)
        except:
                logging.exception('Got exception')  

    def set_chooseCam(self, chooseCam):
        self.chooseCam_ = chooseCam
    def set_stopCam(self, stopCam):
        self.stopCam_ = stopCam
    def set_icon(self, Cam):
        self.Cam = Cam
        self.Cam.setIcon(self.iconRun)

    def setModel(self, cam, config_path, camip):
        self.cam = cam
        if cam == "F1-COP1-05":
            self.CAM1_config_path = config_path
            # self.CAM1_baseDir=  os.path.abspath(os.path.dirname(sys.argv[0]))
            # if not os.path.exists(self.CAM1_baseDir+"/log"):
            #     os.makedirs(self.CAM1_baseDir+"/log")
            # #set logging file
            # sLog = self.CAM1_baseDir+'/log/'+str(dt.datetime.now().date())+'.log'
            # logging.basicConfig(filename=sLog,level=logging.DEBUG,format='%(asctime)s %(message)s')
            # Read parameter from config path
            with open(self.CAM1_config_path, "r", encoding="utf8") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM1_general_config = config["general"]
            self.CAM1_model_all_config = config["model_All"]
            self.CAM1_api = config["api"]
            self.CAM1_apicamsetting = self.CAM1_api["API"] + self.CAM1_api["CAMERA_SETTING"] + camip
            self.CAM1_post = self.CAM1_api["POST"]
            if self.CAM1_post:
                self.CAM1_data_cam = logic.readCam_api(self.CAM1_apicamsetting)
                
                if self.CAM1_data_cam is not None:
                    logic.set_params(self.CAM1_data_cam)
            # Set up Box of Cabin, Panel, Floor and Light indicator position
            logic.set_coordinates()
            # Load model detect
            self.CAM1_model_All = logic.load_modelv10(self.CAM1_model_all_config["weights"])
            self.CAM1_model_keo = logic.load_modelv10(self.CAM1_model_all_config["weights_Keo"])
            # Set Color
            self.CAM1_colors = {
                "NG": eval(self.CAM1_model_all_config['colors']['NG']),
                "OK": eval(self.CAM1_model_all_config['colors']['OK']),
                "OBJECT": eval(self.CAM1_model_all_config["colors"]["OBJECT"]),
                "CABIN": eval(self.CAM1_model_all_config['colors']['CABIN']),
                "PANEL": eval(self.CAM1_model_all_config['colors']['PANEL']),
                "LIGHTRED": eval(self.CAM1_model_all_config['colors']['LIGHTRED']),
                "LIGHTYELLOW": eval(self.CAM1_model_all_config['colors']['LIGHTYELLOW']),
                "LIGHTGREEN": eval(self.CAM1_model_all_config['colors']['LIGHTGREEN']),
                "LIGHTALL": eval(self.CAM1_model_all_config['colors']['LIGHTALL']),
                "FLOOR": eval(self.CAM1_model_all_config['colors']['FLOOR']),
                "SCISSORCHECK": eval(self.CAM1_model_all_config['colors']['SCISSORCHECK']),
                "GATHER": eval(self.CAM1_model_all_config['colors']['GATHER']),
            }
            # logic.CheckDeleteLog(self.CAM1_baseDir)

        elif cam == "F1-COP1-S1":
            self.CAM2_config_path = config_path
            # self.CAM2_baseDir=  os.path.abspath(os.path.dirname(sys.argv[0]))
            # if not os.path.exists(self.CAM2_baseDir+"/log"):
            #     os.makedirs(self.CAM2_baseDir+"/log")
            # #set logging file
            # sLog = self.CAM2_baseDir+'/log/'+str(dt.datetime.now().date())+'.log'
            # logging.basicConfig(filename=sLog,level=logging.DEBUG,format='%(asctime)s %(message)s')        
            # Read parameter from config path
            with open(config_path, "r", encoding="utf8") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM2_general_config = config["general"]
            self.CAM2_model_all_config = config["model_All"]
            self.CAM2_api = config["api"]
            self.CAM2_apicamsetting = self.CAM2_api["API"] + self.CAM2_api["CAMERA_SETTING"] + camip
            self.CAM2_post = self.CAM2_api["POST"]
            if self.CAM2_post:
                self.CAM2_data_cam = cbinside.readCam_api(self.CAM2_apicamsetting)
                if self.CAM2_data_cam is not None:
                    cbinside.set_params(self.CAM2_data_cam)
            # Set up Box of Panel position
            cbinside.set_coordinates()
            # Load model detect
            self.CAM2_model_All = cbinside.load_modelv10(self.CAM2_model_all_config["weights"])
            # Set Color
            self.CAM2_colors = {
                "NG": eval(self.CAM2_model_all_config['colors']['NG']),
                "OK": eval(self.CAM2_model_all_config['colors']['OK']),
                "OBJECT": eval(self.CAM2_model_all_config["colors"]["OBJECT"]),
                "PANEL": eval(self.CAM2_model_all_config['colors']['PANEL']),
                "FLOOR": eval(self.CAM2_model_all_config['colors']['FLOOR']),
                "GLOVECHECK": eval(self.CAM2_model_all_config['colors']['GLOVECHECK']),
                "SCISSORCHECK": eval(self.CAM2_model_all_config['colors']['SCISSORCHECK']),
                "LIGHTRED": eval(self.CAM2_model_all_config['colors']['LIGHTRED']),
                "LIGHTYELLOW": eval(self.CAM2_model_all_config['colors']['LIGHTYELLOW']),
                "LIGHTGREEN": eval(self.CAM2_model_all_config['colors']['LIGHTGREEN']),
                "LIGHTALL": eval(self.CAM2_model_all_config['colors']['LIGHTALL']),
            }
            # cbinside.CheckDeleteLog(self.CAM2_baseDir)

        elif cam == "F1-COP1-04":
            self.CAM3_config_path = config_path
            # self.CAM3_baseDir=  os.path.abspath(os.path.dirname(sys.argv[0]))
            # if not os.path.exists(self.CAM3_baseDir+"/log"):
            #     os.makedirs(self.CAM3_baseDir+"/log")
            # #set logging file
            # sLog = self.CAM3_baseDir+'/log/'+str(dt.datetime.now().date())+'.log'
            # logging.basicConfig(filename=sLog,level=logging.DEBUG,format='%(asctime)s %(message)s')
            # Read parameter from config path
            with open(self.CAM3_config_path, "r", encoding="utf8") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM3_general_config = config["general"]
            self.CAM3_model_all_config = config["model_All"]
            self.CAM3_api = config["api"]
            self.CAM3_apicamsetting = self.CAM3_api["API"] + self.CAM3_api["CAMERA_SETTING"] + camip
            self.CAM3_post = self.CAM3_api["POST"]
            if self.CAM3_post:
                self.CAM3_data_cam = cboutside.readCam_api(self.CAM3_apicamsetting)
                if self.CAM3_data_cam is not None:
                    cboutside.set_params(self.CAM3_data_cam)

            cboutside.set_coordinates()
            # Load model detect
            self.CAM3_model_All = cboutside.load_modelv10(self.CAM3_model_all_config["weights"])
            # Set Color
            self.CAM3_colors = {
                "NG": eval(self.CAM3_model_all_config['colors']['NG']),
                "OK": eval(self.CAM3_model_all_config['colors']['OK']),
                "OBJECT": eval(self.CAM3_model_all_config["colors"]["OBJECT"]),
                "FLOOR": eval(self.CAM3_model_all_config['colors']['FLOOR']),
                "GATHER": eval(self.CAM3_model_all_config['colors']['GATHER'])
            }
            # cboutside.CheckDeleteLog(self.CAM3_baseDir)
        else:
            self.camName = None
            while self.camName is None:
                break

    def connect_camera(self):
        cap = cv2.VideoCapture(self.camera_index)
        if cap.isOpened():
            print("Reconnected the camera...")
            return cap
        else:
            time.sleep(2)
            return None   
         
    def run(self):
        self.chooseCam_ = self.main_ui.chooseCam
        self.stopCam_ = self.main_ui.stopCam
    
        try:
            api_updatestatus = f'http://192.168.96.30:9000/api/UpdateStatus?CamName={self.cam}&Status=1'
            requests.post(api_updatestatus)

            api_updatestatus = None
            self.running = True
            Reconnect = 0
            cap = self.connect_camera()
            status_posted = False

            while self.running:
                self.configure_logging()
                if cap is not None:
                    ret, frame = cap.read()
                else:
                    ret = False
                if not ret:
                    status_posted = False
                    if Reconnect < 500:
                        self.Cam.setIcon(self.iconStop)
                        Reconnect += 1
                        if Reconnect == 1:
                            api_updatestatus = f'http://10.212.10.234:81/api/UpdateStatus?CamName={self.cam}&Status=0'
                            requests.post(api_updatestatus)
                        logging.exception('Camera failed to connect for {} time'.format(Reconnect))
                        cap = self.connect_camera()
                        continue
                    else:
                        self.stop()
                        print("STOP!!")
                else:
                    if not status_posted:
                        api_updatestatus = f'http://10.212.10.234:81/api/UpdateStatus?CamName={self.cam}&Status=1'
                        requests.post(api_updatestatus)
                        status_posted = True

                    import numpy as np
                            
                    frame_copy = frame.copy()
                    self.Cam.setIcon(self.iconRun)
                    if self.cam == "F1-COP1-05":
                        logic.draw_rectangles(frame_copy, self.CAM1_colors)
                        results = self.CAM1_model_All(source=frame, conf=self.CAM1_model_all_config["conf"])  
                        frame = logic.process_results(results, frame_copy, frame, self.CAM1_colors, self.CAM1_model_keo, cap)
                        if self.chooseCam_ == 1:
                            if self.stopCam_ == 1:
                                continue
                            else:
                                self.frameCaptured.emit(frame)

                    if self.cam == "F1-COP1-S1":
                        cbinside.draw_rectangles(frame_copy, self.CAM2_colors)
                        results = self.CAM2_model_All(source=frame, conf=self.CAM2_model_all_config["conf"])  
                        frame = cbinside.process_results(results, frame_copy, frame, self.CAM2_colors, cap)
                        if self.chooseCam_ == 2:
                            if self.stopCam_ == 2:
                                continue
                            else:
                                self.frameCaptured.emit(frame)
                    
                    if self.cam == "F1-COP1-04":
                        cboutside.draw_rectangles(frame_copy, self.CAM3_colors)
                        results = self.CAM3_model_All(source=frame, conf=self.CAM3_model_all_config["conf"])  
                        frame = cboutside.process_results(results, frame_copy, frame, self.CAM3_colors, cap)
                        if self.chooseCam_ == 3:
                            if self.stopCam_ == 3:
                                continue
                            else:
                                self.frameCaptured.emit(frame)
                    # self.frameCaptured.emit(frame)
      
            if cap is not None:
                cap.release()
            # cv2.destroyAllWindows() 
        except Exception as e:
            logging.error('Error at %s', 'Processing', exc_info=e)
    def stop(self):
        self.running = False

class PLC_Advantech(QObject):
    statusUpdated = pyqtSignal(str)  # Tín hiệu gửi trạng thái cho giao diện

    def __init__(self, log_folder="logs"):
        super().__init__()
        self.client = None
        self.connected = False
        self.log_folder = log_folder
        self.current_date = date.today()
        self.logger = self.setup_daily_logger()

    def setup_daily_logger(self):
        """Cấu hình logger theo ngày"""
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        log_file = os.path.join(self.log_folder, f"{datetime.now().strftime('%Y-%m-%d')}.log")
        logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            filemode='a'  # Append nếu file đã tồn tại
        )
        return logging.getLogger()

    def check_date(self):
        """Kiểm tra ngày, nếu khác ngày hiện tại thì tạo file log mới"""
        if date.today() != self.current_date:
            self.current_date = date.today()
            self.logger = self.setup_daily_logger()

    def connect(self, ip, port=5000):
        """Kết nối tới thiết bị Advantech"""
        self.check_date()
        try:
            self.logger.info(f"Attempting to connect to {ip}:{port}")
            self.client = ModbusTcpClient(host=ip, port=port)
            if self.client.connect():
                self.connected = True
                self.statusUpdated.emit(f"Connected to {ip}:{port}")
                self.logger.info(f"Connected to {ip}:{port}")
            else:
                self.statusUpdated.emit(f"Failed to connect to {ip}:{port}")
                self.logger.error(f"Failed to connect to {ip}:{port}")
        except Exception as e:
            self.statusUpdated.emit(f"Error connecting to {ip}:{port}: {e}")
            self.logger.exception(f"Error connecting to {ip}:{port}: {e}")

    def disconnect(self):
        """Ngắt kết nối thiết bị"""
        self.check_date()
        if self.client:
            self.client.close()
            self.connected = False
            self.statusUpdated.emit("Disconnected to PLC")
            self.logger.info("Disconnected to PLC")

    def read_DI(self, register, value):
        """Đọc trạng thái từ Discrete Input (DI)"""
        self.check_date()
        if self.client and self.connected:
            try:
                # Đọc Discrete Input (DI) từ địa chỉ register
                result = self.client.read_discrete_inputs(register, 1)  # Đọc 1 bit từ DI
                if result.isError():
                    self.statusUpdated.emit(f"Error reading DI at address {register}")
                    self.logger.error(f"Error reading DI at address {register}")
                    return None
                self.logger.info(f"Read value {result.bits[0]} from DI address {register}")
                value == result.bits[0]  # Trả về trạng thái của DI (True/False)
                self.logger.info(f"DI: {value}")
                return value
            except Exception as e:
                self.statusUpdated.emit(f"Error reading DI: {e}")
                self.logger.exception(f"Error reading DI: {e}")
                return None
        else:
            self.statusUpdated.emit("PLC Not Connected")
            return None

    def write_DO(self, register, value):
        """Ghi giá trị vào Digital Output (DO) và ghi nhận trạng thái"""
        self.check_date()
        if self.client and self.connected:
            try:
                # Ghi giá trị vào Coil (DO)
                result = self.client.write_coil(register, value)
                if result.isError():
                    self.statusUpdated.emit(f"Error writing {value} to DO at address {register}")
                    self.logger.error(f"Error writing {value} to DO at address {register}")
                    return None

                self.logger.info(f"Successfully written {value} to DO address {register}")
                self.statusUpdated.emit(f"Successfully written {value} to DO address {register}")
                return True
                # # Đọc lại giá trị từ DO sau khi ghi
                # read_result = self.client.read_coils(register, 1)
                # if read_result.isError():
                #     self.statusUpdated.emit(f"Error verifying DO at address {register}")
                #     self.logger.error(f"Error verifying DO at address {register}")
                #     return None

                # read_value = read_result.bits[0]
                # if read_value == value:
                #     self.statusUpdated.emit(f"Successfully written and verified {value} at DO address {register}")
                #     self.logger.info(f"Successfully written and verified {value} at DO address {register}")
                # else:
                #     self.statusUpdated.emit(f"Mismatch after writing to DO address {register}: Expected {value}, Got {read_value}")
                #     self.logger.warning(f"Mismatch after writing to DO address {register}: Expected {value}, Got {read_value}")
                # return read_value
            except Exception as e:
                self.statusUpdated.emit(f"Error writing to DO: {e}")
                self.logger.exception(f"Error writing to DO: {e}")
                return None
        else:
            self.statusUpdated.emit("PLC not connected")
            return None

class Ui_MainWindow(object):
    def __init__(self):
        self.threads = []
        self.model = QStringListModel()
        self.chooseCam = None
        self.stopCam = None
    
    def setupUi(self, MainWindow):
        response = requests.get('http://10.212.10.234:81/api/GetCamSetting')
        self.camSettting = response.json()

        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.WindowModality.ApplicationModal)
        MainWindow.resize(1600, 900)
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(12)
        MainWindow.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("images/favicon.ico"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setDockNestingEnabled(True)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # Khởi tạo PLC_Advantech
        self.plc = PLC_Advantech(log_folder=os.path.join(os.path.abspath(os.path.dirname(sys.argv[0])), "log"))
        self.plc.statusUpdated.connect(self.update_plc_status)

        # Layouts
        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.versionLayout = QtWidgets.QVBoxLayout()
        self.leftLayout = QtWidgets.QVBoxLayout()
        self.rightLayout = QtWidgets.QVBoxLayout()

        # Graphics View
        self.gvMain = QtWidgets.QGraphicsView(parent=self.centralwidget)
        self.gvMain.setObjectName("gvMain")
        self.rightLayout.addWidget(self.gvMain)

        # GroupBox 1
        self.groupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.groupBox.setFixedWidth(400)
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(12)
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")
    
        self.groupBoxLayout = QtWidgets.QVBoxLayout(self.groupBox)

        self.treeView = QtWidgets.QTreeView(parent=self.groupBox)
        self.treeView.setObjectName("treeView")
        self.treeView.header().hide()
        self.groupBoxLayout.addWidget(self.treeView)

        self.treeModel = QtGui.QStandardItemModel()

        self.F1 = QtGui.QStandardItem('F1')
        self.F5 = QtGui.QStandardItem('F5')
        
        self.F1.setEditable(False)
        self.F5.setEditable(False)

        self.F1_COP1_05 = QtGui.QStandardItem('F1-COP1-05')
        self.F1_COP1_S1 = QtGui.QStandardItem('F1-COP1-S1')
        self.F1_COP1_04 = QtGui.QStandardItem('F1-COP1-04')
        
        self.iconRun = QtGui.QIcon("images/run.png")
        self.iconStop = QtGui.QIcon("images/stop.png")
        
        self.F1_COP1_05.setEditable(False)
        self.F1_COP1_S1.setEditable(False)
        self.F1_COP1_04.setEditable(False)
        
        self.F1.appendRow(self.F1_COP1_05)
        self.F1.appendRow(self.F1_COP1_S1)
        self.F1.appendRow(self.F1_COP1_04)

        self.treeModel.appendRow(self.F1)
        self.treeModel.appendRow(self.F5)

        self.treeView.setModel(self.treeModel)

        self.leftLayout.addWidget(self.groupBox)

        # GroupBox 2
        self.groupBox_2 = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.groupBox_2.setFixedWidth(200)
        self.groupBox_2.setObjectName("groupBox_2")
        self.groupBox_2Layout = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.versionLayout.addWidget(self.groupBox_2)
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.groupBox_2)
        # self.plainTextEdit.setPlainText("Version 1.0.0 - 25/09/2024")
        self.plainTextEdit.setReadOnly(True)
        self.groupBox_2Layout.addWidget(self.plainTextEdit)
        self.groupBox_2.hide()
        self.tmpVersion = 0

        self.btnStop = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btnStop.setGeometry(QtCore.QRect(10, 30, 250, 40))
        self.btnStop.setObjectName("Stop")
        self.btnVersion = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btnVersion.setGeometry(QtCore.QRect(10, 30, 250, 40))
        self.btnVersion.setObjectName("Version")
        self.leftLayout.addWidget(self.btnStop)
        self.leftLayout.addWidget(self.btnVersion)

        # Add layouts to main layout
        self.mainLayout.addLayout(self.versionLayout)
        self.mainLayout.addLayout(self.leftLayout)
        self.mainLayout.addLayout(self.rightLayout)
        MainWindow.setCentralWidget(self.centralwidget)

        self.btnStop.clicked.connect(self.stop)
        self.btnVersion.clicked.connect(self.version)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        # Initialize QGraphicsScene and QGraphicsPixmapItem
        self.scene = QGraphicsScene()
        self.gvMain.setScene(self.scene)
        self.scenePixmapItem = None
        
        # Initialize CameraWorker and QThread
        self.thread_cam1 = QThread()
        self.cameraWorker_cam1 = CameraWorker()
        self.cameraWorker_cam1.moveToThread(self.thread_cam1)
        self.cameraWorker_cam1.frameCaptured.connect(self.processFrame)

        self.thread_cam2 = QThread()
        self.cameraWorker_cam2 = CameraWorker()
        self.cameraWorker_cam2.moveToThread(self.thread_cam2)
        self.cameraWorker_cam2.frameCaptured.connect(self.processFrame)

        self.thread_cam3 = QThread()
        self.cameraWorker_cam3 = CameraWorker()
        self.cameraWorker_cam3.moveToThread(self.thread_cam3)
        self.cameraWorker_cam3.frameCaptured.connect(self.processFrame)

        self.cameraWorker_cam1.set_chooseCam(self.chooseCam)
        self.cameraWorker_cam2.set_chooseCam(self.chooseCam)
        self.cameraWorker_cam3.set_chooseCam(self.chooseCam)

        self.cameraWorker_cam1.set_stopCam(self.stopCam)
        self.cameraWorker_cam2.set_stopCam(self.stopCam)
        self.cameraWorker_cam3.set_stopCam(self.stopCam)

        self.cameraWorker_cam1.set_icon(self.F1_COP1_05)

        self.cameraWorker_cam2.set_icon(self.F1_COP1_S1)
        self.cameraWorker_cam3.set_icon(self.F1_COP1_04)

        # Thêm các thành phần giao diện database
        self.dataNewGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.dataNewGroupBox.setTitle("Data Alarm New")
        self.dataNewGroupBox.setFixedHeight(100)
        self.dataNewlayout = QtWidgets.QVBoxLayout(self.dataNewGroupBox)

        # Tạo bảng cho groupbox datanew
        self.newtable = QtWidgets.QTableWidget(1, 6, parent=self.centralwidget)
        self.newtable.setColumnCount(6)
        self.newtable.setHorizontalHeaderLabels(["CamIP", "CamPort", "CamName", "LineID", "AbnormalType", "AbnormalDateTime"])
        self.newtable.setColumnWidth(0, 150)
        self.newtable.setColumnWidth(1, 70)
        self.newtable.setColumnWidth(2, 200)
        self.newtable.setColumnWidth(3, 70)
        self.newtable.setColumnWidth(4, 350)
        self.newtable.horizontalHeader().setStretchLastSection(True) # Để cột cuối cùng co giãn
        self.newtable.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers) # Không cho chỉnh sửa
        self.dataNewlayout.addWidget(self.newtable)

        self.rightLayout.addWidget(self.dataNewGroupBox)

        # Thêm các thành phần giao diện database
        self.dataOldGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.dataOldGroupBox.setTitle("Data Alarm Old")
        self.dataOldGroupBox.setFixedHeight(100)
        self.dataOldlayout = QtWidgets.QVBoxLayout(self.dataOldGroupBox)

        # Tạo bảng cho groupbox dataold
        self.oldtable = QtWidgets.QTableWidget(1, 6, parent=self.centralwidget)
        self.oldtable.setColumnCount(6)
        self.oldtable.setHorizontalHeaderLabels(["CamIP", "CamPort", "CamName", "LineID", "AbnormalType", "AbnormalDateTime"])
        self.oldtable.setColumnWidth(0, 150)
        self.oldtable.setColumnWidth(1, 70)
        self.oldtable.setColumnWidth(2, 200)
        self.oldtable.setColumnWidth(3, 70)
        self.oldtable.setColumnWidth(4, 350)
        self.oldtable.horizontalHeader().setStretchLastSection(True) # Để cột cuối cùng co giãn
        self.oldtable.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers) # Không cho chỉnh sửa
        self.dataOldlayout.addWidget(self.oldtable)

        self.rightLayout.addWidget(self.dataOldGroupBox)

        # Thêm các thành phần giao diện PLC
        self.plcGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.plcGroupBox.setTitle("CCTV Alarm")
        self.plcGroupBox.setFixedWidth(400)
        self.plcLayout = QtWidgets.QVBoxLayout(self.plcGroupBox)
        
        # Nhập IP và Port
        self.plcIpInput = QtWidgets.QComboBox(self.plcGroupBox)
        self.plcIpInput.setPlaceholderText("Select PLC IP")
        # self.plcIpInput.addItems(plc_ips)
        self.plcLayout.addWidget(self.plcIpInput)
        
        self.plcPortInput = QtWidgets.QLineEdit(self.plcGroupBox)
        self.plcPortInput.setPlaceholderText("Enter PLC Port (Default: 5000)")
        # self.plcPortInput.setText(plc_port)
        self.plcLayout.addWidget(self.plcPortInput)
        
        # Nút Connect và Disconnect
        self.plcConnectButton = QtWidgets.QPushButton("Connect PLC", self.plcGroupBox)
        self.plcDisconnectButton = QtWidgets.QPushButton("Disconnect PLC", self.plcGroupBox)
        self.plcLayout.addWidget(self.plcConnectButton)
        self.plcLayout.addWidget(self.plcDisconnectButton)
        
        # Nút đọc/ghi DI và DO
        # self.readDIButton = QtWidgets.QPushButton("Read DI", self.plcGroupBox)
        self.writeDOButton = QtWidgets.QPushButton("Send Alarm", self.plcGroupBox)
        # self.readDIButton.clicked.connect(self.read_DI_action)
        # self.writeDOButton.clicked.connect(self.write_DO_action)
        # self.plcLayout.addWidget(self.readDIButton)
        self.plcLayout.addWidget(self.writeDOButton)
        
        # Hiển thị trạng thái
        self.plcStatusLabel = QtWidgets.QLabel("Status: Not Connected PLC", self.plcGroupBox)
        self.plcLayout.addWidget(self.plcStatusLabel)
        
        # Thêm GroupBox PLC vào layout trái
        self.leftLayout.addWidget(self.plcGroupBox)
        
        # Kết nối sự kiện
        self.plcConnectButton.clicked.connect(self.connect_plc)
        self.plcDisconnectButton.clicked.connect(self.disconnect_plc)
        # self.readDIButton.clicked.connect(self.read_DI_action)
        self.writeDOButton.clicked.connect(self.write_DO_action)

        # Thêm QStatusBar
        self.status_bar = QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.status_bar)

        # Thêm thông tin version
        self.version_label = QLabel(" Version: 1.0.0 ", self.status_bar)
        self.status_bar.addPermanentWidget(self.version_label)

        # Thêm thời gian
        self.time_label = QLabel(" Date: 25/09/2024 ", self.status_bar)
        self.status_bar.addPermanentWidget(self.time_label)
        self.status_bar.setStyleSheet("background-color: #f0f0f0; color: #333; font-size: 15px; font-style: italic;")

        config_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), "setting.ini") # Đường dẫn file config
        plc_ips, plc_port, version_text = self.load_config(config_file) 
        self.plcIpInput.addItems(plc_ips)
        self.plcPortInput.setText(plc_port)
        self.plainTextEdit.setPlainText(version_text)

        for cam in self.camSettting:
            if self.F1_COP1_05.text() == cam["camName"]:
                self.start_cam1(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])
            elif self.F1_COP1_S1.text() == cam["camName"]:
                self.start_cam2(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])
            elif self.F1_COP1_04.text() == cam["camName"]:
                self.start_cam3(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])

        # self.start_cam1(camip='PKID=1', rtsp='//92.168.10.200/FA_Vision/2.SEI_AI_Camera/1.CCTVVideos/F1-COPPER 1_F1-COP1/F1-COP1-05/20240821/Video_20240821153113/F1-COPPER 1_F1-COP1-05_20240819140700_20240819141157.mp4')
        # self.start_cam2(camip='PKID=1', rtsp='//192.168.10.200/FA_Vision/2.SEI_AI_Camera/1.CCTVVideos/F1-COPPER 1_F1-COP1/F1-COP1-2/20240814/F1-COP1-02_20240814_CuonDong.mp4')
        # self.start_cam3(camip='PKID=1', rtsp='//92.168.10.200/FA_Vision/2.SEI_AI_Camera/1.CCTVVideos/F1-COPPER 1_F1-COP1/F1-COP1-04/20240802/Video_20240802164254/F1-COPPER 1_F1-COP1-04_20240802143941_20240802150541.mp4')

        # Kết nối sự kiện khi nhấp chuột vào treeView
        self.treeView.doubleClicked.connect(self.camSelect)

        # Kết nối sự kiện đóng cửa sổ
        MainWindow.closeEvent = self.on_close

        # Khởi động Timer để cập nhật API mỗi 10 giây
        self.setup_timer()

    def show(self, pos):
        """Hiển thị thông báo tại vị trí chỉ định."""
        self.move(pos)
        super().show()

#region camera
    def on_close(self, event):
        self.stop_all()
        event.accept()

    def camSelect(self, index):
        item = self.treeModel.itemFromIndex(index).text()
        if item == self.F1_COP1_05.text():
            self.chooseCam = 1
            self.stopCam = None
        elif item == self.F1_COP1_S1.text():
            self.chooseCam = 2
            self.stopCam = None
        elif item == self.F1_COP1_04.text():
            self.chooseCam = 3
            self.stopCam = None
        else:
            pass
        self.cameraWorker_cam1.set_chooseCam(self.chooseCam)
        self.cameraWorker_cam2.set_chooseCam(self.chooseCam)
        self.cameraWorker_cam3.set_chooseCam(self.chooseCam)

        self.cameraWorker_cam1.set_stopCam(self.stopCam)
        self.cameraWorker_cam2.set_stopCam(self.stopCam)
        self.cameraWorker_cam3.set_stopCam(self.stopCam)
        
    # def camSelect(self, camName):
    #     for item in self.camSettting:
    #         if item["camName"] == camName:
    #             self.cam1(camip='PKID='+item["pkid"])
     
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "CCTV-AI"))
        self.groupBox.setTitle(_translate("MainWindow", "Camera"))
        self.groupBox_2.setTitle(_translate("MainWindow", "History Version"))
        self.btnStop.setText(_translate("MainWindow", "Stop Camera"))
        self.btnVersion.setText(_translate("MainWindow", "Version"))

    def processFrame(self, frame):
        # Convert the frame to RGB format
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Create a QImage using the correct format
        image = QImage(
            frame.data,
            frame.shape[1],
            frame.shape[0],
            frame.strides[0],
            QImage.Format.Format_RGB888
        )
        pixmap = QPixmap.fromImage(image)

        if self.scenePixmapItem is None:
            self.scenePixmapItem = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(self.scenePixmapItem)
        else:
            self.scenePixmapItem.setPixmap(pixmap)
        self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
    
    # Phương thức dừng camera
    def stopCamera(self):
        self.cameraWorker.stop()
        self.thread.quit()
        self.thread.wait()

    # Phương thức hiển thị hình ảnh theo tỷ lệ
    def fitInView(self, rect, aspectRatioMode):
        self.gvMain.fitInView(rect, aspectRatioMode)

    # Phương thức hiển thị phiên bản
    def version(self):
        if self.tmpVersion == 0:
            self.tmpVersion = 1
            self.groupBox_2.show()
        else:
            self.tmpVersion = 0
            self.groupBox_2.hide()

    # Phương thức dừng camera
    def stop(self):
        self.scene.clear()
        self.scenePixmapItem = None
        self.scene.addItem(self.scenePixmapItem)
        self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        if self.chooseCam == 1:
            self.stopCam = 1
            self.cameraWorker_cam1.set_stopCam(self.stopCam)
        elif self.chooseCam == 2:
            self.stopCam = 2
            self.cameraWorker_cam2.set_stopCam(self.stopCam)
        elif self.chooseCam == 3:
            self.stopCam = 3
            self.cameraWorker_cam3.set_stopCam(self.stopCam)

        # self.cameraWorker_cam1.set_stopCam(self.stopCam)
        # self.cameraWorker_cam2.set_stopCam(self.stopCam)
        # self.cameraWorker_cam3.set_stopCam(self.stopCam)
    
    # Phương thức khởi chạy camera
    def start_cam1(self, camip, rtsp):
        self.cameraWorker_cam1.camera_index = rtsp
        self.cameraWorker_cam1.setModel(self.F1_COP1_05.text(), 'SSGLogic/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None
        if not self.thread_cam1.isRunning():
            self.thread_cam1.started.connect(self.cameraWorker_cam1.run)
            self.thread_cam1.start()

    # Phương thức khởi chạy camera
    def start_cam2(self, camip, rtsp):
        self.cameraWorker_cam2.camera_index = rtsp
        self.cameraWorker_cam2.setModel(self.F1_COP1_S1.text(), 'CBInside/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None

        if not self.thread_cam2.isRunning():
            self.thread_cam2.started.connect(self.cameraWorker_cam2.run)
            self.thread_cam2.start()

    # Phương thức khởi chạy camera
    def start_cam3(self, camip, rtsp):
        self.cameraWorker_cam3.camera_index = rtsp
        self.cameraWorker_cam3.setModel(self.F1_COP1_04.text(), 'CBOutside/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None
        if not self.thread_cam3.isRunning():
            self.thread_cam3.started.connect(self.cameraWorker_cam3.run)
            self.thread_cam3.start()

    # Phương thức dừng tất cả camera
    def stop_all(self):
        # Dừng tất cả các camera
        if self.thread_cam1.isRunning():
            self.cameraWorker_cam1.stop()
            self.thread_cam1.quit()
            self.thread_cam1.wait()
        
        if self.thread_cam2.isRunning():
            self.cameraWorker_cam2.stop()
            self.thread_cam2.quit()
            self.thread_cam2.wait()
        
        if self.thread_cam3.isRunning():
            self.cameraWorker_cam3.stop()
            self.thread_cam3.quit()
            self.thread_cam3.wait()

     # Phương thức cập nhật trạng thái PLC
#endregion

#region data
    def setup_timer(self):
        self.timer_reset = QTimer()
        self.timer_reset.timeout.connect(self.reset_Alarm)
        self.timer_reset.start(500) # Cứ 0.5 giây chạy một lần

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera_settings)
        self.timer.start(10000) # Cứ 10 giây chạy một lần

        self.update_camera_settings()  # Cập nhật lần đầu tiên

    def update_camera_settings(self):
        response = requests.get('http://10.212.10.234:81/api/GetTopCamSetting')
        data = response.json()
            
        list_abnormalType = ["No_Use_Glove"
                            ,"Material_Roll_On_Floor"
                            ,"Operator_Touch_In_To_CPanel"
                            ,"Operator_Cut_Material"
                            ,"Many_Operators"
                            ,"Lamp_Indicator_Was_Abnormal"]

        data_selected = [[item.get("camIP", "")
                        ,item.get("camPort","")
                        ,item.get("camName", "")
                        ,item.get("lineId", "")
                        ,list_abnormalType[(item.get("abnormalType", 0) - 1)] if 1 <= item.get("abnormalType", 0) <= len(list_abnormalType) else "Unknown"
                        ,item.get("abnormalDateTime", "").replace("T", " ")] for item in data]
        print("List Selected: ", data_selected)
            
        # Lấy dữ liệu hiện tại từ newtable
        new_data = [
                    [self.newtable.item(row, col).text() if self.newtable.item(row, col) else ""
                    for col in range(self.newtable.columnCount())]
                    for row in range(self.newtable.rowCount())
                    ]
        print("Data New: ",new_data)
        print("normalize_data(data_selected): ", self.normalize_data(data_selected))
        print("normalize_data(new_data): ", self.normalize_data(new_data))

        # So sánh dữ liệu mới và cũ
        if self.normalize_data(data_selected) != self.normalize_data(new_data):
                self.move_data_oldtable()  # Chuyển dữ liệu cũ sang oldtable
                self.add_data_newtable(data)  # Cập nhật dữ liệu mới vào newtable
                if self.plc and self.plc.connected:
                    self.send_Alarm() # Gửi dữ liệu mới qua PLC

    def add_data_newtable(self, data):
        self.newtable.setRowCount(len(data))  # Cập nhật số dòng
        
        list_abnormalType = ["No_Use_Glove"
                            ,"Material_Roll_On_Floor"
                            ,"Operator_Touch_In_To_CPanel"
                            ,"Operator_Cut_Material"
                            ,"Many_Operators"
                            ,"Lamp_Indicator_Was_Abnormal"]
        
        for row, rowData in enumerate(data):
            abnormal_type = list_abnormalType[(rowData.get("abnormalType", 0) - 1)] if 1 <= rowData.get("abnormalType", 0) <= len(list_abnormalType) else "Unknown" # Lấy tên loại lỗi

            self.newtable.setItem(row, 0, QTableWidgetItem(str(rowData.get("camIP", ""))))
            self.newtable.setItem(row, 1, QTableWidgetItem(str(rowData.get("camPort", ""))))
            self.newtable.setItem(row, 2, QTableWidgetItem(rowData.get("camName", "")))
            self.newtable.setItem(row, 3, QTableWidgetItem(str(rowData.get("lineId", ""))))
            self.newtable.setItem(row, 4, QTableWidgetItem(abnormal_type))
            self.newtable.setItem(row, 5, QTableWidgetItem(rowData.get("abnormalDateTime", "").replace("T", " ")))

    def add_data_oldtable(self, data):
        # Cập nhật bảng oldtable
        self.oldtable.setRowCount(len(data))
        for row, rowData in enumerate(data):
            for col, value in enumerate(rowData):
                self.oldtable.setItem(row, col, QTableWidgetItem(value))

    def move_data_oldtable(self):
        row_count = self.newtable.rowCount()
        col_count = self.newtable.columnCount()
        data = []

        # Lấy dữ liệu từ bảng newtable
        for row in range(row_count):
            rowData = []
            for col in range(col_count):
                item = self.newtable.item(row, col)
                rowData.append(item.text() if item else "")
            data.append(rowData)
        # Cập nhật bảng oldtable
        self.oldtable.setRowCount(len(data))
        for row, rowData in enumerate(data):
            for col, value in enumerate(rowData):
                self.oldtable.setItem(row, col, QTableWidgetItem(value))

        # Xóa dữ liệu trong newtable
        self.newtable.setRowCount(0)

    def normalize_data(self, data):
        # Chuẩn hóa dữ liệu
        normalized = []
        for row in data:
            normalized_row = [str(item) for item in row]  # Chuyển tất cả thành chuỗi
            normalized.append("".join(re.findall(r'\w+', " ".join(normalized_row)))) # Loại bỏ các ký tự đặc biệt trừ "_"
        return normalized
    
#endregion

#region PLC
    # Phương thức cập nhật trạng thái PLC
    def update_plc_status(self, status):
        self.plcStatusLabel.setText(f"Status: {status}")
    
    # Phương thức kết nối PLC
    def connect_plc(self):
        ip = self.plcIpInput.currentText()  # Lấy IP từ combobox
        if not ip:
            QtWidgets.QMessageBox.warning(self.centralwidget, "Invalid IP", "Please select a valid IP address!")
            return
        port_text = self.plcPortInput.text()
        try:
            port = int(port_text) if port_text else 502
        except ValueError:
            QtWidgets.QMessageBox.warning(self.centralwidget, "Invalid Port", "Please enter a valid port number.")
            return
        self.plc.connect(ip, port)
        if self.plc.connected:
            QtWidgets.QMessageBox.information(self.centralwidget, "PLC Connection", f"Connected to PLC at {ip}:{port}")
        else:
            QtWidgets.QMessageBox.warning(self.centralwidget, "PLC Connection", f"Failed to connect to PLC at {ip}:{port}. Please check the device and network.")
    
    # Phương thức ngắt kết nối PLC
    def disconnect_plc(self):
        self.plc.disconnect()
    
    # Phương thức đọc số (digital)
    def read_DI_action(self):
        register, ok = QtWidgets.QInputDialog.getInt(self.centralwidget, "Read DI", "Enter DI Address:")
        if ok:
            result = self.plc.read_DI(register)
            if result is not None:
                QtWidgets.QMessageBox.information(self.centralwidget, "DI Value", f"Value: {result}")
    
    # Phương thức ghi số (digital)
    def write_DO_action(self):
        """Thao tác ghi giá trị vào DO và kiểm tra lại trạng thái sau khi ghi"""
        # Test
        # # Nhập địa chỉ DO cần ghi
        # register, ok1 = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write DO", "Enter DO Address:")
        # if not ok1:
        #     return

        # # Nhập giá trị cần ghi (0 hoặc 1)
        # value, ok2 = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write Value", "Enter Value (0 or 1):")
        # if ok2:
        #     # Chuyển đổi giá trị nhập vào thành True (bật) hoặc False (tắt)
        #     value = True if value == 1 else False

        #     # Ghi giá trị vào DO và xác minh
        #     status = self.plc.write_DO(register, value)
        #     if status is not None:
        #         QtWidgets.QMessageBox.information(
        #             self.centralwidget,
        #             "Write DO Status",
        #             f"DO Address: {register}\nExpected Value: {value}\nActual Value: {status}"
        #         )
        #     else:
        #         QtWidgets.QMessageBox.warning(
        #             self.centralwidget,
        #             "Write DO Status",
        #             "Failed to verify the DO status after writing."
        #         )
        register = "17"
        value, ok = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write Value", "Enter Value (0 or 1):")
        if ok:
            value = True if value == 1 else False
            status = self.plc.write_DO(register, value)
            if status is not None:
                self.update_plc_status("The alarm was sent successfully!")

    def send_Alarm(self):
        register = 17
        value = True 
        status = self.plc.write_DO(register, value)
        if status is not None:
            self.update_plc_status("The alarm was sent successfully!")

    def reset_Alarm(self):
        register = 17
        value = False
        if self.plc and self.plc.connected:
            status = self.plc.read_DI(1)
            if status is True:
                status = self.plc.write_DO(register, value)
                self.update_plc_status("The worker successfully turned off the warning signal!")
#endregion

    def load_config(self, file):
        config = configparser.ConfigParser()
        config.read(file, encoding='utf-8')

        # Lấy giá trị từ file Setting.ini
        plc_ips = config.get('DEFAULT', 'IPAddressPLC', fallback="").split(",") # Lấy danh sách IP PLC
        plc_ips = [ip.strip() for ip in plc_ips]
        plc_port = config.get('DEFAULT', 'PortPLC', fallback="5000").strip() # Lấy Port PLC

        version_lines = []
        if config.has_section('VERSION'):
            version_lines = [f"{key} - {value}"
                             for key, value in config.items('VERSION')
                             if key.lower() not in ['ipaddressplc', 'portplc']]  # Lấy thông tin version bỏ qua IP và Port
        version_text = "\n".join(version_lines)
        return plc_ips, plc_port, "\n".join(version_lines) # Trả về danh sách IP PLC, Port PLC, thông tin version


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())