from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt, QTimer, QStringListModel
from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QApplication, QMessageBox, QMainWindow, QStatusBar, QLabel, QTextEdit, QTableView, QTableWidgetItem
from PyQt6.QtGui import QImage, QPixmap
import cv2
from camera_cabin_inside import cabin_inside_camera
from camera_cabin_outside import cabin_outside_camera
from camera_line_safety import line_safety_camera
import time
import yaml
import threading
import os
import logging
import datetime as dt
import requests
from pymodbus.client import ModbusTcpClient
from datetime import datetime, date
import re
import configparser
from logging.handlers import TimedRotatingFileHandler
from dateutil import parser
from CameraCaptureWorker import FreshestFrame

cabin_inside_monitor = cabin_inside_camera.SSGVision(config_path="camera_cabin_inside/config.yaml")
cabin_outside_monitor = cabin_outside_camera.SSGVision(config_path="camera_cabin_outside/config.yaml")
line_safety_monitor = line_safety_camera.SSGVision(config_path="camera_line_safety/config.yaml")

class CameraWorker(QObject):
    frameCaptured = pyqtSignal(object)  # Emit frame data

    def __init__(self, camera_index=0, log_folder="logs", logger_name="CameraWorker"):
        super().__init__()
        self.camera_index = camera_index
        self.running = False

        self.main_ui = Ui_MainWindow()
        self.chooseCam_ = None
        self.stopCam_ = None

        # self.iconRun = None
        # self.iconStop = None

        self.iconRun = QtGui.QIcon("images/run.png")
        self.iconStop = QtGui.QIcon("images/stop.png")

        self.Cam = None

        self.baseDir=  os.path.abspath(os.path.dirname(sys.argv[0]))
        if not os.path.exists(self.baseDir+"/log"):
            os.makedirs(self.baseDir+"/log")
        self.configure_logging()

        # Khá»Ÿi táº¡o logger riÃªng cho CameraWorker
        # self.logger = setup_daily_logger(log_folder, logger_name)

    def set_chooseCam(self, chooseCam):
        self.chooseCam_ = chooseCam

    def set_stopCam(self, stopCam):
        self.stopCam_ = stopCam

    def set_icon(self, Cam):
        self.Cam = Cam
        self.Cam.setIcon(self.iconRun)

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
            self.CAM1_model_main_config = config["model_main"]
            self.CAM1_api = config["api"]
            self.CAM1_apicamsetting = self.CAM1_api["API"] + self.CAM1_api["CAMERA_SETTING"] + camip
            self.CAM1_post = self.CAM1_api["POST"]
            if self.CAM1_post:
                self.CAM1_data_cam = line_safety_monitor.readCam_api(self.CAM1_apicamsetting)

                if self.CAM1_data_cam is not None:
                    line_safety_monitor.set_params(self.CAM1_data_cam)
            # Set up Box of Cabin, Panel, Floor and Light indicator position
            line_safety_monitor.set_coordinates()
            # Load model detect
            self.CAM1_model_main = line_safety_monitor.load_modelv8(self.CAM1_model_main_config["weights"])
            self.CAM1_model_keo = line_safety_monitor.load_modelv8(self.CAM1_model_main_config["weights_Keo"])
            # Set Color
            self.CAM1_colors = {
                "NG": eval(self.CAM1_model_main_config['colors']['NG']),
                "OK": eval(self.CAM1_model_main_config['colors']['OK']),
                "OBJECT": eval(self.CAM1_model_main_config["colors"]["OBJECT"]),
                "CABIN": eval(self.CAM1_model_main_config['colors']['CABIN']),
                "PANEL": eval(self.CAM1_model_main_config['colors']['PANEL']),
                "LIGHTRED": eval(self.CAM1_model_main_config['colors']['LIGHTRED']),
                "LIGHTYELLOW": eval(self.CAM1_model_main_config['colors']['LIGHTYELLOW']),
                "LIGHTGREEN": eval(self.CAM1_model_main_config['colors']['LIGHTGREEN']),
                "LIGHTALL": eval(self.CAM1_model_main_config['colors']['LIGHTALL']),
                "FLOOR": eval(self.CAM1_model_main_config['colors']['FLOOR']),
                "SCISSORCHECK": eval(self.CAM1_model_main_config['colors']['SCISSORCHECK']),
                "GATHER": eval(self.CAM1_model_main_config['colors']['GATHER']),
            }
            # line_safety_monitor.CheckDeleteLog(self.CAM1_baseDir)

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
            self.CAM2_model_main_config = config["model_main"]
            self.CAM2_api = config["api"]
            self.CAM2_apicamsetting = self.CAM2_api["API"] + self.CAM2_api["CAMERA_SETTING"] + camip
            self.CAM2_post = self.CAM2_api["POST"]
            if self.CAM2_post:
                self.CAM2_data_cam = cabin_inside_monitor.readCam_api(self.CAM2_apicamsetting)
                if self.CAM2_data_cam is not None:
                    cabin_inside_monitor.set_params(self.CAM2_data_cam)
            # Set up Box of Panel position
            cabin_inside_monitor.set_coordinates()
            # Load model detect
            self.CAM2_model_main = cabin_inside_monitor.load_model(self.CAM2_model_main_config["weights"])
            # Set Color
            self.CAM2_colors = {
                "NG": eval(self.CAM2_model_main_config['colors']['NG']),
                "OK": eval(self.CAM2_model_main_config['colors']['OK']),
                "OBJECT": eval(self.CAM2_model_main_config["colors"]["OBJECT"]),
                "PANEL": eval(self.CAM2_model_main_config['colors']['PANEL']),
                "FLOOR": eval(self.CAM2_model_main_config['colors']['FLOOR']),
                "GLOVECHECK": eval(self.CAM2_model_main_config['colors']['GLOVECHECK']),
                "SCISSORCHECK": eval(self.CAM2_model_main_config['colors']['SCISSORCHECK']),
                "LIGHTRED": eval(self.CAM2_model_main_config['colors']['LIGHTRED']),
                "LIGHTYELLOW": eval(self.CAM2_model_main_config['colors']['LIGHTYELLOW']),
                "LIGHTGREEN": eval(self.CAM2_model_main_config['colors']['LIGHTGREEN']),
                "LIGHTALL": eval(self.CAM2_model_main_config['colors']['LIGHTALL']),
            }
            # cabin_inside_monitor.CheckDeleteLog(self.CAM2_baseDir)

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
            self.CAM3_model_main_config = config["model_main"]
            self.CAM3_api = config["api"]
            self.CAM3_apicamsetting = self.CAM3_api["API"] + self.CAM3_api["CAMERA_SETTING"] + camip
            self.CAM3_post = self.CAM3_api["POST"]
            if self.CAM3_post:
                self.CAM3_data_cam = cabin_outside_monitor.readCam_api(self.CAM3_apicamsetting)
                if self.CAM3_data_cam is not None:
                    cabin_outside_monitor.set_params(self.CAM3_data_cam)

            cabin_outside_monitor.set_coordinates()
            # Load model detect
            self.CAM3_model_main = cabin_outside_monitor.load_model(self.CAM3_model_main_config["weights"])
            # Set Color
            self.CAM3_colors = {
                "NG": eval(self.CAM3_model_main_config['colors']['NG']),
                "OK": eval(self.CAM3_model_main_config['colors']['OK']),
                "OBJECT": eval(self.CAM3_model_main_config["colors"]["OBJECT"]),
                "FLOOR": eval(self.CAM3_model_main_config['colors']['FLOOR']),
                "GATHER": eval(self.CAM3_model_main_config['colors']['GATHER'])
            }
            # cabin_outside_monitor.CheckDeleteLog(self.CAM3_baseDir)
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
            time.sleep(1)
            return None

    def run(self):
        self.chooseCam_ = self.main_ui.chooseCam
        self.stopCam_ = self.main_ui.stopCam

        try:
            api_updatestatus = f'http://10.212.10.234:81/api/UpdateStatus?CamName={self.cam}&Status=1'
            try:
                requests.post(api_updatestatus, timeout= 5)
            except requests.exceptions.RequestException as e:
                logging.exception('failed to update status')
            self.running = True
            Reconnect = 0
            cap = self.connect_camera()
            thread_cap = FreshestFrame(cap, 0.04)
            status_posted = False
            self.Cam.setIcon(self.iconRun)

            while self.running:
                self.configure_logging()
                if cap is not None:
                    # ret, frame = cap.read()
                    ret, frame = thread_cap.read(True, 0, 2)
                    # print("RET:   ", ret)
                else:
                    ret = -1
                if ret < 0:
                    status_posted = False
                    Reconnect += 1
                    if Reconnect == 1:
                        self.Cam.setIcon(self.iconStop)
                        api_updatestatus = f'http://10.212.10.234:81/api/UpdateStatus?CamName={self.cam}&Status=0'
                        try:
                            requests.post(api_updatestatus, timeout= 5)
                        except requests.exceptions.RequestException as e:
                            logging.exception('failed to update status')
                    logging.exception('Camera {} failed to connect for {} time'.format(self.cam, Reconnect))
                    cap = self.connect_camera()
                    if cap is not None:
                        thread_cap = FreshestFrame(cap, 0.04)
                        Reconnect = 0
                    else:
                        time.sleep(1)
                    continue
                else:
                    Reconnect = 0
                    if not status_posted:
                        api_updatestatus = f'http://10.212.10.234:81/api/UpdateStatus?CamName={self.cam}&Status=1'
                        try:
                            requests.post(api_updatestatus, timeout= 5)
                        except requests.exceptions.RequestException as e:
                            logging.exception('failed to update status')

                        status_posted = True
                        self.Cam.setIcon(self.iconRun)

                    frame_copy = frame.copy()

                    if self.cam == "F1-COP1-05":
                        line_safety_monitor.draw_rectangles(frame_copy, self.CAM1_colors)
                        results = self.CAM1_model_main(source=frame, conf=self.CAM1_model_main_config["conf"])
                        frame = line_safety_monitor.process_results(results, frame_copy, frame, self.CAM1_colors, self.CAM1_model_keo, cap)
                        if self.chooseCam_ == 1:
                            if self.stopCam_ == 1:
                                continue
                            else:
                                self.frameCaptured.emit(frame)

                    if self.cam == "F1-COP1-S1":
                        cabin_inside_monitor.draw_rectangles(frame_copy, self.CAM2_colors)
                        results = self.CAM2_model_main(source=frame, conf=self.CAM2_model_main_config["conf"])
                        frame = cabin_inside_monitor.process_results(results, frame_copy, frame, self.CAM2_colors, cap)
                        if self.chooseCam_ == 2:
                            if self.stopCam_ == 2:
                                continue
                            else:
                                self.frameCaptured.emit(frame)

                    if self.cam == "F1-COP1-04":
                        cabin_outside_monitor.draw_rectangles(frame_copy, self.CAM3_colors)
                        results = self.CAM3_model_main(source=frame, conf=self.CAM3_model_main_config["conf"])
                        frame = cabin_outside_monitor.process_results(results, frame_copy, frame, self.CAM3_colors, cap)
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
    statusUpdated = pyqtSignal(str)  # TÃ­n hiá»‡u gá»­i tráº¡ng thÃ¡i cho giao diá»‡n

    def __init__(self, log_folder="logs"):
        super().__init__()
        self.client = None
        self.connected = False
        self.log_folder = log_folder
        self.current_date = date.today()
        self.logger = self.setup_daily_logger()

    def setup_daily_logger(self):
        """Cáº¥u hÃ¬nh logger theo ngÃ y"""
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        log_file = os.path.join(self.log_folder, f"{datetime.now().strftime('%Y-%m-%d')}.log")
        logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            filemode='a'  # Append náº¿u file Ä‘Ã£ tá»“n táº¡i
        )
        return logging.getLogger()

    def check_date(self):
        """Kiá»ƒm tra ngÃ y, náº¿u khÃ¡c ngÃ y hiá»‡n táº¡i thÃ¬ táº¡o file log má»›i"""
        if date.today() != self.current_date:
            self.current_date = date.today()
            self.logger = self.setup_daily_logger()

    def connect(self, ip, port=5000):
        """Káº¿t ná»‘i tá»›i thiáº¿t bá»‹ Advantech"""
        self.check_date()
        try:
            self.logger.info(f"Attempting to connect to {ip}:{port}")
            # self.client = ModbusTcpClient(ip, port)
            self.client = ModbusTcpClient(host=ip, port=port)
            if self.client.connect():
                self.connected = True
                self.statusUpdated.emit(f"Connected to PLC {ip}:{port}")
                self.logger.info(f"Connected to {ip}:{port}")
            else:
                self.statusUpdated.emit(f"Failed to connect to PLC {ip}:{port}")
                self.logger.error(f"Failed to connect to {ip}:{port}")
        except Exception as e:
            self.statusUpdated.emit(f"Error connecting to PLC {ip}:{port}: {e}")
            self.logger.exception(f"Error connecting to {ip}:{port}: {e}")

    def disconnect(self):
        """Ngáº¯t káº¿t ná»‘i thiáº¿t bá»‹"""
        self.check_date()
        if self.client:
            self.client.close()
            self.connected = False
            self.statusUpdated.emit("Disconnected to PLC")
            self.logger.info("Disconnected to PLC")

    def read_DI(self, register):
        """Äá»c tráº¡ng thÃ¡i tá»« Discrete Input (DI)"""
        self.check_date()
        if self.client and self.connected:
            try:
                # Äá»c Discrete Input (DI) tá»« Ä‘á»‹a chá»‰ register
                result = self.client.read_discrete_inputs(register, count= 1)  # Äá»c 1 bit tá»« DI
                if result.isError():
                    self.statusUpdated.emit(f"Error reading DI at address {register}")
                    # self.logger.error(f"Error reading DI at address {register}")
                    return None
                value = result.bits[0] # Tráº£ vá» tráº¡ng thÃ¡i cá»§a DI (True/False)
                # self.logger.info(f"Read value {value} from DI address {register}")
                return value
            except Exception as e:
                self.statusUpdated.emit(f"Error reading DI: {e}")
                # self.logger.exception(f"Error reading DI: {e}")
                return None
        else:
            self.statusUpdated.emit("PLC Not Connected")
            return None

    def write_DO(self, register, value):
        """Ghi giÃ¡ trá»‹ vÃ o Digital Output (DO) vÃ  ghi nháº­n tráº¡ng thÃ¡i"""
        self.check_date()
        if self.client and self.connected:
            try:
                # Ghi giÃ¡ trá»‹ vÃ o Coil (DO)
                result = self.client.write_coil(register, value)
                if result.isError():
                    self.statusUpdated.emit(f"Error writing {value} to DO at address {register}")
                    self.logger.error(f"Error writing {value} to DO at address {register}")
                    return None


                self.statusUpdated.emit(f"Successfully written {value} to DO address {register}")
                self.logger.info(f"Successfully written {value} to DO address {register}")
                return True

                # # Äá»c láº¡i giÃ¡ trá»‹ tá»« DO sau khi ghi
                # read_result = self.client.read_coils(register, count = 1)
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

    def check_connect(self, register, value):
        try:
            # Ghi giÃ¡ trá»‹ vÃ o Coil (DO)
            result = self.client.write_coil(register, value)
            if result.isError():
                return None

            # Äá»c láº¡i giÃ¡ trá»‹ tá»« DO sau khi ghi
            read_result = self.client.read_coils(register, count = 1)
            if read_result.isError():
                return None
            read_value = read_result.bits[0]
            return read_value
        except Exception as e:
            return None

class Ui_MainWindow(object):
    def __init__(self):
        self.threads = []
        self.model = QStringListModel()
        self.chooseCam = None
        self.stopCam = None

    def setupUi(self, MainWindow):
        response = requests.get('http://10.212.10.234:81/api/GetCamSetting', timeout=5)
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

        # Khá»Ÿi táº¡o PLC_Advantech
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
        self.plainTextEdit.setPlainText("Version 1.1.0 - 14/01/2025")
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

        # ThÃªm cÃ¡c thÃ nh pháº§n giao diá»‡n database
        self.dataNewGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.dataNewGroupBox.setTitle("Data Alarm New")
        self.dataNewGroupBox.setFixedHeight(100)
        self.dataNewlayout = QtWidgets.QVBoxLayout(self.dataNewGroupBox)

        # Táº¡o báº£ng cho groupbox datanew
        self.newtable = QtWidgets.QTableWidget(1, 6, parent=self.centralwidget)
        self.newtable.setColumnCount(6)
        self.newtable.setHorizontalHeaderLabels(["CamIP", "CamPort", "CamName", "LineID", "AbnormalType", "AbnormalDateTime"])
        self.newtable.setColumnWidth(0, 150)
        self.newtable.setColumnWidth(1, 70)
        self.newtable.setColumnWidth(2, 200)
        self.newtable.setColumnWidth(3, 70)
        self.newtable.setColumnWidth(4, 350)
        self.newtable.horizontalHeader().setStretchLastSection(True) # Äá»ƒ cá»™t cuá»‘i cÃ¹ng co giÃ£n
        self.newtable.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers) # KhÃ´ng cho chá»‰nh sá»­a
        self.dataNewlayout.addWidget(self.newtable)

        self.rightLayout.addWidget(self.dataNewGroupBox)

        # ThÃªm cÃ¡c thÃ nh pháº§n giao diá»‡n database
        self.dataOldGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.dataOldGroupBox.setTitle("Data Alarm Old")
        self.dataOldGroupBox.setFixedHeight(100)
        self.dataOldlayout = QtWidgets.QVBoxLayout(self.dataOldGroupBox)

        # Táº¡o báº£ng cho groupbox dataold
        self.oldtable = QtWidgets.QTableWidget(1, 6, parent=self.centralwidget)
        self.oldtable.setColumnCount(6)
        self.oldtable.setHorizontalHeaderLabels(["CamIP", "CamPort", "CamName", "LineID", "AbnormalType", "AbnormalDateTime"])
        self.oldtable.setColumnWidth(0, 150)
        self.oldtable.setColumnWidth(1, 70)
        self.oldtable.setColumnWidth(2, 200)
        self.oldtable.setColumnWidth(3, 70)
        self.oldtable.setColumnWidth(4, 350)
        self.oldtable.horizontalHeader().setStretchLastSection(True) # Äá»ƒ cá»™t cuá»‘i cÃ¹ng co giÃ£n
        self.oldtable.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers) # KhÃ´ng cho chá»‰nh sá»­a
        self.dataOldlayout.addWidget(self.oldtable)

        self.rightLayout.addWidget(self.dataOldGroupBox)

        # NÃºt lÃ m má»›i dá»¯ liá»‡u
        # self.refreshButton = QtWidgets.QPushButton("Refresh Data", self.centralwidget)
        # self.refreshButton.clicked.connect(self.load_data)
        # self.rightLayout.addWidget(self.refreshButton)

        # ThÃªm cÃ¡c thÃ nh pháº§n giao diá»‡n PLC
        self.plcGroupBox = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.plcGroupBox.setTitle("CCTV Alarm")
        self.plcGroupBox.setFixedWidth(400)
        self.plcLayout = QtWidgets.QVBoxLayout(self.plcGroupBox)

        # Nháº­p IP vÃ  Port
        self.plcInputLayout =QtWidgets.QHBoxLayout()
        self.label_IP = QLabel("IP:", self.plcGroupBox)
        self.plcIpInput = QtWidgets.QComboBox(self.plcGroupBox)
        self.plcIpInput.setPlaceholderText("192.168.90.252")
        self.plcIpInput.setFixedSize(340,25)
        self.plcInputLayout.addWidget(self.label_IP)
        self.plcInputLayout.addStretch()
        self.plcInputLayout.addWidget(self.plcIpInput)
        self.plcLayout.addLayout(self.plcInputLayout)
        # self.plcLayout.addWidget(self.plcIpInput)

        self.plcPortInputLayout = QtWidgets.QHBoxLayout()
        self.label_Port =QLabel("Port:", self.plcGroupBox)
        self.plcPortInput = QtWidgets.QLineEdit(self.plcGroupBox)
        self.plcPortInput.setPlaceholderText("Enter PLC Port (Default: 5000)")
        self.plcPortInput.setFixedSize(340,25)
        self.plcPortInputLayout.addWidget(self.label_Port)
        self.plcPortInputLayout.addStretch()
        self.plcPortInputLayout.addWidget(self.plcPortInput)
        self.plcLayout.addLayout(self.plcPortInputLayout)
        # self.plcLayout.addWidget(self.plcPortInput)

        # NÃºt Connect vÃ  Disconnect
        self.plcButtonLayout = QtWidgets.QHBoxLayout()
        self.plcConnectButton = QtWidgets.QPushButton("Connect PLC", self.plcGroupBox)
        self.plcDisconnectButton = QtWidgets.QPushButton("Disconnect PLC", self.plcGroupBox)
        self.plcConnectButton.setStyleSheet("background-color: #4CAF50; color: white;")
        self.plcDisconnectButton.setStyleSheet("background-color: #f44336; color: white;")
        self.plcConnectButton.setFixedSize(180,25)
        self.plcDisconnectButton.setFixedSize(180,25)
        self.plcButtonLayout.addWidget(self.plcConnectButton)
        self.plcButtonLayout.addStretch()
        self.plcButtonLayout.addWidget(self.plcDisconnectButton)
        self.plcLayout.addLayout(self.plcButtonLayout)
        # self.plcLayout.addWidget(self.plcConnectButton)
        # self.plcLayout.addWidget(self.plcDisconnectButton)

        # NÃºt Ä‘á»c/ghi DI vÃ  DO
        # self.readDIButton = QtWidgets.QPushButton("Read DI", self.plcGroupBox)
        # self.readDIButton.clicked.connect(self.read_DI_action)
        # self.plcLayout.addWidget(self.readDIButton)
        self.writeDOButton = QtWidgets.QPushButton("Send Alarm", self.plcGroupBox)
        self.writeDOButton.setStyleSheet("background-color: #32AFD2; color: white;")
        # self.writeDOButton.clicked.connect(self.write_DO_action)
        self.plcLayout.addWidget(self.writeDOButton)

        # Hiá»ƒn thá»‹ tráº¡ng thÃ¡i
        self.plcStatusLabel = QtWidgets.QLabel("Status: Not Connected PLC", self.plcGroupBox)
        self.plcStatusLabel.setStyleSheet("font-size: 14px; color: #333;")
        self.plcLayout.addWidget(self.plcStatusLabel)

        # ThÃªm GroupBox PLC vÃ o layout trÃ¡i
        self.leftLayout.addWidget(self.plcGroupBox)

        # Káº¿t ná»‘i sá»± kiá»‡n
        self.plcConnectButton.clicked.connect(self.connect_plc)
        self.plcDisconnectButton.clicked.connect(self.disconnect_plc)
        # self.readDIButton.clicked.connect(self.read_DI_action)
        self.writeDOButton.clicked.connect(self.write_DO_action)

        # ThÃªm QStatusBar
        self.status_bar = QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.status_bar)

        # ThÃªm thÃ´ng tin phiÃªn báº£n
        self.version_label = QLabel(" Version: 1.0.0 ", self.status_bar)
        self.status_bar.addPermanentWidget(self.version_label)

        # ThÃªm thá»i gian
        self.time_label = QLabel(" Date: 25/09/2024  ", self.status_bar)
        self.status_bar.addPermanentWidget(self.time_label)

        # Them trang thai connect PLC
        self.icon_plc = QLabel()
        self.icon_plc.setPixmap(QPixmap("images/stop.png").scaled(16,16))
        self.text_plc = QLabel("PLC Not Connected")
        self.text_plc.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.status_bar.addWidget(self.icon_plc)
        self.status_bar.addWidget(self.text_plc)

        self.status_bar.setStyleSheet("background-color: #f0f0f0; color: #333; font-size: 15px; font-style: italic;")

        config_file = os.path.join(os.path.abspath(os.path.dirname(__file__)), "setting.ini") # ÄÆ°á»ng dáº«n file config
        plc_ips, plc_port, version_text = self.load_config(config_file)
        self.plcIpInput.addItems(plc_ips)
        self.plcPortInput.setText(plc_port)
        self.plainTextEdit.setPlainText(version_text)

        # Táº¡o QTimer Ä‘á»ƒ cáº­p nháº­t thá»i gian
        # self.timer = QTimer(self.status_bar)
        # self.timer.timeout.connect(self.update_time)
        # self.timer.start(1000)  # Cáº­p nháº­t má»—i giÃ¢y
        # self.update_time()

        for cam in self.camSettting:
            if self.F1_COP1_05.text() == cam["camName"]:
                self.start_cam1(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])
            elif self.F1_COP1_S1.text() == cam["camName"]:
                self.start_cam2(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])
            elif self.F1_COP1_04.text() == cam["camName"]:
                self.start_cam3(camip='PKID=' + str(cam["pkid"]), rtsp=cam["rtsp"])

        # self.start_cam1(camip='PKID=1', rtsp='//192.168.10.200/FA_Vision/2.SEI_AI_Camera/1.CCTVVideos/Video_20241004/Video_20241004150212/F1-COPPER 1_F1-COP1-05_20241004062959_20241004065959.mp4')
        # self.start_cam2(camip='PKID=1', rtsp = 'D:/PROJECT/Project_CCTV/7.CCTV-AI/03-Development/Tool/CutVideo/outputcut/20240822164635_20240822165738_164651_23.mp4')
        # self.start_cam3(camip='PKID=1', rtsp = 'D:/PROJECT/Project_CCTV/7.CCTV-AI/03-Development/FormCam/F1-COP1-04_20241004102800_150.mp4')

        # Káº¿t ná»‘i sá»± kiá»‡n khi nháº¥p chuá»™t vÃ o treeView
        self.treeView.doubleClicked.connect(self.camSelect)

        # Káº¿t ná»‘i sá»± kiá»‡n Ä‘Ã³ng cá»­a sá»•
        MainWindow.closeEvent = self.on_close

        # Khá»Ÿi Ä‘á»™ng Timer Ä‘á»ƒ cáº­p nháº­t API má»—i 10 giÃ¢y
        self.setup_timer()

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

    def stopCamera(self):
        self.cameraWorker.stop()
        self.thread.quit()
        self.thread.wait()

    def fitInView(self, rect, aspectRatioMode):
        self.gvMain.fitInView(rect, aspectRatioMode)

    def version(self):
        if self.tmpVersion == 0:
            self.tmpVersion = 1
            self.groupBox_2.show()
        else:
            self.tmpVersion = 0
            self.groupBox_2.hide()

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

    def start_cam1(self, camip, rtsp):
        self.cameraWorker_cam1.camera_index = rtsp
        self.cameraWorker_cam1.setModel(self.F1_COP1_05.text(), 'camera_line_safety/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None
        if not self.thread_cam1.isRunning():
            self.thread_cam1.started.connect(self.cameraWorker_cam1.run)
            self.thread_cam1.start()

    def start_cam2(self, camip, rtsp):
        self.cameraWorker_cam2.camera_index = rtsp
        self.cameraWorker_cam2.setModel(self.F1_COP1_S1.text(), 'camera_cabin_inside/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None

        if not self.thread_cam2.isRunning():
            self.thread_cam2.started.connect(self.cameraWorker_cam2.run)
            self.thread_cam2.start()

    def start_cam3(self, camip, rtsp):
        self.cameraWorker_cam3.camera_index = rtsp
        self.cameraWorker_cam3.setModel(self.F1_COP1_04.text(), 'camera_cabin_outside/config.yaml', camip)
        self.scene.clear()
        self.scenePixmapItem = None
        if not self.thread_cam3.isRunning():
            self.thread_cam3.started.connect(self.cameraWorker_cam3.run)
            self.thread_cam3.start()

    def stop_all(self):
        # Dá»«ng táº¥t cáº£ cÃ¡c camera
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

    def setup_timer(self):
        self.timer_reset = QTimer()
        self.timer_reset.timeout.connect(self.reset_Alarm)
        self.timer_reset.start(500) # Cá»© 0.5 giÃ¢y cháº¡y má»™t láº§n

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera_settings)
        self.timer.start(10000)  # Cá»© 10 giÃ¢y cháº¡y má»™t láº§n

        self.timer_check_Connect = QTimer()
        self.timer_check_Connect.timeout.connect(self.check_connect)
        self.timer_check_Connect.start(30000)

        self.update_camera_settings()  # Cáº­p nháº­t láº§n Ä‘áº§u tiÃªn
        self.check_connect()

    # def retry_get_data(timeout=100, retry_delay= 5):
    #     while True:
    #         try:
    #             response = requests.get('http://10.212.10.234:81/api/GetTopCamSetting', timeout=timeout)
    #             data = response.json()
    #             print("Dá»¯ liá»‡u nháº­n Ä‘Æ°á»£c:", data)
    #             return data  # ThoÃ¡t khá»i vÃ²ng láº·p khi cÃ³ dá»¯ liá»‡u
    #         except requests.exceptions.Timeout:
    #             print(f"QuÃ¡ thá»i gian chá» {timeout} giÃ¢y. Äang thá»­ láº¡i sau {retry_delay} giÃ¢y...")
    #         except requests.exceptions.RequestException as e:
    #             print(f"Lá»—i xáº£y ra: {e}. Äang thá»­ láº¡i sau {retry_delay} giÃ¢y...")

    #         # Chá» trÆ°á»›c khi thá»­ láº¡i
    #         time.sleep(retry_delay)

    def update_camera_settings(self):
        try:
            response = requests.get('http://10.212.10.234:81/api/GetTopCamSetting', timeout=100)
            data = response.json()
        except:
            logging.exception('QuÃ¡ thá»i timeout {timeout} giÃ¢y')
            return None

        # data = self.retry_get_data()

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
        # print("List Selected: ", data_selected)

        # Láº¥y dá»¯ liá»‡u hiá»‡n táº¡i tá»« newtable
        new_data = [
                    [self.newtable.item(row, col).text() if self.newtable.item(row, col) else ""
                    for col in range(self.newtable.columnCount())]
                    for row in range(self.newtable.rowCount())
                   ]
        # print("Data New: ",new_data)
        # print("normalize_data(data_selected): ", self.normalize_data(data_selected))
        # print("normalize_data(new_data): ", self.normalize_data(new_data))

        # So sÃ¡nh dá»¯ liá»‡u má»›i vÃ  cÅ©
        if self.normalize_data(data_selected) != self.normalize_data(new_data):
                self.move_data_oldtable()  # Chuyá»ƒn dá»¯ liá»‡u cÅ© sang oldtable
                self.add_data_newtable(data)  # Cáº­p nháº­t dá»¯ liá»‡u má»›i vÃ o newtable
                if self.plc and self.plc.connected:
                    self.send_Alarm() # Gá»­i dá»¯ liá»‡u má»›i qua PLC

    def add_data_newtable(self, data):
        self.newtable.setRowCount(len(data))  # Cáº­p nháº­t sá»‘ dÃ²ng

        list_abnormalType = ["No_Use_Glove"
                            ,"Material_Roll_On_Floor"
                            ,"Operator_Touch_In_To_CPanel"
                            ,"Operator_Cut_Material"
                            ,"Many_Operators"
                            ,"Lamp_Indicator_Was_Abnormal"]

        for row, rowData in enumerate(data):
            abnormal_type = list_abnormalType[(rowData.get("abnormalType", 0) - 1)] if 1 <= rowData.get("abnormalType", 0) <= len(list_abnormalType) else "Unknown" # Láº¥y tÃªn loáº¡i lá»—i

            self.newtable.setItem(row, 0, QTableWidgetItem((str(rowData.get("camIP", "")))))
            self.newtable.setItem(row, 1, QTableWidgetItem(str(rowData.get("camPort", ""))))
            self.newtable.setItem(row, 2, QTableWidgetItem(rowData.get("camName", "")))
            self.newtable.setItem(row, 3, QTableWidgetItem(str(rowData.get("lineId", ""))))
            self.newtable.setItem(row, 4, QTableWidgetItem(abnormal_type))
            self.newtable.setItem(row, 5, QTableWidgetItem(rowData.get("abnormalDateTime", "").replace("T", " ")))

    def add_data_oldtable(self, data):
        # Cáº­p nháº­t báº£ng oldtable
        self.oldtable.setRowCount(len(data))
        for row, rowData in enumerate(data):
            for col, value in enumerate(rowData):
                self.oldtable.setItem(row, col, QTableWidgetItem(value))

    def move_data_oldtable(self):
        row_count = self.newtable.rowCount()
        col_count = self.newtable.columnCount()
        data = []
        # Láº¥y dá»¯ liá»‡u tá»« báº£ng newtable
        for row in range(row_count):
            rowData = []
            for col in range(col_count):
                item = self.newtable.item(row, col)
                rowData.append(item.text() if item else "")
            data.append(rowData)
        # Cáº­p nháº­t báº£ng oldtable
        self.oldtable.setRowCount(len(data))
        for row, rowData in enumerate(data):
            for col, value in enumerate(rowData):
                self.oldtable.setItem(row, col, QTableWidgetItem(value))

        # XÃ³a dá»¯ liá»‡u trong newtable
        self.newtable.setRowCount(0)

    def normalize_data(self, data):
        # Chuáº©n hÃ³a dá»¯ liá»‡u trÆ°á»›c khi hiá»ƒn thá»‹
        normalized = []
        for row in data:
            normalized_row = [str(item) for item in row]  # Chuyá»ƒn táº¥t cáº£ thÃ nh chuá»—i
            normalized.append("".join(re.findall(r'\w+', " ".join(normalized_row)))) # Loáº¡i bá» cÃ¡c kÃ½ tá»± Ä‘áº·c biá»‡t trá»« "_"
        return normalized

     # PhÆ°Æ¡ng thá»©c cáº­p nháº­t tráº¡ng thÃ¡i PLC
    def update_plc_status(self, status):
        self.plcStatusLabel.setText(f"Status: {status}")
        self.plcStatusLabel.setStyleSheet("font-size: 14px;")

    # PhÆ°Æ¡ng thá»©c cáº­p nháº­t tráº¡ng thÃ¡i StatusBar connect PLC
    def update_connect_plc_status(self, status):
        if status == True:
            self.icon_plc.setPixmap(QPixmap("images/run.png").scaled(16,16))
            self.text_plc.setText("Connected To PLC")
            self.text_plc.setStyleSheet("color: green; font-size: 14px; font-weight: bold;")
        else:
            self.icon_plc.setPixmap(QPixmap("images/stop.png").scaled(16,16))
            self.text_plc.setText("PLC Not Connected")
            self.text_plc.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")


    # PhÆ°Æ¡ng thá»©c káº¿t ná»‘i PLC
    def connect_plc(self):
        # ip = self.plcIpInput.currentText()
        ip = self.plcIpInput.itemText(0)
        if not ip:
            QtWidgets.QMessageBox.warning(self.centralwidget, "Invalid IP", "Please enter a valid IP address!")
            return
        port_text = self.plcPortInput.text()
        try:
            port = int(port_text) if port_text else 502
        except ValueError:
            QtWidgets.QMessageBox.warning(self.centralwidget, "Invalid Port", "Please enter a valid port number!")
            return
        self.plc.connect(ip, port)
        if self.plc.connected:
            QtWidgets.QMessageBox.information(self.centralwidget, "PLC Connection", f"Connected to PLC at {ip}:{port}")
            self.update_connect_plc_status(True)
        else:
            QtWidgets.QMessageBox.warning(self.centralwidget, "PLC Connection", f"Failed to connect to PLC at {ip}:{port}. Please check the device and network.")
            self.update_connect_plc_status(False)

    # PhÆ°Æ¡ng thá»©c ngáº¯t káº¿t ná»‘i PLC
    def disconnect_plc(self):
        self.plc.disconnect()
        self.update_connect_plc_status(False)

    # PhÆ°Æ¡ng thá»©c Ä‘á»c sá»‘ (digital)
    def read_DI_action(self):
        register, ok = QtWidgets.QInputDialog.getInt(self.centralwidget, "Read DI", "Enter DI Address:")
        if ok:
            result = self.plc.read_DI(register)
            if result is not None:
                QtWidgets.QMessageBox.information(self.centralwidget, "DI Value", f"Value: {result}")


    def write_DO_action(self):
        """Thao tÃ¡c ghi giÃ¡ trá»‹ vÃ o DO vÃ  kiá»ƒm tra láº¡i tráº¡ng thÃ¡i sau khi ghi"""
        # Test
        # Nháº­p Ä‘á»‹a chá»‰ DO cáº§n ghi
        # register, ok1 = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write DO", "Enter DO Address:") # Thanh ghi dia chi 17
        # if not ok1:
        #     return
        # # Nháº­p giÃ¡ trá»‹ cáº§n ghi (0 hoáº·c 1)
        # value, ok2 = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write Value", "Enter Value (0 or 1):")
        # if ok2:
        #     # Chuyá»ƒn Ä‘á»•i giÃ¡ trá»‹ nháº­p vÃ o thÃ nh True (báº­t) hoáº·c False (táº¯t)
        #     value = True if value == 1 else False
        #     # Ghi giÃ¡ trá»‹ vÃ o DO vÃ  xÃ¡c minh
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

        register = 17
        value, ok = QtWidgets.QInputDialog.getInt(self.centralwidget, "Write Value", "Enter Value (0 or 1):", min=0, max=1)
        if ok:
            value = True if value == 1 else False
            status = self.plc.write_DO(register, value)
            if status is not None:
                # QtWidgets.QMessageBox.information(
                #     self.centralwidget,
                #     "Write DO Status",
                #     f"DO Address: {register}\nExpected Value: {value}\nActual Value: {status}"
                # )
                self.update_plc_status("The alarm was sent successfully!")

    def send_Alarm(self):
        register = 17
        value = True
        status = self.plc.write_DO(register, value)
        if status is not None:
            self.update_plc_status("The alarm was sent successfully!")

    def reset_Alarm(self):
        register_DI = 0
        register_DO = 17
        value_DO = False
        if self.plc and self.plc.connected:
            status_DI = self.plc.read_DI(register_DI)
            if status_DI is True:
                status_DO = self.plc.write_DO(register_DO, value_DO)
                if status_DO is not None:
                    self.update_plc_status("The worker successfully turned off the warning signal!")

    def check_connect(self):
        if self.plc and self.plc.connected:
            status_DO = self.plc.check_connect(22, True)
            if status_DO is None:
                self.update_connect_plc_status(False)
            else:
                self.update_connect_plc_status(True)
        else:
            ip = self.plcIpInput.itemText(0)
            port_text = self.plcPortInput.text()
            port = int(port_text) if port_text else 502
            self.plc.connect(ip, port)
            if self.plc.connected:
                self.update_connect_plc_status(True)
            else:
                self.update_connect_plc_status(False)

    def load_config(self, file):
        config = configparser.ConfigParser()
        config.read(file, encoding='utf-8')

        # Láº¥y giÃ¡ trá»‹ tá»« file Setting.ini
        plc_ips = config.get('DEFAULT', 'IPAddressPLC', fallback="").split(",") # Láº¥y danh sÃ¡ch IP PLC
        plc_ips = [ip.strip() for ip in plc_ips]
        plc_port = config.get('DEFAULT', 'PortPLC', fallback="5000").strip() # Láº¥y Port PLC

        version_lines = []
        if config.has_section('VERSION'):
            version_lines = [f"{key} - {value}"
                             for key, value in config.items('VERSION')
                             if key.lower() not in ['ipaddressplc', 'portplc']]  # Láº¥y thÃ´ng tin version bá» qua IP vÃ  Port
        version_text = "\n".join(version_lines)
        return plc_ips, plc_port, "\n".join(version_lines) # Tráº£ vá» danh sÃ¡ch IP PLC, Port PLC, thÃ´ng tin version


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())

