import os
import time
import cv2
import yaml
import torch
import numpy as np
import sys
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import requests
import base64
import json
import logging               #import lib for write log
import datetime as dt       #import datetime library
from dateutil import parser      #import lib for read config
from .CameraCaptureWorker import FreshestFrame
from .CentroidTracker import  CentroidTracker
from .CentroidTrackerHistory import CentroidTrackerHistory
sys.path.insert(0, './yolov10')
from datetime import datetime
from ultralytics import YOLOv10, YOLO
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath
# os.environ['CUDA_LAUNCH_BLOCKING']="1"
# os.environ["TORCH_USE_CUDA_DSA"]="1"

GLOVE_CONSTANT = 0 # Class Glove
NOGLOVE_CONSTANT = 1 # Class NoGlove
HAND_CONSTANT = 2 # Class Hand
PEOPLE_CONSTANT = 3 # Class People
ROLL_CONSTANT = 4 # Class Roll
SCISSOR_LABEL = 3 # Class Scissor
MIN_SCORE_CONSTANT = 0.5 # Score to check
MIN_CHECK_FRAMES = 10 # Number of frames to check
CLOSE_PEOPLE = 700 # Distance close people
TIME_CLOSE_PEOPLE = 120 # Time close people
NUM_PEOPLE_CHECK = 3
# MIN_SCORE_CONSTANT_CHECK = 0.8
# also acts (partly) like a cv.VideoCapture
# thread sleep
sleep_interval=0.04
# logging.basicConfig(filename= 'post_time_alarm_cam1.log', level= logging.INFO,
#                     format= '%(asctime)s - %(message)s')
class SSGVision:
    def __init__(self, config_path):
        self.logger_cam1 = logging.getLogger('cam1')
        self.logger_cam1.setLevel(logging.INFO)
        file_handler_cam1 = logging.FileHandler('post_time_alarm_cam1.log')
        file_handler_cam1.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.logger_cam1.addHandler(file_handler_cam1)        

        # Read parameter from config path
        self.CAM1_config_path = config_path
        # Read parameter from config path
        with open(self.CAM1_config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
        self.CAM1_general_config = config["general"]
        self.CAM1_UseThreadCap = self.CAM1_general_config["ThreadCap"]
        self.CAM1_model_all_config = config["model_All"]
        self.CAM1_roll_warning = config["accept_roll"]
        self.CAM1_scissor_warning = config["Scissor_warning"]
        self.CAM1_lightbox = config["lightBox"]
        self.CAM1_scissorBox = config["scissorBox"]
        self.CAM1_cabin_warning = config["cabin"]
        self.CAM1_panel_warning = config["panel"]
        self.CAM1_gather_warning = config['gather_warning']
        self.CAM1_api = config["api"]
        self.CAM1_MaxRetry = int(self.CAM1_general_config['MaxRetry'])
        self.CAM1_MaxRetryAPI = int(self.CAM1_general_config['MaxRetryAPI'])
        self.CAM1_ip_address = self.CAM1_general_config['source']
        self.CAM1_auto_restart = self.CAM1_general_config['autoRestart']
        self.CAM1_output_path = self.CAM1_general_config['output']
        self.CAM1_device_setting = self.CAM1_general_config['device']
        self.CAM1_imgsz = self.CAM1_model_all_config['imgsz']
        self.CAM1_apicamsetting = self.CAM1_api["API"] + self.CAM1_api["CAMERA_SETTING"]
        self.CAM1_apiabnormal = self.CAM1_api["API"] + self.CAM1_api["ABNORMAL"]
        self.CAM1_apimedia = self.CAM1_api["API"] + self.CAM1_api["MEDIA"]
        self.CAM1_post = self.CAM1_api["POST"]
        # Set up Box of Cabin, Panel, Floor and Light indicator position
        self.CAM1_desired_width = 1920
        self.CAM1_desired_height = 1080
        self.CAM1_writer = self.setup_video_writer()
        self.CAM1_results = None
        self.CAM1_dect_copper = []
        self.CAM1_duration = 0
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
        # Initialize required parameter for check light indicator
        # self.CAM1_pre_mean_Red = 0
        # self.CAM1_pre_mean_Green = 0
        # self.CAM1_pre_mean_Yellow = 0
        self.CAM1_lightRed = False
        self.CAM1_lightGreen = False
        self.CAM1_lightYellow = False

        # Initialize required parameter for check (no use glove, roll on the floor and hand touch Panel)
        self.CAM1_check_glove = 0
        self.CAM1_check_roll = 0
        self.CAM1_check_touchpanel = 0
        self.CAM1_check_cut = 0
        self.CAM1_check_gather = 0
        self.CAM1_check_light = 0

        self.CAM1_check_flashing = 0

        self.CAM1_noglove_img_check = {}
        self.CAM1_roll_img_check = {}
        self.CAM1_touch_panel_img_check = {}
        self.CAM1_cut_img_check = {}
        self.CAM1_light_img_check = {}

        # Initialize other parameter
        self.CAM1_start_time = None
        self.CAM1_text_color = (255, 255, 255)
        self.CAM1_Tracker = CentroidTrackerHistory(30)
        self.CAM1_RollTracker = CentroidTrackerHistory(15)
        self.CAM1_HandTracker = CentroidTracker(10)
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
    def setup_video_writer(self):
        if self.CAM1_general_config['save']:
            pass
        else:
            return None
        filename = os.path.basename(self.CAM1_output_path)
        save_path = os.path.dirname(self.CAM1_output_path)
        vid_path = os.path.join(save_path, filename+ ".avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(vid_path, fourcc, 24, (self.CAM1_desired_width, self.CAM1_desired_height))
        if not writer.isOpened():
            print(f"Can't Open VideoWriter With Path: {vid_path}")
        return writer
    # Read parameter from Camera
    def readCam_api(self, api):
        try:
            response = requests.get(api)
            print("Read API successful...")
            data = response.json()
            return data
        except:
            time.sleep(2)
            return None

    def set_params(self, data_cam):
        self.CAM1_params_glove = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 1,
            'AbnormalDuration': 1
        }
        self.CAM1_params_roll = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 2,
            'AbnormalDuration': 1
        }
        self.CAM1_params_touchpanel = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 3,
            'AbnormalDuration': 1
        }
        self.CAM1_params_cut = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 4,
            'AbnormalDuration': 1
        }
        self.CAM1_params_manyoperators = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 5,
            'AbnormalDuration': 120
        }
        self.CAM1_params_lamp = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 6,
            'AbnormalDuration': 1
        }
        self.CAM1_headers = {
                'Content-Type': 'application/json'
            }

    def set_coordinates(self):
        self.CAM1_left_warning_coor = self.CAM1_scissorBox['left']
        self.CAM1_top_warning_coor = self.CAM1_scissorBox['top']
        self.CAM1_right_warning_coor = self.CAM1_scissorBox['right']
        self.CAM1_bottom_warning_coor = self.CAM1_scissorBox['bottom']

        self.CAM1_points = [self.CAM1_roll_warning[f'point{num + 1}'] for num in range(self.CAM1_roll_warning['Count'])]
        self.CAM1_polygon = Polygon([eval(point) for point in self.CAM1_points])
        self.CAM1_np_roll = np.array([eval(point) for point in self.CAM1_points])

        self.CAM1_points_scissor = [self.CAM1_scissor_warning[f'point{num + 1}'] for num in range(self.CAM1_scissor_warning['Count'])]
        self.CAM1_polygon_scissor = Polygon([eval(point) for point in self.CAM1_points_scissor])
        self.CAM1_np_scissor = np.array([eval(point) for point in self.CAM1_points_scissor])

        self.CAM1_points_panel = [self.CAM1_panel_warning[f'point{num + 1}'] for num in range(self.CAM1_panel_warning['Count'])]
        self.CAM1_polygon_panel = Polygon([eval(point) for point in self.CAM1_points_panel])
        self.CAM1_np_panel = np.array([eval(point) for point in self.CAM1_points_panel])

        self.CAM1_points_cabin = [self.CAM1_cabin_warning[f'point{num + 1}'] for num in range(self.CAM1_cabin_warning['Count'])]
        self.CAM1_polygon_cabin = Polygon([eval(point) for point in self.CAM1_points_cabin])
        self.CAM1_np_cabin = np.array([eval(point) for point in self.CAM1_points_cabin])

        self.CAM1_points_gather = [self.CAM1_gather_warning[f'point{num + 1}'] for num in range(self.CAM1_gather_warning['Count'])]
        self.CAM1_polygon_gather = Polygon([eval(point) for point in self.CAM1_points_gather])
        self.CAM1_np_gather = np.array([eval(point) for point in self.CAM1_points_gather])

        self.CAM1_thresh = self.CAM1_lightbox['thresh']
        self.CAM1_rect_red = self.parse_rectangle(self.CAM1_lightbox['RectRed'])
        self.CAM1_rect_yellow = self.parse_rectangle(self.CAM1_lightbox['RectYellow'])
        self.CAM1_rect_green = self.parse_rectangle(self.CAM1_lightbox['RectGreen'])
        self.CAM1_rect_all = self.parse_rectangle(self.CAM1_lightbox['RectAll'])
    # Convert String to Int
    def parse_rectangle(self, rect_str):
        rect_str = rect_str.strip('()')
        return list(map(int, rect_str.split(',')))

    def check_cuda(self):
        h = torch.cuda.is_available()
        print("CUDA available:"+str(h))
        return h

    def load_model(self, model_path):
        model = YOLO(model_path)
        # model.fuse()
        device = torch.device(self.CAM1_device_setting)
        model.to(device)
        # model.half()
        return model

    def load_modelv8(self, model_path):
        model = YOLO(model_path)
        # model.fuse()
        device = torch.device(self.CAM1_device_setting)
        model.to(device)
        # model.half()
        return model


    def connect_camera(self):
        with open(self.CAM1_config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM1_general_config = config["general"]
            self.CAM1_ip_address = self.CAM1_general_config['source']
        cap = cv2.VideoCapture(self.CAM1_ip_address)
        if cap.isOpened():
            logging.exception('Camera connected')
            print("Reconnected the camera...")
            return cap
        else:
            time.sleep(2)
            return None
    # Calculate distance from 2 point
    def euclidean_distance(self, box1, box2):
        left1, top1, right1, bottom1 = box1
        left2, top2, right2, bottom2 = box2
        center1 = ((left1 + right1) / 2, (top1 + bottom1) / 2)
        center2 = ((left2 + right2) / 2, (top2 + bottom2) / 2)
        distance = np.sqrt((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2)
        return distance
    # Find close people
    def find_close_people(self, boxes, threshold=CLOSE_PEOPLE):
    # def find_close_people(self, boxes):
        close_boxes = set()
        for boxx in range(len(boxes)):
            close_count = 0
            current_close_boxes = []
            for boxy in range(len(boxes)):
                if boxx != boxy and self.euclidean_distance(boxes[boxx], boxes[boxy]) < threshold:
                    close_count += 1
                    current_close_boxes.append(tuple(boxes[boxy]))
            if close_count >= 2:
                close_boxes.add(tuple(boxes[boxx]))
                close_boxes.update(current_close_boxes)
        return close_boxes

    def find_max_score_img(self, img_dict):
        max_scores = []

        for im in img_dict.values():
            max_scores.append(im[1])
        num_max = np.argmax(max_scores)
        img_max_score = list(img_dict.values())[num_max][0]
        return img_max_score
    # Check case
    def check(self, mode="", cap=None, frame=None, img_dict=None):
        try:
            if mode == "gathering":
                if self.CAM1_duration / TIME_CLOSE_PEOPLE == 1:
                    self.post_alarm(self.CAM1_params_manyoperators, frame)
            elif mode == "noglove":
                if self.CAM1_check_glove > MIN_CHECK_FRAMES and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM1_params_glove, img_max_score)
                    self.CAM1_check_glove = 0
                    self.CAM1_noglove_img_check = {}
            elif mode == "touchpanel":
                # print(int(cap.get(cv2.CAP_PROP_POS_FRAMES)))
                # print(int(cap.get(cv2.CAP_PROP_FPS)))
                # nm = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS))
                # print(nm)
                # print(self.CAM1_check_touchpanel)
                if self.CAM1_check_touchpanel > MIN_CHECK_FRAMES and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    # print("check panel: ", self.CAM1_check_touchpanel)
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM1_params_touchpanel, img_max_score)
                    self.CAM1_check_touchpanel = 0
                    self.CAM1_touch_panel_img_check = {}
            elif mode == "roll":
                if self.CAM1_check_roll > MIN_CHECK_FRAMES and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM1_params_roll, img_max_score)
                    self.CAM1_check_roll = 0
                    self.CAM1_roll_img_check = {}
    
            elif mode == "cut":
                print(self.CAM1_check_cut)
                if self.CAM1_check_cut >= MIN_CHECK_FRAMES and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM1_params_cut, img_max_score)
                    self.CAM1_check_cut = 0
                    self.CAM1_cut_img_check = {}
            elif mode == "light":
                if self.CAM1_check_light > MIN_CHECK_FRAMES and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    # img_max_score = self.find_max_score_img(img_dict=img_dict)
                    # self.post_alarm(self.CAM1_params_lamp, img_max_score)
                    self.CAM1_check_light = 0
                    self.CAM1_light_img_check = {}

            if cap.get(cv2.CAP_PROP_POS_FRAMES) > 1 and int(cap.get(cv2.CAP_PROP_POS_FRAMES)) % int(cap.get(cv2.CAP_PROP_FPS)) == 1:
                self.CAM1_check_glove = 0
                self.CAM1_check_touchpanel = 0
                self.CAM1_check_roll = 0
                self.CAM1_check_cut = 0
                self.CAM1_check_light = 0

                self.CAM1_check_flashing = 0

                self.CAM1_noglove_img_check = {}
                self.CAM1_roll_img_check = {}
                self.CAM1_touch_panel_img_check = {}
                self.CAM1_cut_img_check = {}
                self.CAM1_light_img_check = {}
        except Exception as e:
            logging.error('Error at %s', 'Post alarm', exc_info=e)
    # Check light is On or Off
    def update_label_color(self, label, mean_abs, pre_mean_abs, threshold):
        if pre_mean_abs != 0:
            if mean_abs - pre_mean_abs > self.CAM1_thresh:
                label = True
            elif mean_abs - pre_mean_abs < -self.CAM1_thresh:
                label = False
        else:
            if mean_abs > threshold:
                label = True
            else:
                label = False
        return label
    # Run
    def draw_rectangles(self, frame, colors):
        cv2.rectangle(frame, (self.CAM1_rect_red[0], self.CAM1_rect_red[1]), (self.CAM1_rect_red[2], self.CAM1_rect_red[3]), colors["LIGHTRED"], 2)
        cv2.rectangle(frame, (self.CAM1_rect_yellow[0], self.CAM1_rect_yellow[1]), (self.CAM1_rect_yellow[2], self.CAM1_rect_yellow[3]), colors["LIGHTYELLOW"], 2)
        cv2.rectangle(frame, (self.CAM1_rect_green[0], self.CAM1_rect_green[1]), (self.CAM1_rect_green[2], self.CAM1_rect_green[3]), colors["LIGHTGREEN"], 2)
        cv2.rectangle(frame, (self.CAM1_rect_all[0], self.CAM1_rect_all[1]), (self.CAM1_rect_all[2], self.CAM1_rect_all[3]), colors["LIGHTALL"], 2)
        cv2.polylines(frame, [self.CAM1_np_roll], True, colors["FLOOR"], 2)
        cv2.polylines(frame, [self.CAM1_np_scissor], True, colors["SCISSORCHECK"], 2)
        cv2.polylines(frame, [self.CAM1_np_panel], True, colors["PANEL"], 2)
        cv2.polylines(frame, [self.CAM1_np_cabin], True, colors["CABIN"], 2)
        cv2.polylines(frame, [self.CAM1_np_gather], True, colors["GATHER"], 2)

    def calculate_light_means(self, frame, rect_comp):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.addWeighted(frame, 1, np.zeros(frame.shape, frame.dtype), 1, -50)
        mean_red_abs = np.mean(frame[self.CAM1_rect_red[1]:self.CAM1_rect_red[3], self.CAM1_rect_red[0]:self.CAM1_rect_red[2]])
        mean_yellow_abs = np.mean(frame[self.CAM1_rect_yellow[1]:self.CAM1_rect_yellow[3], self.CAM1_rect_yellow[0]:self.CAM1_rect_yellow[2]])
        mean_green_abs = np.mean(frame[self.CAM1_rect_green[1]:self.CAM1_rect_green[3], self.CAM1_rect_green[0]:self.CAM1_rect_green[2]])
        mean_comp = np.mean(frame[rect_comp[1]:rect_comp[3], rect_comp[0]:rect_comp[2]])
        return mean_red_abs, mean_yellow_abs, mean_green_abs, mean_comp

    # Post alarm parameter to Web
    def post_alarm(self, params, img):
        start_time = dt.datetime.now()
        self.logger_cam1.info(f"Start Time: {start_time}")

        img_base64 = cv2.imencode('.jpg', img)
        img_str = base64.b64encode(img_base64[1]).decode("utf-8")
        abnormal = requests.post(url=self.CAM1_apiabnormal, params=params)
        self.CAM1_code = abnormal.json()['code']

        end_time = dt.datetime.now()
        time_diff = end_time - start_time
        logging.info(f"Time taken for post request: {time_diff}")

        now = dt.datetime.now()
        dt_string = now.strftime("%d%m%Y%H%M%S") + '_cam1.jpg'
        params_save = {
            "AbnormalId" : self.CAM1_code,
            "ImageName" : dt_string,
            "ImageData" : "data:image/jpeg;base64," + img_str
        }
        requests.post(self.CAM1_apimedia, headers=self.CAM1_headers, data=json.dumps(params_save), timeout= 5)

    def process_results(self, results, frame_copy, frame_org, colors, model_keo, cap):
        try:
            num_persons = 0
            num_roll = 0
            count_peopleincabin = 0
            person_boxes = []
            rool_boxes = []
            person_boxes_track_gather = []
            outside_gather = []

            # Lamp handling
            # mean_red_abs, mean_yellow_abs, mean_green_abs = self.calculate_light_means(self.frame)
            # self.lightRed = self.update_label_color(self.lightRed, mean_red_abs, self.pre_mean_Red, 130)
            # self.lightYellow = self.update_label_color(self.lightYellow, mean_yellow_abs, self.pre_mean_Yellow, 130)
            # self.lightGreen = self.update_label_color(self.lightGreen, mean_green_abs, self.pre_mean_Green, 130)
            rect_comp = [1757, 290, 1777, 310]
            mean_red_abs, mean_yellow_abs, mean_green_abs, mean_comp = self.calculate_light_means(frame_org, rect_comp)
            mean_brightness = (mean_red_abs + mean_yellow_abs + mean_green_abs) / 3
            list_mean_all = [mean_red_abs, mean_yellow_abs, mean_green_abs]
            # print("y",mean_yellow_abs)
            # print("g",mean_green_abs)
            # print("r",mean_red_abs)
            if abs(mean_red_abs - min(list_mean_all)) >= 100:
                self.CAM1_lightRed = True
            else:
                self.CAM1_lightRed = False
            if abs(mean_yellow_abs - min(list_mean_all)) >= 100:
                self.CAM1_lightYellow = True
            else:
                self.CAM1_lightYellow = False
            if abs(mean_green_abs - min(list_mean_all)) >= 100:
                self.CAM1_lightGreen = True
            else:
                self.CAM1_lightGreen = False
            # print("___Green {}".format(abs(mean_green_abs - min(list_mean_all))))
            # print("___Yellow {}".format(abs(mean_yellow_abs - min(list_mean_all))))
            if abs(mean_red_abs - mean_brightness) < 20 and abs(mean_yellow_abs - mean_brightness) < 20 and abs(mean_green_abs - mean_brightness) < 20:
                if abs(mean_brightness - mean_comp) <= 50:
                    self.CAM1_lightRed =False
                    self.CAM1_lightYellow = False
                    self.CAM1_lightGreen = False
                else:
                    self.CAM1_lightRed =True
                    self.CAM1_lightYellow = True
                    self.CAM1_lightGreen = True

            if self.CAM1_lightRed:
                cv2.putText(frame_copy, "RED", (self.CAM1_rect_red[0] - 75, self.CAM1_rect_red[1] + 13), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=self.CAM1_colors["LIGHTRED"], thickness=2)
            if self.CAM1_lightGreen:
                cv2.putText(frame_copy, "GREEN", (self.CAM1_rect_green[0] - 75, self.CAM1_rect_green[1] + 13), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=self.CAM1_colors["LIGHTGREEN"], thickness=2)
            if self.CAM1_lightYellow:
                cv2.putText(frame_copy,"YELLOW", (self.CAM1_rect_yellow[0] - 75, self.CAM1_rect_yellow[1] + 13), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=self.CAM1_colors["LIGHTYELLOW"], thickness=2)

            # Check all light are on
            if self.CAM1_lightGreen and self.CAM1_lightRed and self.CAM1_lightYellow:
                if self.CAM1_post:
                    self.CAM1_check_light += 1
                    self.CAM1_light_img_check[self.CAM1_check_light] = [frame_copy, 0]
            else:
                if self.CAM1_post:
                    self.CAM1_check_light = 0
            if self.CAM1_lightYellow:
                if self.CAM1_post:
                    self.CAM1_check_flashing += 1

            # self.CAM1_pre_mean_Red = mean_red_abs
            # self.CAM1_pre_mean_Yellow = mean_yellow_abs
            # self.CAM1_pre_mean_Green = mean_green_abs

            for r in results:
                if r.boxes is None:
                    continue
                for result in r.boxes.data:
                    result = result.cpu().detach().numpy()
                    boxes = result[:4]
                    left, top, right, bottom = boxes

                    box_width = right - left
                    box_height = bottom - top
                    if  box_width > 6 and box_height > 6: # thu nho kich thuoc > 6px
                        left, top, right, bottom = left - 3, top - 3, right + 3, bottom +3
                    else:
                        # giu nguyen neu kich <= 6px
                        left, top, right, bottom = int(left), int(top), int(right), int(bottom)

                    classes = int(result[5])
                    scores = result[4]
                    # Class People
                    if classes == PEOPLE_CONSTANT and scores >= MIN_SCORE_CONSTANT:
                        # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                        # cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                        num_persons += 1
                        person_boxes.append(boxes)
                        # centerx_person = (left + right)/2
                        # centery_person = (top + bottom)/2
                        # if self.CAM1_polygon_cabin.contains(Point(left, top)):
                        #     count_peopleincabin += 1

                    # Class Gloved and NoGlove
                    # elif (classes == GLOVE_CONSTANT or classes == NOGLOVE_CONSTANT or classes == HAND_CONSTANT):
                    #     # if ((self.CAM1_left_warning_coor < left < self.CAM1_right_warning_coor) and (self.CAM1_top_warning_coor < top <self.CAM1_bottom_warning_coor)):
                    #     if self.CAM1_polygon_cabin.contains(Point(left, top)):
                    #         if classes == GLOVE_CONSTANT:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #                 cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #         elif classes == NOGLOVE_CONSTANT and scores > 0.7:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                    #                 cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                    #                 # if scores > 0.8:
                    #                 if self.CAM1_post:
                    #                     self.CAM1_check_glove += 1
                    #                     self.CAM1_noglove_img_check[self.CAM1_check_glove] = [frame_copy, scores]
                    #         elif classes == HAND_CONSTANT:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #                 cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #     x_panel = left
                    #     y_panel = top
                    #     p_panel = Point(x_panel, y_panel)
                    #     if self.CAM1_polygon_panel.contains(p_panel) or self.CAM1_polygon_panel.contains(Point(right, top)):
                            
                    #         if classes == GLOVE_CONSTANT:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #                 cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #         elif classes == NOGLOVE_CONSTANT:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                    #                 cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                    #         elif classes == HAND_CONSTANT:
                    #                 cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #                 cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                            
                    #         # if True:
                    #         cv2.putText(frame_copy, str("Touch Panel"), (int(left), int(bottom)),
                    #                             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                    #         if self.CAM1_lightGreen and (not self.CAM1_lightRed and not self.CAM1_lightYellow):
                    #             # if scores > 0.8:
                    #             if self.CAM1_post:
                    #                 self.CAM1_check_touchpanel += 1
                    #                 self.CAM1_touch_panel_img_check[self.CAM1_check_touchpanel] = [frame_copy, scores]
                            # else:
                            #     continue
                    # Roll
                    elif classes == ROLL_CONSTANT and scores> 0.7:
                        num_roll += 1
                        # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), (255,0,0), 1)
                        # cv2.putText(frame_copy, "Roll_{:.2f}%".format(scores * 100), (int(left), int(top)),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                        x_center_roll = (right + left)/2
                        y_center_roll = (top + bottom)/2
                        p = Point(x_center_roll, y_center_roll)
                        rool_boxes.append(boxes)
                        self.CAM1_dect_copper.append([x_center_roll, y_center_roll])

            if num_roll > 0:
                rect_boxes = self.CAM1_RollTracker.update(rool_boxes)
                for key, value in rect_boxes.items():
                    Cenx,CenY =value
                    dect_copper = self.CAM1_RollTracker.get_history(key)
                    bbox = self.CAM1_RollTracker.get_current_bbox(key)
                    left, top, right, bottom = bbox
                    cent_x, cent_y = (left + right)/2, (top + bottom)/2
                    if len(dect_copper) >= 5:
                        total_distance = 0
                        for i in range(-5, -1):
                            current_point = np.array(dect_copper[i])
                            past_point = np.array(dect_copper[i + 1])
                            total_distance += abs(np.linalg.norm(current_point - past_point))

                        distance = total_distance/4
                        p2 = Point(Cenx, CenY)
                        if self.CAM1_polygon.contains(p2) and distance <= 1:
                            distance_p_list = []
                            if len(person_boxes) > 0:
                                for box_p in person_boxes:
                                    lp, tp, rp, bp = box_p
                                    c_xp, c_yp = (lp + rp)/2, (tp + bp)/2
                                    distance_p = abs(np.linalg.norm(np.array([c_xp, c_yp]) - np.array([cent_x, cent_y])))
                                    distance_p_list.append(distance_p)
                                if min(distance_p_list) < 130:
                                    cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                                    cv2.putText(frame_copy, "Copper_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                                else:
                                    cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                    cv2.putText(frame_copy, "Copper on Floor_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                    if self.CAM1_post:
                                        self.CAM1_check_roll += 1
                                        self.CAM1_roll_img_check[self.CAM1_check_roll] = [frame_copy, scores]
                            else:
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                cv2.putText(frame_copy, "Copper on Floor_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                if self.CAM1_post:
                                    self.CAM1_check_roll += 1
                                    self.CAM1_roll_img_check[self.CAM1_check_roll] = [frame_copy, scores]
                            # cv2.putText(frame_copy, str("Copper on Floor_{}").format(key ), (int(Cenx), int(CenY)),
                            #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                            # cv2.rectangle(frame_copy, (int(Cenx -5), int(CenY-5 )), (int(Cenx+5), int(CenY+5)), colors["NG"], 2)

                        else:
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                            cv2.putText(frame_copy, "Copper_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                            # cv2.rectangle(frame_copy, (int(Cenx -5), int(CenY-5 )), (int(Cenx+5), int(CenY+5)), colors["OBJECT"], 2)
                            # cv2.putText(frame_copy, str("Copper_{}".format(key )), (int(Cenx), int(CenY)),
                            #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OK"], 2)
            if num_persons > 0:
                recttrack = self.CAM1_Tracker.update(person_boxes)
                num_persons = 0
                self.CAM1_text_color = (255, 0, 0)
                self.CAM1_gather_color = (0, 0, 255)

                if self.CAM1_duration / TIME_CLOSE_PEOPLE >= 1:
                    self.CAM1_text_color = (0, 0, 255)

                for key, value in recttrack.items():
                    num_persons += 1
                    # print(key)
                    # print(value)
                    Cenx,CenY =value
                    # box_ = self.CAM1_Tracker.get_current_bbox(key)
                    
                    # self.CAM1_text_color = (0, 0, 255)
                    # cv2.putText(frame_copy, "People", (int(Cenx -20), int(CenY-20)),
                    #                     cv2.FONT_HERSHEY_SIMPLEX, 1, self.CAM1_text_color, 2)
                    bbox = self.CAM1_Tracker.get_current_bbox(key)
                    left_, top_, right_, bottom_ = bbox
                    if self.CAM1_polygon_gather.contains(Point(Cenx, CenY)) or self.CAM1_polygon_gather.contains(Point(left_, top_)) or self.CAM1_polygon_gather.contains(Point(left_, bottom_)) or self.CAM1_polygon_gather.contains(Point(right_, top_)) or self.CAM1_polygon_gather.contains(Point(right_, bottom_)):
                        person_boxes_track_gather.append(bbox)
                    else:
                        outside_gather.append(bbox)
                    # TrajectoryPerson = self.CAM1_Tracker.get_history(key)
                close_people_boxes = self.find_close_people(person_boxes_track_gather)
                # close_people_boxes = [tuple(box) for i, box in enumerate(person_boxes_track) if close_people_counts[i] >= 2]
                for i, box in enumerate(person_boxes_track_gather):
                    left, top, right, bottom = box
                    if self.CAM1_duration / TIME_CLOSE_PEOPLE >= 1:
                        if tuple(box) in close_people_boxes:
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                            cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                        else:
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                            cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                    else:
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                            cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)

                if len(close_people_boxes) >= NUM_PEOPLE_CHECK:
                    if self.CAM1_start_time is None:
                        self.CAM1_start_time = time.time()
                else:
                    self.CAM1_start_time = None

                if self.CAM1_start_time is not None:
                    self.CAM1_duration = int(time.time() - self.CAM1_start_time)
                    cv2.putText(frame_copy, f"Duration: {self.CAM1_duration}s", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    self.CAM1_text_color = (0, 0, 255)
                else:
                    self.CAM1_text_color = (255, 255, 255)
                    # Draw line close people
                    # for (box1, box2) in close_people:
                    #     x1, y1, x2, y2 = box1
                    #     x3, y3, x4, y4 = box2
                    #     center1 = (int(x1 + x2) // 2, int(y1 + y2) // 2)
                    #     center2 = (int(x3 + x4) // 2, int(y3 + y4) // 2)
                    #     frame_copy = cv2.line(frame_copy, center1, center2, (0, 0, 255), 2)
                for box in outside_gather:
                    left, top, right, bottom = box
                    cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                person_boxes = []
            # frame_copy = cv2.putText(frame_copy, f'Operators: {len(close_people_boxes)}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, self.CAM1_text_color, 2, cv2.LINE_AA)
            # Detect Scissor
            # print(count_peopleincabin)
            # if num_persons > 0:
            # if self.CAM1_lightGreen and not self.CAM1_lightRed and not self.CAM1_lightYellow:
                frame_keo = frame_org[self.CAM1_top_warning_coor:self.CAM1_bottom_warning_coor, self.CAM1_left_warning_coor:self.CAM1_right_warning_coor]
                # cv2.imwrite("frame_keo.jpg", frame_keo)
                # for i in range(0, 1):
                #     break
                # frame_keo_copy = frame_keo.copy()
                # if self.CAM1_general_config['save']:
                #     output_keo = self.CAM1_output_path + "/Scissor"
                #     if not os.path.exists(output_keo):
                #         os.makedirs(output_keo)
                #     imgOrg_path = os.path.join(output_keo,  os.path.basename(self.CAM1_ip_address).split(".mp4")[0] + f"_{int(self.CAM1_cap.get(cv2.CAP_PROP_POS_FRAMES))}_org.jpg")
                #     cv2.imwrite(imgOrg_path, frame_keo)
                if True:
                    results_keo = model_keo(source=frame_keo, conf=0.5, iou=0.5, agnostic_nms=True)
                    # add 
                    for result_keo in results_keo:
                        for result in result_keo.boxes.data:
                            result = result.cpu().detach().numpy()
                            scores_2 = result[4]
                            classes_2 = int(result[5])
                            if (classes_2 == GLOVE_CONSTANT or classes_2 == NOGLOVE_CONSTANT or classes_2 == HAND_CONSTANT):
                                boxes = result[:4].astype(int)
                                left_, top_, right_, bottom_ = boxes
                                # if ((self.CAM1_left_warning_coor < left < self.CAM1_right_warning_coor) and (self.CAM1_top_warning_coor < top <self.CAM1_bottom_warning_coor)):
                                if self.CAM1_polygon_cabin.contains(Point(left_ + self.CAM1_left_warning_coor, top_ + self.CAM1_top_warning_coor)):
                                    
                                    if classes_2 == GLOVE_CONSTANT:
                                            cv2.rectangle(frame_copy, (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)), colors["OBJECT"], 2)
                                            cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                                    elif classes_2 == NOGLOVE_CONSTANT and scores_2 > 0.8:
                                            cv2.rectangle(frame_copy, (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)), colors["NG"], 2)
                                            cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                                            # print(self.CAM1_check_glove)
                                            # if scores_2 > 0.8:
                                            if self.CAM1_post:
                                                # print(self.CAM1_check_glove)
                                                self.CAM1_check_glove += 1
                                                self.CAM1_noglove_img_check[self.CAM1_check_glove] = [frame_copy, scores_2]
                                    elif classes_2 == HAND_CONSTANT:
                                            cv2.rectangle(frame_copy, (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)), colors["OBJECT"], 2)
                                            cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                                x_panel = left_ + self.CAM1_left_warning_coor
                                y_panel = top_ + self.CAM1_top_warning_coor

                                x_panel_cent = ((left_ + self.CAM1_left_warning_coor) + (right_ + self.CAM1_left_warning_coor))/2
                                y_panel_cent = ((top_ + self.CAM1_top_warning_coor) + (bottom_ + self.CAM1_top_warning_coor))/2

                                p_panel = Point(x_panel, y_panel)
                                p_cent_panel = Point(x_panel_cent, y_panel_cent)
                                if self.CAM1_polygon_panel.contains(p_panel) or self.CAM1_polygon_panel.contains(p_cent_panel): 
                                    # if self.CAM1_lightGreen and (not self.CAM1_lightRed and not self.CAM1_lightYellow):
                                        cv2.rectangle(frame_copy, (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)), colors["NG"], 2)
                                        cv2.putText(frame_copy, str("Touch Panel"), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)),
                                                        cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                        # if True:
                                        #    cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                                        # if classes_2 == GLOVE_CONSTANT:
                                        #     cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                                        # elif classes_2 == NOGLOVE_CONSTANT:
                                        #     cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                                        # elif classes_2 == HAND_CONSTANT:
                                        #     cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores_2*100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)  
                                        # cv2.putText(frame_copy, str("Touch Panel"), (int(right_ + self.CAM1_left_warning_coor, int(bottom_ + self.CAM1_top_warning_coor))),
                                        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                        if self.CAM1_post:
                                                # print("check panel _1: ", self.CAM1_check_touchpanel)
                                                self.CAM1_check_touchpanel += 1
                                                self.CAM1_touch_panel_img_check[self.CAM1_check_touchpanel] = [frame_copy, scores_2]
                            elif classes_2 == SCISSOR_LABEL and scores_2 > 0.75:
                                boxes_keo = result[:4]
                                left_, top_, right_, bottom_ = boxes_keo
                                x_scissor = left_ + self.CAM1_left_warning_coor
                                y_scissor = top_ + self.CAM1_top_warning_coor
                                p_scissor = Point(x_scissor, y_scissor)
                                if self.CAM1_polygon_scissor.contains(p_scissor):
                                    # classes_keo = SCISSOR_LABEL
                                    cv2.rectangle(frame_copy, (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)), (int(right_ + self.CAM1_left_warning_coor), int(bottom_ + self.CAM1_top_warning_coor)), colors["NG"], 2)
                                    cv2.putText(frame_copy, "Scissor_{:.2f}%".format(scores_2 * 100), (int(left_ + self.CAM1_left_warning_coor), int(top_ + self.CAM1_top_warning_coor)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                    if self.CAM1_post:
                                        self.CAM1_check_cut += 1
                                        self.CAM1_cut_img_check[self.CAM1_check_cut] = [frame_copy, scores_2]


            if self.CAM1_post:
                # Call Function Check Case
                if self.CAM1_lightGreen:
                # if True:
                # print(self.CAM1_check_glove)
                    self.check(mode="touchpanel", cap=cap, frame=frame_org, img_dict=self.CAM1_touch_panel_img_check)
                    self.check(mode="noglove", cap=cap, frame=frame_org, img_dict=self.CAM1_noglove_img_check)
                    self.check(mode="cut", cap=cap, frame=frame_org, img_dict=self.CAM1_cut_img_check)
                self.check(mode="gathering", cap=cap, frame=frame_copy, img_dict=frame_org)
                self.check(mode="roll", cap=cap, frame=frame_org, img_dict=self.CAM1_roll_img_check)
                # self.check(mode="light", cap=cap, frame=frame_org, img_dict=self.CAM1_light_img_check)
                if self.CAM1_check_light > MIN_CHECK_FRAMES:
                    cv2.putText(frame_copy, "light is moved", (int(20), int(20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)

            if self.CAM1_general_config['save']:
                self.CAM1_writer.write(frame_copy)
        except Exception as e:
            logging.error('Error at %s', 'Processing', exc_info=e)
        return frame_copy
    