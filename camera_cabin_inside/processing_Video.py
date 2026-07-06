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
import threading
import logging               #import lib for write log
import datetime as dt       #import datetime library
from dateutil import parser      #import lib for read config
from .CameraCaptureWorker import FreshestFrame

sys.path.insert(0, './yolov10')
from datetime import datetime
from ultralytics import YOLOv10
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

GLOVE_CONSTANT = 0 # Class Glove
NOGLOVE_CONSTANT = 1 # Class NoGlove
HAND_CONSTANT = 2 # Class Hand
SCISSOR_CONSTANT = 3 # Class Scissor
ROLL_CONSTANT = 4 # Class Roll

MIN_SCORE_CONSTANT = 0.5 # Score to check 
MIN_CHECK_FRAMES = 7 # Number of frames to check
# MIN_SCORE_CONSTANT_CHECK = 0.8
sleep_interval = 0.05   
# logging.basicConfig(filename= 'post_time_alarm_cam2.log', level= logging.INFO,
#                     format= '%(asctime)s - %(message)s')
class SSGVision:
    def __init__(self, config_path):
        self.logger_cam2 = logging.getLogger('cam2')
        self.logger_cam2.setLevel(logging.INFO)
        file_handler_cam2 = logging.FileHandler('post_time_alarm_cam2.log')
        file_handler_cam2.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.logger_cam2.addHandler(file_handler_cam2)
        
        self.CAM2_config_path = config_path      
        # Read parameter from config path
        with open(config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
        self.CAM2_general_config = config["general"]
        self.CAM2_UseThreadCap = self.CAM2_general_config["ThreadCap"]
        self.CAM2_model_all_config = config["model_All"]
        self.CAM2_lightbox = config["lightBox"]
        self.CAM2_glovedBox = config["glovedBox"]
        self.CAM2_scissorBox = config["scissorBox"]
        self.CAM2_panel_warning = config["accept_panel"]
        self.CAM2_roll_warning = config["accept_roll"]
        self.CAM2_api = config["api"]
        self.CAM2_MaxRetry = int(self.CAM2_general_config['MaxRetry'])
        self.CAM2_MaxRetryAPI = int(self.CAM2_general_config['MaxRetryAPI'])
        self.CAM2_ip_address = self.CAM2_general_config['source']
        self.CAM2_auto_restart = self.CAM2_general_config['autoRestart']
        self.CAM2_output_path = self.CAM2_general_config['output']
        self.CAM2_device_setting = self.CAM2_general_config['device']
        self.CAM2_imgsz = self.CAM2_model_all_config['imgsz']
        self.CAM2_apicamsetting = self.CAM2_api["API"] + self.CAM2_api["CAMERA_SETTING"]
        self.CAM2_apiabnormal = self.CAM2_api["API"] + self.CAM2_api["ABNORMAL"]
        self.CAM2_apimedia = self.CAM2_api["API"] + self.CAM2_api["MEDIA"]
        self.CAM2_post = self.CAM2_api["POST"]

        # Load model detect
        self.CAM2_model_All = self.load_model(self.CAM2_model_all_config["weights"])
        # Record Camera
        self.CAM2_desired_width = 1920
        self.CAM2_desired_height = 1080
        self.CAM2_writer = self.setup_video_writer()
        self.CAM2_results = None

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
        # Initialize required parameter for check light indicator 
        self.CAM2_pre_mean_Red = 0
        self.CAM2_pre_mean_Green = 0
        self.CAM2_pre_mean_Yellow = 0
        self.CAM2_lightRed = False
        self.CAM2_lightGreen = False
        self.CAM2_lightYellow = False

        # Initialize required parameter for check (no use glove, roll on the floor and hand touch Panel) 
        self.CAM2_check_glove = 0
        self.CAM2_check_touchpanel = 0
        self.CAM2_check_cut = 0
        self.CAM2_count_light = 0    
        self.CAM2_check_roll = 0

        self.CAM2_check_flashing = 0

        self.CAM2_noglove_img_check = {}
        self.CAM2_roll_img_check = {}
        self.CAM2_touch_panel_img_check = {}
        self.CAM2_cut_img_check = {}
        self.CAM2_light_img_check = {}

        # Initialize other parameter
        self.CAM2_start_time = None
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
    # Read parameter from Camera
    def readCam_api(self, api):
        try:
            response = requests.get(api)
            data = response.json()
            return data
        except:
            time.sleep(2)
            return None
    
    def set_params(self, data_cam):
        self.CAM2_params_glove = {
            'Camip':data_cam["pkid"],
            'AbnormalType': 1,
            'AbnormalDuration': 1
        }
        self.CAM2_params_roll = {
            'Camip':data_cam["pkid"],
            'AbnormalType': 2,
            'AbnormalDuration': 1
        }
        self.CAM2_params_touchpanel = {
            'Camip':data_cam["pkid"],
            'AbnormalType': 3,
            'AbnormalDuration': 1
        }
        self.CAM2_params_cut = {
            'Camip':data_cam["pkid"],
            'AbnormalType': 4,
            'AbnormalDuration': 1
        }
        self.CAM2_params_lamp = {
            'Camip':data_cam["pkid"],
            'AbnormalType': 6,
            'AbnormalDuration': 1
        }
        self.CAM2_headers = {
                'Content-Type': 'application/json'
            }
    def set_coordinates(self):
        # self.CAM2_left_glove_coor = self.CAM2_glovedbox['left']
        # self.CAM2_top_glove_coor = self.CAM2_glovedbox['top']
        # self.CAM2_right_glove_coor = self.CAM2_glovedbox['right']
        # self.CAM2_bottom_glove_coor = self.CAM2_glovedbox['bottom']

        self.CAM2_points_gloved = [self.CAM2_glovedBox[f'point{num + 1}'] for num in range(self.CAM2_glovedBox['Count'])]
        self.CAM2_polygon_gloved = Polygon([eval(point) for point in self.CAM2_points_gloved])
        self.CAM2_np_gloved = np.array([eval(point) for point in self.CAM2_points_gloved])

        self.CAM2_points_scissor = [self.CAM2_scissorBox[f'point{num + 1}'] for num in range(self.CAM2_scissorBox['Count'])]
        self.CAM2_polygon_scissor = Polygon([eval(point) for point in self.CAM2_points_scissor])
        self.CAM2_np_scissor = np.array([eval(point) for point in self.CAM2_points_scissor])          

        self.CAM2_points_panel = [self.CAM2_panel_warning[f'point{num + 1}'] for num in range(self.CAM2_panel_warning['Count'])]
        self.CAM2_polygon_panel = Polygon([eval(point) for point in self.CAM2_points_panel])
        self.CAM2_np_panel = np.array([eval(point) for point in self.CAM2_points_panel])  

        self.CAM2_points_roll = [self.CAM2_roll_warning[f'point{num + 1}'] for num in range(self.CAM2_roll_warning['Count'])]
        self.CAM2_polygon_roll = Polygon([eval(point) for point in self.CAM2_points_roll])
        self.CAM2_np_roll = np.array([eval(point) for point in self.CAM2_points_roll])                

        self.CAM2_thresh = self.CAM2_lightbox['thresh']
        self.CAM2_rect_red = self.parse_rectangle(self.CAM2_lightbox['RectRed'])
        self.CAM2_rect_yellow = self.parse_rectangle(self.CAM2_lightbox['RectYellow'])
        self.CAM2_rect_green = self.parse_rectangle(self.CAM2_lightbox['RectGreen'])
        self.CAM2_rect_all = self.parse_rectangle(self.CAM2_lightbox['RectAll'])

        # self.CAM2_rect_red2 = self.parse_rectangle(self.CAM2_lightbox['RectRed2'])
        # self.CAM2_rect_yellow2 = self.parse_rectangle(self.CAM2_lightbox['RectYellow2'])
        # self.CAM2_rect_green2 = self.parse_rectangle(self.CAM2_lightbox['RectGreen2'])
        # self.CAM2_rect_all2 = self.parse_rectangle(self.CAM2_lightbox['RectAll2'])  

    # Convert String to Int
    def parse_rectangle(self, rect_str):
        rect_str = rect_str.strip('()')
        return list(map(int, rect_str.split(',')))

    def check_cuda(self):
        h = torch.cuda.is_available()
        print("CUDA available:"+str(h))
        return h
    
    def load_model(self, model_path):
        model = YOLOv10(model_path)
        # model.fuse()
        device = torch.device(self.CAM2_device_setting)
        model.to(device)
        # model.half()
        return model
    
    def connect_camera(self):
        with open(self.CAM2_config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM2_general_config = config["general"]
            self.CAM2_ip_address = self.CAM2_general_config['source']
        cap = cv2.VideoCapture(self.CAM2_ip_address)
        if cap.isOpened():
            logging.exception('Camera connected')
            print("Reconnected the camera...")
            return cap
        else:
            time.sleep(2)
            return None
        
    def setup_video_writer(self):
        if self.CAM2_general_config['save']:
            pass
        else:
            return None
        filename = os.path.basename(self.CAM2_output_path)
        save_path = os.path.dirname(self.CAM2_output_path)
        vid_path = os.path.join(save_path, filename+ ".avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(vid_path, fourcc, 24, (self.CAM2_desired_width, self.CAM2_desired_height))
        if not writer.isOpened():
            print(f"Can't Open VideoWriter With Path: {vid_path}")
        return writer
    def find_max_score_img(self, img_dict):
        max_scores = []
        for im in img_dict.values():
            max_scores.append(im[1])
        num_max = np.argmax(max_scores)
        img_max_score = list(img_dict.values())[num_max][0]
        return img_max_score
        
    # Check case
    def check(self, mode="", cap=None, img_dict=None):
        try:
            if mode == "noglove":
                if self.CAM2_check_glove > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM2_params_glove, img_max_score)
                    self.CAM2_check_glove = 0
                    self.CAM2_noglove_img_check = {}
            elif mode == "touchpanel":
                if self.CAM2_check_touchpanel > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM2_params_touchpanel, img_max_score)
                    self.CAM2_check_touchpanel = 0 
                    self.CAM2_touch_panel_img_check = {}
            elif mode == "cut":
                if self.CAM2_check_cut > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM2_params_cut, img_max_score)
                    self.CAM2_check_cut = 0  
                    self.CAM2_cut_img_check = {}
            elif mode == "roll":
                if self.CAM2_check_roll > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM2_params_roll, img_max_score)
                    self.CAM2_check_roll = 0
                    self.CAM2_roll_img_check = {}
            elif mode =='light':
                if self.CAM2_count_light > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM2_params_lamp, img_max_score)
                    self.CAM2_count_light = 0  
                    self.CAM2_light_img_check = {}
            if cap.get(cv2.CAP_PROP_POS_FRAMES) > 1 and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 1:
                self.CAM2_check_glove = 0
                self.CAM2_check_touchpanel = 0
                self.CAM2_check_cut = 0
                self.CAM2_count_light = 0
                self.CAM2_check_roll = 0

                self.CAM2_check_flashing = 0

                self.CAM2_noglove_img_check = {}
                self.CAM2_roll_img_check = {}
                self.CAM2_touch_panel_img_check = {}
                self.CAM2_cut_img_check = {}
                self.CAM2_light_img_check = {}
        except Exception as e:
            logging.error('Error at %s', 'Post alarm', exc_info=e)   
    # Check light is On or Off
    def update_label_color(self, label, mean_abs, pre_mean_abs, threshold):
        if pre_mean_abs != 0:
            if mean_abs - pre_mean_abs > self.CAM2_thresh:
                label = True
            elif mean_abs - pre_mean_abs < -self.CAM2_thresh:
                label = False
        else:
            if mean_abs > threshold:
                label = True
            else:
                label = False
        return label    
    def draw_rectangles(self, frame, colors):
        cv2.polylines(frame, [self.CAM2_np_panel], True, colors["PANEL"], 2)
        cv2.polylines(frame, [self.CAM2_np_roll], True, colors["FLOOR"], 2)
        cv2.polylines(frame, [self.CAM2_np_scissor], True, colors["SCISSORCHECK"], 2)
        # cv2.rectangle(frame, (self.CAM2_left_glove_coor, self.CAM2_top_glove_coor), (self.CAM2_right_glove_coor, self.CAM2_bottom_glove_coor), colors["GLOVECHECK"], 2)
        cv2.polylines(frame, [self.CAM2_np_gloved], True, colors["GLOVECHECK"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_red[0], self.CAM2_rect_red[1]), (self.CAM2_rect_red[2], self.CAM2_rect_red[3]), colors["LIGHTRED"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_yellow[0], self.CAM2_rect_yellow[1]), (self.CAM2_rect_yellow[2], self.CAM2_rect_yellow[3]), colors["LIGHTYELLOW"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_green[0], self.CAM2_rect_green[1]), (self.CAM2_rect_green[2], self.CAM2_rect_green[3]), colors["LIGHTGREEN"], 2)
        cv2.rectangle(frame, (self.CAM2_rect_all[0], self.CAM2_rect_all[1]), (self.CAM2_rect_all[2], self.CAM2_rect_all[3]), colors["LIGHTALL"], 2)

        # cv2.rectangle(frame, (self.CAM2_rect_red2[0], self.CAM2_rect_red2[1]), (self.CAM2_rect_red2[2], self.CAM2_rect_red2[3]), colors["LIGHTRED"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_yellow2[0], self.CAM2_rect_yellow2[1]), (self.CAM2_rect_yellow2[2], self.CAM2_rect_yellow2[3]), colors["LIGHTYELLOW"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_green2[0], self.CAM2_rect_green2[1]), (self.CAM2_rect_green2[2], self.CAM2_rect_green2[3]), colors["LIGHTGREEN"], 2)
        # cv2.rectangle(frame, (self.CAM2_rect_all2[0], self.CAM2_rect_all2[1]), (self.CAM2_rect_all2[2], self.CAM2_rect_all2[3]), colors["LIGHTALL"], 2)

    def calculate_light_means(self, frame, rect_comp):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_red_abs = np.mean(frame[self.CAM2_rect_red[1]:self.CAM2_rect_red[3], self.CAM2_rect_red[0]:self.CAM2_rect_red[2]])
        mean_yellow_abs = np.mean(frame[self.CAM2_rect_yellow[1]:self.CAM2_rect_yellow[3], self.CAM2_rect_yellow[0]:self.CAM2_rect_yellow[2]])
        mean_green_abs = np.mean(frame[self.CAM2_rect_green[1]:self.CAM2_rect_green[3], self.CAM2_rect_green[0]:self.CAM2_rect_green[2]])
        mean_comp = np.mean(frame[rect_comp[1]:rect_comp[3], rect_comp[0]:rect_comp[2]])
        return mean_red_abs, mean_yellow_abs, mean_green_abs, mean_comp
    
    # Post alarm parameter to Web
    def post_alarm(self, params, img):
        start_time = dt.datetime.now()
        
        self.logger_cam2.info(f"Start time: {start_time}")

        img_base64 = cv2.imencode('.jpg', img)
        img_str = base64.b64encode(img_base64[1]).decode("utf-8") 
        abnormal = requests.post(url=self.CAM2_apiabnormal, params=params)
        self.CAM2_code = abnormal.json()['code']

        end_time = dt.datetime.now()
        time_diff = end_time - start_time
        self.logger_cam2.info(f"Time taken for post request: {time_diff}")

        now = dt.datetime.now()
        dt_string = now.strftime("%d%m%Y%H%M%S") + '_cam2.jpg' 
        params_save = {
            "AbnormalId" : self.CAM2_code,
            "ImageName" : dt_string,
            "ImageData" : "data:image/jpeg;base64," + img_str
        }
        requests.post(self.CAM2_apimedia, headers=self.CAM2_headers, data=json.dumps(params_save), timeout= 5)

    def process_results(self, results, frame_copy, frame_org, colors, cap):
        try:
            # Lamp handling
            # mean_red_abs, mean_yellow_abs, mean_green_abs = self.calculate_light_means(self.frame)
            # self.lightRed = self.update_label_color(self.lightRed, mean_red_abs, self.pre_mean_Red, 130)
            # self.lightYellow = self.update_label_color(self.lightYellow, mean_yellow_abs, self.pre_mean_Yellow, 130)
            # self.lightGreen = self.update_label_color(self.lightGreen, mean_green_abs, self.pre_mean_Green, 130)
            rect_comp = [1757, 290, 1777, 310]
            mean_red_abs, mean_yellow_abs, mean_green_abs, mean_comp = self.calculate_light_means(frame_org, rect_comp)
            mean_brightness = (mean_red_abs + mean_yellow_abs + mean_green_abs) / 3
            list_mean_all = [mean_red_abs, mean_yellow_abs, mean_green_abs]

            if abs(mean_red_abs - min(list_mean_all)) >= 50:
                self.CAM2_lightRed = True
            else:
                self.CAM2_lightRed = False
            if abs(mean_yellow_abs - min(list_mean_all)) >= 50:
                self.CAM2_lightYellow = True 
            else:
                self.CAM2_lightYellow = False       
            if abs(mean_green_abs - min(list_mean_all)) >= 50:
                self.CAM2_lightGreen = True  
            else:
                self.CAM2_lightGreen = False

            if abs(mean_red_abs - mean_brightness) < 20 and abs(mean_yellow_abs - mean_brightness) < 20 and abs(mean_green_abs - mean_brightness) < 20:
                if abs(mean_brightness - mean_comp) <= 50:
                    self.CAM2_lightRed =False
                    self.CAM2_lightYellow = False
                    self.CAM2_lightGreen = False
                else:
                    self.CAM2_lightRed =True
                    self.CAM2_lightYellow = True
                    self.CAM2_lightGreen = True
                    
            if self.CAM2_lightRed:
                cv2.putText(frame_copy, "RED", (self.CAM2_rect_red[0], self.CAM2_rect_red[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=self.CAM2_colors["LIGHTRED"], thickness=2)
            if self.CAM2_lightGreen:
                cv2.putText(frame_copy, "GREEN", (self.CAM2_rect_green[0], self.CAM2_rect_green[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=self.CAM2_colors["LIGHTGREEN"], thickness=2)
            if self.CAM2_lightYellow:
                cv2.putText(frame_copy,"YELLOW", (self.CAM2_rect_yellow[0], self.CAM2_rect_yellow[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=self.CAM2_colors["LIGHTYELLOW"], thickness=2)
            
            if self.CAM2_lightGreen and self.CAM2_lightRed and self.CAM2_lightYellow:
                if self.CAM2_post:
                    self.CAM2_count_light += 1
                    self.CAM2_light_img_check[self.CAM2_count_light] = [frame_copy, 0]

            if self.CAM2_lightYellow:
                if self.CAM2_post:
                    self.CAM2_check_flashing += 1
                
            # self.CAM2_pre_mean_Red = mean_red_abs
            # self.CAM2_pre_mean_Yellow = mean_yellow_abs
            # self.CAM2_pre_mean_Green = mean_green_abs
            roll_count = []
            glove_count = []
            noglove_count = []
            scissor_in_area = 0
            glove_in_area = 0
            noglove_in_area = 0
            for r in results:
                if r.boxes is None:
                    continue
                for result in r.boxes.data:
                    result = result.cpu().detach().numpy()
                    boxes = result[:4]
                    left, top, right, bottom = boxes
                    classes = int(result[5])
                    scores = result[4]
    
                    # Class Gloved and NoGlove and Hand
                    # if (classes == GLOVE_CONSTANT or classes == NOGLOVE_CONSTANT or classes == HAND_CONSTANT):
                    #     p_glove = Point(left, top)
                    #     if self.CAM2_polygon_gloved.contains(p_glove):
                    #         if classes == GLOVE_CONSTANT:
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #             cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #         elif classes == NOGLOVE_CONSTANT and scores > 0.8:
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                    #             cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                    #                     # if scores > 0.8:
                    #             if self.CAM2_post:        
                    #                 self.CAM2_check_glove += 1
                    #                 self.CAM2_noglove_img_check[self.CAM2_check_glove] = [frame_copy, scores]
                    #         else:
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #             cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)                        
                    #         # x_center_panel = (left + right)/2
                    #         # y_center_panel = top + (right - left)/2
                    #     x_center_panel = (left + right)/2
                    #     y_center_panel = top
                    #     p = Point(x_center_panel, y_center_panel)
                    #     if self.CAM2_polygon_panel.contains(p):
                    #         if classes == GLOVE_CONSTANT:    
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #             cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #         elif classes == NOGLOVE_CONSTANT and scores > 0.8:
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                    #             cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                    #         else:
                    #             cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    #             cv2.putText(frame_copy, "Hand_{:.2f}%".format(scores*100), (int(left), int(top)),
                    #                         cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                    #             # if self.CAM2_lightGreen and not self.CAM2_lightRed and not self.CAM2_lightYellow:
                    #         cv2.putText(frame_copy, str("Touch Panel"), (int(right), int(bottom)),
                    #                                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                    #         if self.CAM2_post:
                    #             self.CAM2_check_touchpanel += 1
                    #             self.CAM2_touch_panel_img_check[self.CAM2_check_touchpanel] = [frame_copy, scores]
                    
                    if (classes == GLOVE_CONSTANT or classes == NOGLOVE_CONSTANT or classes == HAND_CONSTANT):
                        p_glove = Point(left, top)

                        x_center_panel = (left + right)/2
                        y_center_panel = top
                        p = Point(x_center_panel, y_center_panel)

                        if classes == GLOVE_CONSTANT:
                            glove_count.append([left, top, right, bottom])
                            # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                            # cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                            #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                            if self.CAM2_polygon_gloved.contains(p_glove):
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                                cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)                                
                                glove_in_area += 1
                            elif self.CAM2_polygon_panel.contains(p):
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                # cv2.putText(frame_copy, "Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                                #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors['OBJECT'], 2)
                                cv2.putText(frame_copy, "Touchpanel", (int(left), int(bottom)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                                if self.CAM2_post:
                                    self.CAM2_check_touchpanel += 1
                                    self.CAM2_touch_panel_img_check[self.CAM2_check_touchpanel] = [frame_copy, scores]
                        elif classes == NOGLOVE_CONSTANT and scores > 0.8:
                            noglove_count.append([left, top, right, bottom])
                            # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                            # cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                            #                 cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)   
                            if self.CAM2_polygon_gloved.contains(p_glove):
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                cv2.putText(frame_copy, "No Glove_{:.2f}%".format(scores*100), (int(left), int(top)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                                noglove_in_area += 1
                                if self.CAM2_post:        
                                    self.CAM2_check_glove += 1
                                    self.CAM2_noglove_img_check[self.CAM2_check_glove] = [frame_copy, scores]
                            elif self.CAM2_polygon_panel.contains(p) or self.CAM2_polygon_panel.contains(Point(left, top)) or self.CAM2_polygon_panel.contains(Point(right, top)):
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                cv2.putText(frame_copy, "Touchpanel", (int(left), int(bottom)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                                if self.CAM2_post:
                                    self.CAM2_check_touchpanel += 1
                                    self.CAM2_touch_panel_img_check[self.CAM2_check_touchpanel] = [frame_copy, scores]
                    # Class Scissor
                    elif classes == SCISSOR_CONSTANT and scores > 0.875:
                        scissor_in_area += 1
                        x_center_scissor = left
                        y_center_scissor = top

                        p_scissor = Point(x_center_scissor, y_center_scissor)
                        if self.CAM2_polygon_scissor.contains(p_scissor):
                        # if self.CAM2_lightGreen and not self.CAM2_lightRed and not self.CAM2_lightYellow:
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                            cv2.putText(frame_copy, "Scissor_{:.2f}%".format(scores*100), (int(left), int(top)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                            if self.CAM2_post:
                                self.CAM2_check_cut += 1
                                self.CAM2_cut_img_check[self.CAM2_check_cut] = [frame_copy, scores]

                    elif classes == ROLL_CONSTANT:
                        roll_count.append([left, top, right, bottom])
                        x_center_roll = left
                        y_center_roll = bottom

                        p_roll = Point(x_center_roll, y_center_roll)
                        if self.CAM2_polygon_roll.contains(p_roll):
                            cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                            cv2.putText(frame_copy, "Roll_{:.2f}%".format(scores*100), (int(left), int(top)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, colors['NG'], 2)
                            if self.CAM2_post:
                                self.CAM2_check_roll += 1 
                                self.CAM2_roll_img_check[self.CAM2_check_roll] = [frame_copy, scores]

            # Không đeo găng tay khi bê roll
            if len(roll_count) > 0 and len(noglove_count) > 0:
                for roll_box in roll_count:
                    for noglove_box in noglove_count:
                        if not (roll_box[2] < noglove_box[0] or  # roll_box bên trái glove_box
                            roll_box[0] > noglove_box[2] or  # roll_box bên phải glove_box
                            roll_box[3] < noglove_box[1] or  # roll_box bên trên glove_box
                            roll_box[1] > noglove_box[3]):   # roll_box bên dưới glove_box
                            cv2.putText(frame_copy, "Glove intersects Roll", (int(roll_box[0]), int(roll_box[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors["NG"], 2)

            # Không bê roll bằng 2 tay
            if len(roll_count) > 0 and (len(glove_count) + len(noglove_count)) > 1:
                for roll_box in roll_count:
                    hands_touching_roll = 0  # Đếm số tay chạm vào vùng cuộn đồng

                    # Kiểm tra từng box tay đeo găng
                    for glove_box in glove_count:
                        if not (roll_box[2] < glove_box[0] or  # roll_box bên trái glove_box
                                roll_box[0] > glove_box[2] or  # roll_box bên phải glove_box
                                roll_box[3] < glove_box[1] or  # roll_box bên trên glove_box
                                roll_box[1] > glove_box[3]):   # roll_box bên dưới glove_box
                            hands_touching_roll += 1

                    # Kiểm tra từng box tay không đeo găng
                    for noglove_box in noglove_count:
                        if not (roll_box[2] < noglove_box[0] or  # roll_box bên trái noglove_box
                                roll_box[0] > noglove_box[2] or  # roll_box bên phải noglove_box
                                roll_box[3] < noglove_box[1] or  # roll_box bên trên noglove_box
                                roll_box[1] > noglove_box[3]):   # roll_box bên dưới noglove_box
                            hands_touching_roll += 1

                    # Nếu có 2 box tay nhưng chỉ 1 tay chạm vào vùng cuộn đồng
                    if (len(glove_count) + len(noglove_count)) >= 2 and hands_touching_roll == 1:
                        print("Carrying Roll with 2 hands, but only 1 hand is touching the Roll!")
                        cv2.putText(frame_copy, "1 Hand Touching Roll", 
                                    (int(roll_box[0]), int(roll_box[1]) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors["NG"], 2)
            # Không đeo găng khi dùng kéo cắt
            if scissor_in_area > 0 and noglove_in_area > 0:
                print("Using Scissor without wearing gloves!")
            if self.CAM2_post:
                # Call Function Check Case
                if self.CAM2_lightGreen:
                # and self.CAM2_check_flashing == 0:
                # if True:
                # if self.CAM2_lightGreen:
                    self.check(mode="touchpanel", cap=cap, img_dict=self.CAM2_touch_panel_img_check)
                    self.check(mode="noglove", cap=cap, img_dict=self.CAM2_noglove_img_check)
                    self.check(mode="cut", cap=cap, img_dict=self.CAM2_cut_img_check)
                self.check(mode="light", cap=cap, img_dict=self.CAM2_light_img_check)
                self.check(mode="roll", cap=cap, img_dict=self.CAM2_roll_img_check)
            if self.CAM2_general_config['save']:
                self.CAM2_writer.write(frame_copy)
        except Exception as e:
            logging.error('Error at %s', 'Processing', exc_info=e)
        return frame_copy