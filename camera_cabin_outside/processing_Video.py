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
from .CentroidTracker import  CentroidTracker
from .CentroidTrackerHistory import CentroidTrackerHistory
sys.path.insert(0, './yolov10')
from datetime import datetime
from ultralytics import YOLOv10
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

PEOPLE_CONSTANT = 3 # Class People
ROLL_CONSTANT = 4 # Class Roll

MIN_SCORE_CONSTANT = 0.5 # Score to check
MIN_CHECK_FRAMES = 10 # Number of frames to check
CLOSE_PEOPLE = 900 # Distance close people
TIME_CLOSE_PEOPLE = 120 # Time close people
NUM_PEOPLE_CHECK = 3
sleep_interval = 0.05
# logging.basicConfig(filename= 'post_time_alarm_cam3.log', level= logging.INFO,
#                     format= '%(asctime)s - %(message)s')
class SSGVision:
    def __init__(self, config_path):
        self.logger_cam3 = logging.getLogger('cam3')
        self.logger_cam3.setLevel(logging.INFO)
        file_handler_cam3 = logging.FileHandler('post_time_alarm_cam3.log')
        file_handler_cam3.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        self.logger_cam3.addHandler(file_handler_cam3)
        self.CAM3_config_path = config_path
        # Read parameter from config path
        with open(self.CAM3_config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
        self.CAM3_general_config = config["general"]
        self.CAM3_UseThreadCap = self.CAM3_general_config["ThreadCap"]
        self.CAM3_model_all_config = config["model_All"]
        self.CAM3_roll_warning = config["accept_roll"]
        self.CAM3_gather_warning = config['gather_warning']
        self.CAM3_api = config["api"]
        self.CAM3_MaxRetry = int(self.CAM3_general_config['MaxRetry'])
        self.CAM3_MaxRetryAPI = int(self.CAM3_general_config['MaxRetryAPI'])
        self.CAM3_ip_address = self.CAM3_general_config['source']
        self.CAM3_auto_restart = self.CAM3_general_config['autoRestart']
        self.CAM3_output_path = self.CAM3_general_config['output']
        self.CAM3_device_setting = self.CAM3_general_config['device']
        self.CAM3_imgsz = self.CAM3_model_all_config['imgsz']
        self.CAM3_apicamsetting = self.CAM3_api["API"] + self.CAM3_api["CAMERA_SETTING"]
        self.CAM3_apiabnormal = self.CAM3_api["API"] + self.CAM3_api["ABNORMAL"]
        self.CAM3_apimedia = self.CAM3_api["API"] + self.CAM3_api["MEDIA"]
        self.CAM3_post = self.CAM3_api["POST"]
        # Check GPU
        # Record Camera
        self.CAM3_desired_width = 1920
        self.CAM3_desired_height = 1080
        self.CAM3_writer = self.setup_video_writer()
        self.CAM3_results = None
        self.CAM3_dect_copper = []
        self.CAM3_duration = 0
        # Set Color
        self.CAM3_colors = {
            "NG": eval(self.CAM3_model_all_config['colors']['NG']),
            "OK": eval(self.CAM3_model_all_config['colors']['OK']),
            "OBJECT": eval(self.CAM3_model_all_config["colors"]["OBJECT"]),
            "FLOOR": eval(self.CAM3_model_all_config['colors']['FLOOR']),
            "GATHER": eval(self.CAM3_model_all_config['colors']['GATHER'])
        }
        # Initialize required parameter for check (no use glove, roll on the floor and hand touch Panel)
        self.CAM3_check_roll = 0

        self.CAM3_roll_img_check = {}
        # Initialize other parameter
        self.CAM3_start_time = None
        self.CAM3_text_color = (255, 255, 255)
        self.CAM3_Tracker = CentroidTrackerHistory(30)
        self.CAM3_RollTracker = CentroidTrackerHistory(15)
        self.CAM3_HandTracker = CentroidTracker(10)
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
        self.CAM3_params_roll = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 2,
            'AbnormalDuration': 1
        }
        self.CAM3_params_manyoperators = {
            'Camip': data_cam["pkid"],
            'AbnormalType': 5,
            'AbnormalDuration': 120
        }
        self.CAM3_headers = {
                'Content-Type': 'application/json'
            }
    def set_coordinates(self):
        self.CAM3_points_roi1 = [self.CAM3_roll_warning["ROI1"][f'point{num + 1}'] for num in range(self.CAM3_roll_warning["ROI1"]['Count'])]
        self.CAM3_polygon_roi1 = Polygon([eval(point) for point in self.CAM3_points_roi1])
        self.CAM3_np_roll_roi1 = np.array([eval(point) for point in self.CAM3_points_roi1])

        self.CAM3_points_roi2 = [self.CAM3_roll_warning["ROI2"][f'point{num + 1}'] for num in range(self.CAM3_roll_warning["ROI2"]['Count'])]
        self.CAM3_polygon_roi2 = Polygon([eval(point) for point in self.CAM3_points_roi2])
        self.CAM3_np_roll_roi2 = np.array([eval(point) for point in self.CAM3_points_roi2])

        self.CAM3_points_gather = [self.CAM3_gather_warning[f'point{num + 1}'] for num in range(self.CAM3_gather_warning['Count'])]
        self.CAM3_polygon_gather = Polygon([eval(point) for point in self.CAM3_points_gather])
        self.CAM3_np_gather = np.array([eval(point) for point in self.CAM3_points_gather]) 
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
        device = torch.device(self.CAM3_device_setting)
        model.to(device)
        # model.half()
        return model

    def connect_camera(self):
        with open(self.CAM3_config_path, "r", encoding="utf8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.CAM3_general_config = config["general"]
            self.CAM3_ip_address = self.CAM3_general_config['source']
        cap = cv2.VideoCapture(self.CAM3_ip_address)
        if cap.isOpened():
            logging.exception('Camera connected')
            print("Reconnected the camera...")
            return cap
        else:
            time.sleep(2)
            return None

    def setup_video_writer(self):
        if self.CAM3_general_config['save']:
            pass
        else:
            return None
        filename = os.path.basename(self.CAM3_output_path)
        save_path = os.path.dirname(self.CAM3_output_path)
        vid_path = os.path.join(save_path, filename+ ".avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(vid_path, fourcc, 10, (self.CAM3_desired_width, self.CAM3_desired_height))
        if not writer.isOpened():
            print(f"Can't Open VideoWriter With Path: {vid_path}")
        return writer
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
    def check(self, mode="", cap=None, img_dict=None):
        try:
            if mode == "gathering":
                if self.CAM3_duration / TIME_CLOSE_PEOPLE == 1:
                    self.post_alarm(self.CAM3_params_manyoperators, img_dict)
            elif mode == "roll":
                if self.CAM3_check_roll > MIN_CHECK_FRAMES and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 0:
                    img_max_score = self.find_max_score_img(img_dict=img_dict)
                    self.post_alarm(self.CAM3_params_roll, img_max_score)
                    self.CAM3_check_roll = 0
                    self.CAM3_roll_img_check = {}

            if cap.get(cv2.CAP_PROP_POS_FRAMES) > 1 and cap.get(cv2.CAP_PROP_POS_FRAMES) % int(cap.get(cv2.CAP_PROP_FPS)) == 1:
                self.CAM3_check_roll = 0
                self.CAM3_roll_img_check = {}

        except Exception as e:
            logging.error('Error at %s', 'Post alarm', exc_info=e)
    # Run

    def draw_rectangles(self, frame, colors):
        cv2.polylines(frame, [self.CAM3_np_roll_roi1], True, colors["FLOOR"], 2)
        cv2.polylines(frame, [self.CAM3_np_roll_roi2], True, colors["FLOOR"], 2)
        cv2.polylines(frame, [self.CAM3_np_gather], True, colors["GATHER"], 2)

    # Post alarm parameter to Web
    def post_alarm(self, params, img):
        start_time = dt.datetime.now()
        self.logger_cam3.info(f"Start Time: {start_time}")

        img_base64 = cv2.imencode('.jpg', img)
        img_str = base64.b64encode(img_base64[1]).decode("utf-8")
        abnormal = requests.post(url=self.CAM3_apiabnormal, params=params)
        self.CAM3_code = abnormal.json()['code']

        end_time = dt.datetime.now()
        time_diff = end_time - start_time
        self.logger_cam3.info(f"Time taken for post request: {time_diff}")

        now = dt.datetime.now()
        dt_string = now.strftime("%d%m%Y%H%M%S") + '_cam3.jpg' 
        params_save = {
            "AbnormalId" : self.CAM3_code,
            "ImageName" : dt_string,
            "ImageData" : "data:image/jpeg;base64," + img_str
        }
        requests.post(self.CAM3_apimedia, headers=self.CAM3_headers, data=json.dumps(params_save), timeout= 5)

    def process_results(self, results, frame_copy, frame_org, colors, cap):
        try:
            num_persons = 0
            num_roll = 0
            person_boxes = []
            rool_boxes = []
            person_boxes_track_gather = []
            outside_roi_boxes = []
            for r in results:
                if r.boxes is None:
                    continue
                for result in r.boxes.data:
                    result = result.cpu().detach().numpy()
                    boxes = result[:4]
                    left, top, right, bottom = boxes
                    classes = int(result[5])
                    scores = result[4]
                    # Class People
                    if classes == PEOPLE_CONSTANT and scores > 0.5:
                        # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 1)
                        # cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                        num_persons += 1
                        person_boxes.append(boxes)
                        # centerx_person = (left + right)/2
                        # centery_person = (top + bottom)/2

                    # Roll
                    # if classes == ROLL_CONSTANT and scores > MIN_SCORE_CONSTANT:
                    elif classes == ROLL_CONSTANT and scores > 0.7:
                        num_roll += 1
                        # cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), (255,0,0), 1)
                        # cv2.putText(frame_copy, "Roll_{:.2f}%".format(scores * 100), (int(left), int(top)),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)
                        x_center_roll = (right + left)/2
                        y_center_roll = (top + bottom)/2
                        p = Point(x_center_roll, y_center_roll)
                        rool_boxes.append(boxes)
                        self.CAM3_dect_copper.append([x_center_roll, y_center_roll])
            if num_roll > 0:
                rect_boxes = self.CAM3_RollTracker.update(rool_boxes)
                for key, value in rect_boxes.items():
                    Cenx,CenY =value 
                    dect_copper = self.CAM3_RollTracker.get_history(key)
                    bbox = self.CAM3_RollTracker.get_current_bbox(key)
                    left, top, right, bottom = bbox
                    cent_x, cent_y = (left + right)/2, (top + bottom)/2
                    if len(dect_copper) >= 5:
                        total_distance = 0
                        for i in range(-5, -1):
                            current_point = np.array(dect_copper[i])
                            past_point = np.array(dect_copper[i + 1])
                            total_distance += abs(np.linalg.norm(current_point - past_point))

                        distance = total_distance/4

                        p2 = Point((left + right)//2, bottom)
                        if (self.CAM3_polygon_roi1.contains(p2) or self.CAM3_polygon_roi2.contains(p2)) and distance <= 1:
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
                                    if self.CAM3_post:
                                        self.CAM3_check_roll += 1
                                        self.CAM3_roll_img_check[self.CAM3_check_roll] = [frame_copy, scores]                
                            else:
                                cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["NG"], 2)
                                cv2.putText(frame_copy, "Copper on Floor_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors["NG"], 2)
                                if self.CAM3_post:
                                    self.CAM3_check_roll += 1
                                    self.CAM3_roll_img_check[self.CAM3_check_roll] = [frame_copy, scores]
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
                recttrack = self.CAM3_Tracker.update(person_boxes)
                num_persons = 0
                self.CAM3_text_color = (255, 0, 0)
                self.CAM3_gather_color = (0, 0, 255)

                if self.CAM3_duration / TIME_CLOSE_PEOPLE >= 1:
                    self.CAM3_text_color = (0, 0, 255)
                    
                for key, value in recttrack.items():
                    num_persons += 1
                    Cenx,CenY =value
                    box = self.CAM3_Tracker.get_current_bbox(key)
                    left, top, right, bottom = box
                    center = Point((left + right) / 2, (top + bottom) / 2)  
                    if self.CAM3_polygon_gather.contains(center):
                        person_boxes_track_gather.append(box)
                    else:
                        outside_roi_boxes.append(box)     

                close_people_boxes = self.find_close_people(person_boxes_track_gather)
                # close_people_boxes = [tuple(box) for i, box in enumerate(person_boxes_track) if close_people_counts[i] >= 2]
                for i, box in enumerate(person_boxes_track_gather):
                    left, top, right, bottom = box
                    if self.CAM3_duration / TIME_CLOSE_PEOPLE >= 1:
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
                    if self.CAM3_start_time is None:
                        self.CAM3_start_time = time.time()
                else:
                    self.CAM3_start_time = None

                if self.CAM3_start_time is not None:
                    self.CAM3_duration = int(time.time() - self.CAM3_start_time)
                    cv2.putText(frame_copy, f"Duration: {self.CAM3_duration}s", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    self.CAM3_text_color = (0, 0, 255)
                else:
                    self.CAM3_text_color = (255, 255, 255)
                    # Draw line close people
                    # for (box1, box2) in close_people:
                    #     x1, y1, x2, y2 = box1
                    #     x3, y3, x4, y4 = box2
                    #     center1 = (int(x1 + x2) // 2, int(y1 + y2) // 2)
                    #     center2 = (int(x3 + x4) // 2, int(y3 + y4) // 2)
                    #     frame_copy = cv2.line(frame_copy, center1, center2, (0, 0, 255), 2)
                for box in outside_roi_boxes:
                    left, top, right, bottom = box
                    cv2.rectangle(frame_copy, (int(left), int(top)), (int(right), int(bottom)), colors["OBJECT"], 2)
                    cv2.putText(frame_copy, "People_{:.2f}%".format(scores * 100), (int(left), int(top)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, colors["OBJECT"], 2)                
                person_boxes_track_gather = []
                outside_roi_boxes = []
                person_boxes = []
            if self.CAM3_post:
                # Call Function Check Case
                self.check(mode="gathering", cap=cap, img_dict=frame_copy)
                self.check(mode="roll",cap=cap, img_dict=self.CAM3_roll_img_check)
            if self.CAM3_general_config['save']:
                self.CAM3_writer.write(frame_copy)
        except Exception as e:
            logging.error('Error at %s', 'Processing', exc_info=e)
        return frame_copy 