#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
import apriltag
import cv2
import glob
import math
import numpy as np
import os
import rospy
import time

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from distutils.util import strtobool
from numpy.linalg import inv
from pprint import pprint
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray,Header,Empty, String, Bool


class collect_data:
  
    def __init__(self):
    
      # image size
      self.width = 856
      self.height = 480
      ###############################################################################
      # image converter
      self.bridge = CvBridge()
      self.detector = apriltag.Detector()
      ###############################################################################
      
      # ros sub, pub
      #self.point_pub = rospy.Publisher("/target_point_fira", Float64MultiArray, queue_size = 1)
      self.point_road_pub = rospy.Publisher("/target_point_fira_road", Float64MultiArray, queue_size = 1)
      self.point_tag_pub = rospy.Publisher("/target_point_fira_tag", Float64MultiArray, queue_size = 1)
      self.point_land_pub = rospy.Publisher("/target_point_fira_land", Float64MultiArray, queue_size = 1)
      self.subscriber = rospy.Subscriber("/image/compressed", CompressedImage, self.image_cb,  queue_size = 1, buff_size=2**24)
      # self.subscriber = rospy.Subscriber("/dji/image", Image, self.image_cb, queue_size=1, buff_size=2**24)  # 
      self.status_sub = rospy.Subscriber("/dji/status", String, self._status_cb, queue_size = 1)
      self.sub_start_detect = rospy.Subscriber("/control_color", Bool, self.start_detect_cb, queue_size=10) 
      self.sub_start_detect_road = rospy.Subscriber("/control_road", Bool, self.start_detect_road_cb, queue_size=10)  
      ###############################################################################
      
      # video writer
      self.title  = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
      self.fourcc = cv2.VideoWriter_fourcc('X',"V",'I','D')
      self.out = cv2.VideoWriter(self.title + '.avi', self.fourcc, 15,(self.width, self.height))
      self.out_ori = cv2.VideoWriter(self.title + '_ori.avi', self.fourcc, 15,(self.width, self.height))
      
      #self.write_data_file = open(self.title+"_data.csv", 'w')
      #self.write_data_file.write("id,lat,lot,alt,yaw\n")
      #self.count = 0
       
      self.img = np.array([])
      #self.t = 0
      ###############################################################################
           
      self.isFlying = False
      self.isConnected = False
      self.areMotorsOn = False

      self.home = [500.0, 500.0]
      self.battery = 100

      self.yaw = 0.0
      self.lat = 500.0
      self.lot = 500.0
      self.alt = 0.01
      self.indoor_height = 0.01
        
      self.gimbal_pitch = 0.0  # up 29.2  ~  -90 down
      self.gimbal_yaw = 0.0
      self.gimbal_roll = 0.0
      self.intrinsic_parameters = np.array([639.840550, 0.000000, 418.805188, 0.000000, 488.641295, 237.164378, 0.000000, 0.000000, 1.000000]).reshape([3,3])

      self.nav_intrinsic_parameters = inv(self.intrinsic_parameters)
      self.gne = np.array([0,1,0,-1,0,0,0,0,self.indoor_height+0.01]).reshape([3,3])
      self.nav_gne = inv(self.gne)
      ###############################################################################

      ###  mission variable
      # self.laps: All laps count
      # self.now_laps: now lap
      # self.now_tag: now tag
      # self.tag_info: tag information
      # self.found_tag_list: found tag id to add
      ###
      self.laps = 2
      self.now_laps = 2 #1
      self.now_tag = 0
      # vert = 0, hori = 1, up 0, down 1
      # self.tag_info = [ [0, [0.6 , 1.17]], [1, [0.3, 1.1, 0]], [0, [0.3 , 1.3]], [1, [0.3 , 1.5, 1]] ]
      self.tag_info = [ [0, [0.5, 0.85]], [0, [0.5 , 1]], [1, [0.5 , 0.6, 0]] ] 
      # test landing
      # self.tag_info = [] 
      # self.tag_info = [ [1, [0.5 , 0.6, 0]] ]
      self.found_tag_list = []
      
      ### start/cancel detect ###
      self.start_detect =  False  # False
      self.start_detect_road = False  #False
      
      ### distance cal ###
      self.F = 24
      self.m_x = self.intrinsic_parameters[0][0] / self.F
      self.m_y = self.intrinsic_parameters[1][1] / self.F
      
      ### vertical window width and height  
      self.vert_gap_w = 1000.0
      self.vert_gap_h = 700.0
      
      ### horizantal window width and height 
      self.hori_gap_w = 900.0
      self.hori_gap_h = 900.0
      self.hori_target = ""
      self.target_type = "vert_gap"
      
      ### using circular mask to follow road
      self.template = np.load("template.npy")
      
      ### follow road settings
      self.area_in = [0]*4
      self.area_out = [0]*4
      self.out_prob = [0]*4
      self.in_prob = [0]*4
      self.max_area_in = 0
      self.max_area_out = 0
      self.s_img = [None]*9
      self.contour_in = [None]*4
      self.contour_out = [None]*4
      self.contour_center = None
      self.in_ignore_index = []
      self.out_ignore_index = []
      self.direction_text = ["right", "forward", "left", "backward"]
      self.out_last_max_index = 1 # forward, 2 left, 3 backward
      self.in_last_max_index = 1  # forward 

    def img_spilt(self, img, w, h):
        s = [None]*9
        for i in range(1,len(s)+1):
          temp = np.zeros(img.shape[:2], dtype=np.uint8)
          mask = np.where(self.template == i)
          temp[mask] = 1
          s[i - 1] = cv2.bitwise_and(img,img, mask = temp)
          show = s[i - 1].copy()
          #show = cv2.resize(show, (86,48))
          #cv2.imshow("s"+str(i), show)
        #cv2.waitKey(0)  
        return s

    # return choose region point 
    def choose_center(self, max_index, c1):
      x = 0
      y = 0
      M = cv2.moments(c1)
      if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
      if cv2.pointPolygonTest(c1,(x,y),False) == 1.0:
        return int(x), int(y)
      else:
        extLeft = tuple(c1[c1[:, :, 0].argmin()][0])
        extRight = tuple(c1[c1[:, :, 0].argmax()][0])
        extTop = tuple(c1[c1[:, :, 1].argmin()][0])
        extBot = tuple(c1[c1[:, :, 1].argmax()][0])

        rule = [  
                   [extLeft, extLeft, extLeft],
                   [extBot,extBot,extBot],
                   [extRight, extRight, extRight],
                   [extTop,extTop,extTop]                            
               ]
                 
        y_dis = math.sqrt((extBot[0] - extTop[0])**2 + (extBot[1] - extTop[1])**2)
        x_dis = math.sqrt((extRight[0] - extLeft[0])**2 + (extRight[1] - extLeft[1])**2)
        i = 0
        if y_dis < x_dis:
          i = 0
        elif y_dis < x_dis:
          i = 1
        else:
          i = 2

        return int(rule[max_index][i][0]), int(rule[max_index][i][1])

    def get_ignore_index(self, index): 
        if index == 0:
          return [2]
        elif index == 1:
          return [3]
        elif index == 2:
          return [0]
        elif index == 3:
          return [1]

    def createModelArea(self, split_img):
      # create base area list
      contour_region = [None]*9
      out_area = [0]*9

      for i in range(0, len(split_img)):
        contour_region[i], _ = cv2.findContours(split_img[i].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if contour_region[i] is None:
          contour_region[i] = np.array([])
        if len(contour_region[i]) > 0:
          a = 0
          for j in range(0, len(contour_region[i])):
            a+=cv2.contourArea(contour_region[i][j])
          out_area[i] = a
        else:
          out_area[i] = 0.0

      return contour_region, out_area, sum(out_area)

    def computeModelArea(self, split_img):
      # create base area list
      contour_region = [None]*9
      out_area = [0]*9

      for i in range(0, len(split_img)):
        contour_region[i], _ = cv2.findContours(split_img[i].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if contour_region[i] is None:
          contour_region[i] = np.array([]) 
        
        if len(contour_region[i]) > 0:
          a = 0
          for j in range(0, len(contour_region[i])):
            #if (i+1) not in self.in_ignore_index and (i+1) not in self.out_ignore_index:
            a+=cv2.contourArea(contour_region[i][j])
          out_area[i] = a
        else:
          out_area[i] = 0.0
 
      if out_area[4] == 0 and 0 in self.in_ignore_index:
        self.in_ignore_index.remove(0)
        #self.out_ignore_index.remove(0)
      if out_area[5] == 0 and 1 in self.in_ignore_index:
        self.in_ignore_index.remove(1)
        #self.out_ignore_index.remove(1)
      if out_area[6] == 0 and 2 in self.in_ignore_index:
        self.in_ignore_index.remove(2)
        #self.out_ignore_index.remove(2)
      if out_area[7] == 0 and 3 in self.in_ignore_index:
        self.in_ignore_index.remove(3)
        #self.out_ignore_index.remove(3)
         
      return contour_region, out_area, sum(out_area)    

    def start_detect_cb(self, msg):
      self.start_detect = msg.data
      
    def start_detect_road_cb(self, msg):
      self.start_detect_road = msg.data
      if self.start_detect_road == False:
        self.in_ignore_index = []
        self.out_ignore_index = []
        #self.direction_text = ["right", "forward", "left", "backward"]
        self.out_last_max_index = 1 # forward, 2 left, 3 backward
        self.in_last_max_index = 1  # forward 
      
    def _status_cb(self, msg):
      temp_data = msg.data.split(";")
      #for i in range(0, len(temp_data)):
      self.home = [float(temp_data[0].split("=")[1]), float(temp_data[1].split("=")[1])]  
      self.yaw = float(temp_data[3].split("=")[1])
      self.battery = float(temp_data[4].split("=")[1])
      self.isConnected = strtobool(temp_data[5].split("=")[1])
      self.areMotorsOn = strtobool(temp_data[6].split("=")[1])
      self.isFlying = strtobool(temp_data[7].split("=")[1])
      self.lat = float(temp_data[8].split("=")[1])
      self.lot = float(temp_data[9].split("=")[1])
      self.alt = float(temp_data[10].split("=")[1])
      self.gimbal_pitch = float(temp_data[12].split("=")[1])
      self.gimbal_yaw = float(temp_data[13].split("=")[1])
      self.gimbal_roll = float(temp_data[11].split("=")[1])
      self.indoor_height = float(temp_data[14].split("=")[1])   
      print(self.battery)
      rot_mat = np.array([math.cos(math.radians(self.gimbal_pitch+90)), -math.sin(math.radians(self.gimbal_pitch+90)), math.sin(math.radians(self.gimbal_pitch+90)), math.cos(math.radians(self.gimbal_pitch+90))])
      
      self.gne = np.array([rot_mat[0],rot_mat[1],0,rot_mat[2],rot_mat[3],0,0,0, (self.indoor_height+0.01)*(1/math.sin(math.radians(-self.gimbal_pitch)))]).reshape([3,3])
      self.nav_gne = inv(self.gne) 
        
    def transelt_point(self,x,y):
      image_point = np.array([ x,y,1])
      a = np.dot(self.nav_gne,self.nav_intrinsic_parameters)
      b= np.dot(a,image_point)
      b=b/b[2]
      return b[0],b[1]
    
    def preProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      # red hsv range and mask on hsv_img
      lower_red_0 = np.array([110, 15, 1]) 
      upper_red_0 = np.array([155, 255, 255])
      #lower_red_1 = np.array([175, 70, 0]) 
      #upper_red_1 = np.array([180, 255, 255])
         
      red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      #red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      #red_mask = cv2.bitwise_or(red_mask0, red_mask1)  
      return red_mask0
    
    def roadProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      cv2.imshow("H",hsv_img[:,:,0])
      cv2.imshow("S",hsv_img[:,:,1])
      cv2.imshow("V",hsv_img[:,:,2])
      lower_road = np.array([0, 0, 0])
      upper_road = np.array([255, 255, 37])
      ret,road_mask_white = cv2.threshold(hsv_img[:,:,1],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
     
      print(ret)
      road_mask_white = cv2.inRange(hsv_img, lower_road, upper_road)
      kernel = np.ones((13,13),np.uint8)
      #dilation = cv.dilat(img,kernel,iterations = 1)
      #road_mask_white = cv2.morphologyEx(road_mask_white, cv2.MORPH_CLOSE, kernel)
      cv2.imshow("ostu", road_mask_white)
      cv2.waitKey(1)
      return road_mask_white
    
    def roadProcessing_old(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      # red hsv range and mask on hsv_img
      lower_red_0 = np.array([41, 15, 1]) 
      upper_red_0 = np.array([75, 255, 255])
      #lower_red_1 = np.array([175, 70, 0]) H
      #upper_red_1 = np.array([180, 255, 255])
         
      red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      #red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      #red_mask = cv2.bitwise_or(red_mask0, red_mask1)  
      return red_mask0
    
    def landmarkProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      # red hsv range and mask on hsv_img
      # lower_red_0 = np.array([3, 0, 0]) 
      # upper_red_0 = np.array([167, 255, 255])
      # dark
      lower_red_0 = np.array([100, 24, 0]) 
      upper_red_0 = np.array([138, 255, 172])
      #lower_red_1 = np.array([175, 70, 0]) 
      #upper_red_1 = np.array([180, 255, 255])
         
      red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      #red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      #red_mask = cv2.bitwise_or(red_mask0, red_mask1)  
      return red_mask0
    
    """def run_old(self, img):
      #res = img.copy()
      pre_img = self.preProcessing(img.copy())
      
      cv2.imshow("pre_img", pre_img)
      
      max_contour_contours, max_contour_h = cv2.findContours(pre_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
         
      if max_contour_contours is not None and len(max_contour_contours) > 0:
        out_max_contours = max(max_contour_contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(out_max_contours)
        center = (x+int(w/2), y+int(h/2))
        
        if self.target_type == "":
          if float(h) / float(w) <= 0.5 :
            self.target_type = "hori_gap"
          else:
            self.target_type = "vert_gap"
        #elif self.target_type != "":
        
        if self.target_type == "vert_gap":
          #dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2) 
          #print(dis)
          true_dis = self.F * ( self.vert_gap_w ) / (w/self.m_x) / 1000.0
          print(true_dis)
          #cv2.drawContours(res, [out_max_contours], -1, (0,0,255), cv2.FILLED)
          #cv2.rectangle(res, (x,y), (x+w,y+h), (0,0,255), 4)
          #cv2.circle(res, center, 2, (0,0,255), -1)
          #cv2.circle(res, (int(self.width/2), int(self.height/2)+48), 2, (255,0,0), -1)

          if true_dis < 1.2 and true_dis > 0.9:
            #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 0, 1, int(self.width/2), int(self.height/2)+48]))
            return x, y, w, h, center[0], center[1], 0, 1, int(self.width/2), int(self.height/2)+48 
          
          elif true_dis < 0.9:
            dx = center[0] - int(self.width/2)
            dy = center[1] - (int(self.height/2)+48)
            if (abs(dx) > int(self.width/40)) or (abs(dy) > int(self.height/40)):
              #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 0, 2, int(self.width/2), int(self.height/2)+48]))
              return x, y, w, h, center[0], center[1], 0, 2, int(self.width/2), int(self.height/2)+48
            else:
              #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 0, 3, int(self.width/2), int(self.height/2)+48]))
              self.start_detect =False
              self.target_type = ""
              return x, y, w, h, center[0], center[1], 0, 3, int(self.width/2), int(self.height/2)+48
          else:
            #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 0, 0, int(self.width/2), int(self.height/2)+48]))
            return x, y, w, h, center[0], center[1], 0, 0, int(self.width/2), int(self.height/2)+48
        
        elif self.target_type == "hori_gap":
          
          true_dis = self.F * (self.hori_gap_w) / (w/self.m_x) / 1000.0
          print(true_dis)
          #cv2.drawContours(res, [out_max_contours], -1, (0,0,255), cv2.FILLED)
          #cv2.rectangle(res, (x,y), (x+w,y+h), (0,0,255), 4)
          #cv2.circle(res, center, 2, (0,0,255), -1)
          target = [int(self.width/2), int(self.height/2)+48]
          if self.hori_target == "":
            if center[1] < int(self.height/2)+48:
              #cv2.circle(res, (int(self.width/2), 60), 2, (255,0,0), -1)
              target = [int(self.width/2), 60]
              self.hori_target = "up"
            elif center[1] >= int(self.height/2)+48:
              #cv2.circle(res, (int(self.width/2), 370), 2, (255,0,0), -1)
              target = [int(self.width/2), 370]
              self.hori_target = "down"
          elif self.hori_target != "":
            if self.hori_target == "up":
              #cv2.circle(res, (int(self.width/2), 60), 2, (255,0,0), -1)
              target = [int(self.width/2), 60]
            elif self.hori_target == "down":
              #cv2.circle(res, (int(self.width/2), 370), 2, (255,0,0), -1)
              target = [int(self.width/2), 370]
          
          if true_dis < 1.2 and true_dis > 0.9:
            #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 1, 1, target[0], target[1]]))
            return x, y, w, h, center[0], center[1], 1, 1, target[0], target[1]
          
          elif true_dis < 0.9:
            dx = center[0] - target[0]
            dy = center[1] - target[1]
            if (abs(dx) > int(self.width/40)) or (abs(dy) > int(self.height/40)):
              #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 1, 2, target[0], target[1]]))
              return x, y, w, h, center[0], center[1], 1, 2, target[0], target[1] 
            else:
              #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 1, 3, target[0], target[1]]))
              
              self.start_detect =False
              self.target_type = "" 
              self.hori_target = ""
              return x, y, w, h, center[0], center[1], 1, 3, target[0], target[1]
          else:
            #self.point_pub.publish(Int16MultiArray(data = [ center[0], center[1], 1, 0, target[0], target[1]]))
            return x, y, w, h, center[0], center[1], 1, 0, target[0], target[1]        
        #return res"""
    
    def road_extract(self, img):
      
      ### hsv to find mask ###
      road_img = self.roadProcessing(img.copy())
      
      ### find contours ###
      road_contours, road_contour_h = cv2.findContours(road_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
      if road_contours is None:
        road_contours = np.array([])
         
      if len(road_contours) > 0:
        
        ### max contour ###
        max_road_contour = max(road_contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(max_road_contour)
        center = (x+int(w/2), y+int(h/2))
        
        ### draw max contour on new image ###
        blank_image = np.zeros((self.height,self.width, 1), np.uint8)
        cv2.drawContours(blank_image, [max_road_contour], -1, (255,255,255), cv2.FILLED)
        cv2.imshow("blank_image", blank_image)
        
        ### split img ###
        split_img = self.img_spilt(blank_image.copy(), int(self.width/3), int(self.height/3))
      
        ### first run ### 
        if self.in_last_max_index == -1:
          out_contour_region, out_area, out_max_area = self.createModelArea(split_img)
          self.contour_out = out_contour_region[:4]
          self.contour_in = out_contour_region[4:8]
          self.contour_center = out_contour_region[8]
          self.area_out = out_area[:4]
          self.area_in = out_area[4:8]
          self.in_prob = list(np.array(self.area_in) / out_max_area)
          self.out_prob = list(np.array(self.area_out) / out_max_area)
          self.max_area_in = max(self.in_prob)
          self.max_area_out = max(self.out_prob)
        ### loop ###    
        else:
          out_contour_region, out_area, out_max_area = self.computeModelArea(split_img)    
          self.contour_out = out_contour_region[:4]
          self.contour_in = out_contour_region[4:8]
          self.contour_center = out_contour_region[8]
          self.area_out = out_area[:4]
          self.area_in = out_area[4:8]
          for i in range(0,len(self.area_in)):
            if i in self.in_ignore_index:
              #print(i)
              #print(self.in_ignore_index)
              out_max_area = out_max_area - (self.area_in[i] + self.area_out[i])
              self.area_in[i] -= self.area_in[i]
              self.area_out[i] -= self.area_out[i]
                   
          self.in_prob = list(np.array(self.area_in) / out_max_area)
          self.out_prob = list(np.array(self.area_out) / out_max_area)
          self.max_area_in = max(self.in_prob)
          self.max_area_out = max(self.out_prob)
        ### out, left or right ###
        if self.max_area_in == 0:
          print("Out circle!!! ")
          target_cx, target_cy = (int(self.width/2), int(self.height/2)+48)
          target_px, target_py = (int(self.width/2), int(self.height/2)+48)
          followType = 3 # road maybe in ext left or right, or outer circle
          inContour = 0
          d1x,d1y = self.transelt_point(center[0], center[1])
          d2x,d2y = self.transelt_point(target_cx, target_cy)
          dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)  
          print("follow type: 3, dis", dis)
          return center[0], center[1], target_cx, target_cy, target_px, target_py, followType, inContour, dis
        ### in, tune angle and forward ###
        else:
          followType = 0 # normal
          if self.in_last_max_index == -1:     
            if self.max_area_in == 0:
              print("no water found!!!", self.in_prob)
            elif self.max_area_in != 0:
              if self.in_prob.index(self.max_area_in) != 3:
                self.in_last_max_index = self.in_prob.index(self.max_area_in)
                self.in_ignore_index = self.in_ignore_index + self.get_ignore_index(self.in_last_max_index)
                self.in_ignore_index = list(set(self.in_ignore_index))
                print("first decision: ", self.in_last_max_index)     
               
          elif self.in_last_max_index != -1:
            if self.in_prob[self.in_last_max_index] != 0:
              #continue follow
              if self.in_prob.index(self.max_area_in) == self.in_last_max_index:
                print("decision not changed: ", self.in_last_max_index)
                #followType = 0   
                self.in_ignore_index = self.in_ignore_index + self.get_ignore_index(self.in_last_max_index)
                self.in_ignore_index = list(set(self.in_ignore_index))
              # maybe quick to corner
              elif self.in_prob.index(self.max_area_in) != self.in_last_max_index:
                print("decision not changed(maybe soon changed): ", self.in_last_max_index)
                followType = 1 # slow to corner  
                self.in_ignore_index = self.in_ignore_index + self.get_ignore_index(self.in_last_max_index)
                self.in_ignore_index = list(set(self.in_ignore_index))
            # corner
            elif self.in_prob[self.in_last_max_index] == 0:  
              if self.max_area_in != 0:
                print("decision changed: ",self.in_prob.index(self.max_area_in), self.in_last_max_index)
                if self.in_prob.index(self.max_area_in) != 3:
                  self.in_last_max_index = self.in_prob.index(self.max_area_in)
                if self.get_ignore_index(self.in_last_max_index)[0] not in self.in_ignore_index:
                  self.in_ignore_index = self.in_ignore_index + self.get_ignore_index(self.in_last_max_index)
                  self.in_ignore_index = list(set(self.in_ignore_index))     
              elif self.max_area_in == 0:
                print("no water found!!!", self.in_prob) 
                self.out_last_max_index = 1 # forward, 2 left, 3 backward
                self.in_last_max_index = 1  # forward 
            
            ### final decide index ### 
            draw_index = self.in_last_max_index
            
            
            ### computer target pos
            target_x = None
            target_y = None
            if len(self.contour_in[draw_index]) > 0:
              target_x, target_y = self.choose_center(draw_index, self.contour_in[draw_index][0])
            
            ### type decide ###
            if target_x is not None:
              #cv2.circle(res, (target_x, target_y), 2, (0,255,0), -1)
              target_cx, target_cy = (int(self.width/2), int(self.height/2)+48)
              target_px, target_py = (int(self.width/2), int(self.height/2)+48)
              rev_index = self.get_ignore_index(draw_index)[0]
              ### 0: forward and backward 
              if len(self.contour_in[rev_index]) > 0:
                target_px, target_py = self.choose_center(rev_index, self.contour_in[rev_index][0])
                d1x,d1y = self.transelt_point(target_cx, target_cy)
                d2x,d2y = self.transelt_point(target_px, target_cy)
                dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)  
                print("follow type: 0, dis", dis)
              ### 2: only forward ###
              elif len(self.contour_in[rev_index]) == 0:
                followType = 2 # p = c 
                #target_px, target_py = self.choose_center(rev_index, self.contour_in[rev_index][0])
                d1x,d1y = self.transelt_point(target_cx, target_cy)
                d2x,d2y = self.transelt_point(target_px, target_py)
                dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)  
                print("follow type: 2, dis", dis)
              ### check center is in center-contour
              inContour = 0
              if cv2.pointPolygonTest(max_road_contour, (target_cx, target_cy), False) == 1.0:
                inContour = 1  
                
              return [center[0], center[1], target_cx, target_cy, target_px, target_py, followType, inContour, dis]
              #print(target_x, target_y, target_cx, target_cy, target_px, target_py)
              #return target_x, target_y, target_cx, target_cy, target_px, target_py, followType, inContour, dis
      return None
    def run_landDetect(self, img):
      
      ### hsv to get land mask ###
      land_img = self.landmarkProcessing(img.copy())
      
      ### find contours ###
      land_contours, land_contour_h = cv2.findContours(land_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
      ### None check ###
      if land_contours is None:
        land_contours = np.array([])
      
      ### found ###   
      if len(land_contours) > 0:
        ### max area contour ###
        max_land_contour = max(land_contours, key = cv2.contourArea)
        
        ### find center ###
        x,y,w,h = cv2.boundingRect(max_land_contour)
        center = (x+int(w/2), y+int(h/2))
        
        ### check 
        d1x,d1y = self.transelt_point(x, y)
        d2x,d2y = self.transelt_point(x+w, y)
        dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)  
        
        ### maybe error
        if dis < 0.3:
          print("not land")
          return -1, -1
        ### return pixel position
        else:
          return center[0], center[1]
      ### not found, return -1, -1 ###
      else:
        print("not found.")
        return -1, -1
    
    def image_cb(self, ros_data):
        #print(rospy.get_time())
        try: 
          np_arr = np.fromstring(ros_data.data, np.uint8)
          self.img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
          self.img = cv2.resize(self.img, (int(self.width), int(self.height)), interpolation=cv2.INTER_CUBIC)
        except CvBridgeError as e:
            print(e)
                    
        res = self.img.copy()
        img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
          
        ### April Tag detect ### 
        result = self.detector.detect(img_gray)
        found = 0
        tag_pos = [-1,-1]
        cx, cy = -1, -1
        
        ### land condition 
        if len(self.found_tag_list) == len(self.tag_info):
          cx, cy = self.run_landDetect(self.img.copy())
          
        ### found April Tag  
        if len(result) > 0:
          found = 1
            
          ### ID not in self.found_tag_list, add.               
          ### using tag_info to publish data: 0: vertical, 1: horizantal
          ### isfound, tag_center, tag left up corner, tag left down corner, current tag counts, window type, window height, window distance, window up/down
           
          if result[0].tag_id not in self.found_tag_list:
            self.found_tag_list.append(result[0].tag_id)
            self.now_tag = len(self.found_tag_list) - 1
          tag_pos = result[0].center
          tag_corn = result[0].corners
          cv2.circle(res, (int(tag_pos[0]), int(tag_pos[1])), 2, (0,255,0), -1)
          if self.tag_info[self.now_tag][0] == 0:
            self.point_tag_pub.publish(Float64MultiArray(data = [found, int(tag_pos[0]), int(tag_pos[1]), int(tag_corn[0][0]), int(tag_corn[0][1]), int(tag_corn[3][0]), int(tag_corn[3][1]), self.now_tag, self.tag_info[self.now_tag][0], self.tag_info[self.now_tag][1][0], self.tag_info[self.now_tag][1][1], -1]))
          elif self.tag_info[self.now_tag][0] == 1:
            self.point_tag_pub.publish(Float64MultiArray(data = [found, int(tag_pos[0]), int(tag_pos[1]), int(tag_corn[0][0]), int(tag_corn[0][1]), int(tag_corn[3][0]), int(tag_corn[3][1]), self.now_tag, self.tag_info[self.now_tag][0], self.tag_info[self.now_tag][1][0], self.tag_info[self.now_tag][1][1], self.tag_info[self.now_tag][1][2]]))
            
        ### no tag, no road , but landmark
        if found == 0:
          # check laps, not finished >>> reset list and parameters
          #                 finished >>> detect_road > False, publish laps+1 (if laps = 2, then publish 3) and laps
          # publish data: landmark position, center, now laps and all laps
          if cx != -1 and cy != -1:
            if self.now_laps < self.laps :
              self.now_laps += 1
              self.found_tag_list = []
              self.now_tag = 0
              self.point_land_pub.publish(Float64MultiArray(data = [cx, cy, int(self.width / 2), int(self.height/2), self.now_laps, self.laps]))
              
            else: 
              self.start_detect_road = False
              cv2.circle(res, (cx, cy), 2, (255,255,255), -1)
              self.point_land_pub.publish(Float64MultiArray(data = [cx, cy, int(self.width / 2), int(self.height/2), self.now_laps+1, self.laps]))    
          
        ### road detect ###  
        if self.start_detect_road == True:
          road_res = self.road_extract(self.img)
          if road_res is not None:
            [target_rx, target_ry, target_cx, target_cy, target_px, target_py, followType, inContour, dis] = road_res
            cv2.circle(res, (target_rx, target_ry), 2, (0,0,255), -1)
            cv2.circle(res, (target_cx, target_cy), 2, (0,0,0), -1)
            cv2.circle(res, (target_px, target_py), 2, (255,0,0), -1)

            self.point_road_pub.publish(Float64MultiArray(data = [ target_rx, target_ry, target_cx, target_cy, target_px, target_py, followType, inContour, int(dis*100000) / 100000.0]))
          
        ### record video and show img ###
        self.out_ori.write(self.img)
        self.out.write(res)
        cv2.imshow("res", res)  
        cv2.waitKey(1)     
    
    #def run_mission(self):
    #  while not rospy.is_shutdown():
    #    if len(self.img) != 0:
          
          
            
def main():
    rospy.init_node('collect_data_dji', anonymous=True)
    ss_n = collect_data()
    try:
        #ss_n.run_mission()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()  

