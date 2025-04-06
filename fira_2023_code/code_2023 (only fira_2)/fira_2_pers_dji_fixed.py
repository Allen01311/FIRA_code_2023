#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
import math
import numpy as np
import rospy
import sys
import time

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from distutils.util import strtobool
from numpy.linalg import inv
from pprint import pprint
from pyzbar import pyzbar
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, NavSatFix, PointCloud2
from std_msgs.msg import Float64MultiArray,Header,Empty, String, Bool, Int8

class collect_data:

    def __init__(self):

        # image size
        self.width = 856
        self.height = 480
        ###############################################################################
    
        # image converter
        self.bridge = CvBridge()
        ###############################################################################

        # ros sub, pub
        self.subscriber = rospy.Subscriber("/image/compressed", CompressedImage, self.image_cb,  queue_size = 1, buff_size=2**24)
        #self.subscriber = rospy.Subscriber("/dji/image", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.status_sub = rospy.Subscriber("/dji/status", String, self._status_cb, queue_size = 1)
        self.qrcode_pub = rospy.Publisher("/target_fira_2_qrcode", Float64MultiArray, queue_size = 1)
        self.point_road_pub = rospy.Publisher("/target_point_fira_road", Float64MultiArray, queue_size = 1)
        self.sub_start_detect_red = rospy.Subscriber("/control_red", Bool, self.start_detect_red_cb, queue_size=10) 
        self.point_red_pub = rospy.Publisher("/target_point_fira_red", Float64MultiArray, queue_size = 1)
        self.sub_update_mission = rospy.Subscriber("/update_mission", Int8, self.update_mission_cb, queue_size = 1)
        self.sub_start_detect_land = rospy.Subscriber("/control_land", Bool, self.start_detect_land_cb, queue_size=10)
        self.sub_start_detect_road = rospy.Subscriber("/control_road", Bool, self.start_detect_road_cb, queue_size=10)  
        self.point_land_pub = rospy.Publisher("/target_point_fira_land", Float64MultiArray, queue_size = 1)
        self.sub_start_collect3 = rospy.Subscriber("/collect3", Bool, self.start_collect3_cb, queue_size=10)
        self.sub_start_collect4 = rospy.Subscriber("/collect4", Bool, self.start_collect4_cb, queue_size=10)
        ###############################################################################
        
        # video writer
        self.title  = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        self.fourcc = cv2.VideoWriter_fourcc('X',"V",'I','D')
        self.out = cv2.VideoWriter(self.title + '.avi', self.fourcc, 15,(self.width, self.height))
        self.out_ori = cv2.VideoWriter(self.title + '_ori.avi', self.fourcc, 15,(self.width, self.height))
        
        #self.count = 0
       
        self.img = np.array([])
        #self.t = 0
        #######################################
        
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
        
        ############################################################################
        
        ### first mission:code ###
        # 1: Delivering the first aid kit
        # 2: Insulator scan in electrical tower
        # 3: Searching for victims in tall building
        # 4: Return and land on starting point
        self.now_mission = 1
        
        ### direction text ###
        self.direction_text = ["N","E","S","W"]
        
        ### now direction ###
        self.direction = "N"
        
        ### mission_tpye ###
        self.mission_type = -1
        
        ### mission setting ###
        self.mission3_data = []
        self.mission4_data = []
        self.start_detect_red = False
        self.start_detect_road = False
        self.start_detect_land = False
        self.start_collect3 = False
        self.start_collect4 = False
        self.start_detect_first_pass = False
        self.start_detect_second_pass = False
        
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
      #gne1 = np.array([rot_mat[0],0,rot_mat[2],0,1,0,rot_mat[1],0,rot_mat[3]]).reshape([3,3])
      
      self.gne = np.array([rot_mat[0],rot_mat[1],0,rot_mat[2],rot_mat[3],0,0,0, (self.indoor_height+0.01)*(1/math.sin(math.radians(-self.gimbal_pitch)))]).reshape([3,3])
      #self.gne = np.dot(gne1,gne2)
      self.nav_gne = inv(self.gne) 
    
    def start_collect3_cb(self, msg):
      self.start_collect3 = msg.data
    
    def start_collect4_cb(self, msg):
      self.start_collect4 = msg.data
    
    def start_detect_land_cb(self, msg):
      self.start_detect_land = msg.data
    
    def start_detect_red_cb(self, msg):
      self.start_detect_red = msg.data
    
    def update_mission_cb(self, msg):
      self.now_mission = msg.data

    def start_detect_road_cb(self, msg):
      self.start_detect_road = msg.data

    def start_detect_first_pass_cb(self, msg):
      self.start_detect_first_pass = msg.data
      
    def start_detect_second_pass_cb(self, msg):
      self.start_detect_second_pass = msg.data  
        
    def transelt_point(self,x,y):
      image_point = np.array([ x,y,1])
      a = np.dot(self.nav_gne,self.nav_intrinsic_parameters)
      b= np.dot(a,image_point)
      b=b/b[2]
      return b[0],b[1]    
    
    def redProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      # red hsv range and mask on hsv_img
      lower_red_0 = np.array([0, 70, 0]) 
      upper_red_0 = np.array([15, 255, 255])
      #lower_red_1 = np.array([175, 70, 0]) 
      #upper_red_1 = np.array([180, 255, 255])
         
      red_mask = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      #red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      #red_mask = cv2.bitwise_or(red_mask0, red_mask1)  
      return red_mask
   
    def roadProcessing_old(self, img):
      blurred_img = cv2.GaussianBlur(img, (5, 5), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
      lower_road = np.array([0, 0, 1])
      upper_road = np.array([180, 25, 255])
      # white image
      road_mask_white = cv2.inRange(hsv_img, lower_road, upper_road)
      # black image
      road_mask_black = 255 - road_mask_white
      return road_mask_white  
    
    def roadProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      cv2.imshow("H",hsv_img[:,:,0])
      cv2.imshow("S",hsv_img[:,:,1])
      cv2.imshow("V",hsv_img[:,:,2])
      lower_road = np.array([0, 90, 0])
      upper_road = np.array([179, 255, 255])
      ret,road_mask_white = cv2.threshold(hsv_img[:,:,1],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
     
      print(ret)
      road_mask_white = cv2.inRange(hsv_img, lower_road, upper_road)
      kernel = np.ones((13,13),np.uint8)
      #dilation = cv.dilat(img,kernel,iterations = 1)
      #road_mask_white = cv2.morphologyEx(road_mask_white, cv2.MORPH_CLOSE, kernel)
      cv2.imshow("ostu", road_mask_white)
      cv2.waitKey(1)
      return road_mask_white      
    
    def landmarkProcessing(self, img):
      blurred_img = cv2.GaussianBlur(img, (13, 13), 0)
      hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

      # red hsv range and mask on hsv_img
      lower_red_0 = np.array([100, 15, 1]) 
      upper_red_0 = np.array([130, 255, 255])
      #lower_red_1 = np.array([175, 70, 0]) 
      #upper_red_1 = np.array([180, 255, 255])
         
      red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
      #red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
      #red_mask = cv2.bitwise_or(red_mask0, red_mask1)  
      return red_mask0
         
    def run(self, img):
      #res = img.copy()
      ### qrcode decode ###
      barcodes = pyzbar.decode(img, symbols=[pyzbar.ZBarSymbol.QRCODE])
      
      ### road QRcode ###
      if self.start_collect4 == False and self.start_collect3 == False:
        if len(barcodes) > 0:
            #for barcode in barcodes:
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcodes[0].rect
          
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcodes[0].data.decode("utf-8")
            barcodeType = barcodes[0].type
            return x, y, w, h, barcodeData, barcodeType
            # draw the barcode data and barcode type on the image
            # print the barcode type and data to the terminal
            # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        else:
          return -1, -1, -1, -1, "", ""
      ### mission 3 ###
      elif self.start_collect3 == True:
        if len(barcodes) > 0:
          for barcode in barcodes:
            (x, y, w, h) = barcode.rect

            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            text = "{}".format(barcodeData)

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            self.mission3_data.append(text)
            cv2.imwrite("mission2_" + text + ".jpg", img)
          self.mission3_data = list(set(self.mission3_data))
        return -2, -2, -2, -2, "", ""
      ### mission 4 ###
      elif self.start_collect4 == True:
        if len(barcodes) > 0:
          for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            text = "{}".format(barcodeData)
            
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            self.mission4_data.append(text)
            cv2.imwrite("mission3_" + text + ".jpg", img)
          self.mission4_data = list(set(self.mission4_data))
        return -2, -2, -2, -2, "", ""  

    def choose_target(self, cnt, index):

      px = []
      for point in cnt:
        if point[0][1] == index:
          px.append(point[0][0])
      
      if len(px) > 0:
        return (int(float(sum(px)) / len(px)), index)
        
      else:
        return -1, -1    
        
    def red_extract(self, img):
      ### delivery mission ###
      
      ### hsv to get red mask ###
      red = self.redProcessing(img)
      
      ### find contours ###
      red_contours, red_contour_h = cv2.findContours(red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
      ### None check ###
      if red_contours is None:
        red_contours = np.array([])
      
      ### found ###   
      if len(red_contours) > 0:
        ### max area contour ###
        max_red_contour = max(red_contours, key = cv2.contourArea)
        
        ### find center ### 
        x,y,w,h = cv2.boundingRect(max_red_contour)
        center = (x+int(w/2), y+int(h/2))
        
        ### check  
        d1x,d1y = self.transelt_point(x, y)
        d2x,d2y = self.transelt_point(x+w, y)
        dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2)  
        
        ### maybe error 
        if dis < 0.3:
          print("not found")
          return -1, -1, -1, -1
        ### return pixel position and view point  
        else:
          return center[0], center[1], int(self.width/2), int(self.height/4*3)
      ### not found, return -1, -1 ###
      else:
        print("not found.")
        return -1, -1, -1, -1
    
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
        try: 
            np_arr = np.fromstring(ros_data.data, np.uint8)
            decode_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            self.img = cv2.resize(decode_img, (int(self.width), int(self.height)), interpolation=cv2.INTER_CUBIC)
            
        except CvBridgeError as e:
            print(e)
        
        res = self.img.copy()
        
        # QRCODE detection
        x, y, w, h, barcodeData, barcodeType = self.run(self.img.copy())
        
        ### detect ###
        ### road QRcode ###
        if x != -1 and x != -2:
          cv2.rectangle(res, (x, y), (x + w, y + h), (0, 0, 255), 2)
          text = "{}".format(barcodeData)
          cv2.putText(res, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
          if "," in barcodeData:
            split_data = barcodeData.split(",")
            self.direction = split_data[self.now_mission - 1]
            self.mission_type = float(split_data[-1])
            #print(self.mission_type, self.direction_text.index(self.direction))
            self.qrcode_pub.publish(Float64MultiArray(data = [self.mission_type, self.direction_text.index(self.direction), x, y, x, y+h, x+int(w/2), y+int(h/2), self.direction_text.index(split_data[0]), self.direction_text.index(split_data[1]), self.direction_text.index(split_data[2]), self.direction_text.index(split_data[3]),float(split_data[-1])]))
        ### not road QRcode ###
        else:
         if x != -2:
          self.qrcode_pub.publish(Float64MultiArray(data = [self.mission_type, self.direction_text.index(self.direction), -1, -1, -1, -1, -1, -1,-1,-1,-1,-1,-1]))
        
        ### road detect ###
        if self.start_detect_road == True:
          road_mask = self.roadProcessing(self.img)
          contours, hierarchy = cv2.findContours(road_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
          total_cnt = []
          
          if contours is None:
            contours = np.array([])
          
          ### add near contours to one ###
          if len(contours) > 0:
            for idx, cnt in enumerate(contours):
              contour_img = np.zeros(self.img.shape[:2], np.uint8)
              cv2.drawContours(contour_img, [cnt], -1, 255, cv2.FILLED)
              area_thresh = cv2.countNonZero(contour_img)/(self.img.shape[0]*self.img.shape[1])
              if area_thresh > 0.1 :
                 total_cnt.append(cnt)

            center_x = self.img.shape[0]//2
            center_y = self.img.shape[1]//2
            closest_cnt = []
            if len(total_cnt) == 1:
              closest_cnt = total_cnt
            else :
              min_dist = 100000
              for idx, cnt in enumerate(total_cnt) :
                dist = abs(cv2.pointPolygonTest(cnt, (center_x,center_y), True))
                if dist < min_dist :
                  dist = min_dist
                  closest_cnt = [cnt]
           
            # final image and draw closest_cnt
            final_img = np.zeros(self.img.shape[:2], np.uint8)
            
            cv2.drawContours(final_img, closest_cnt, -1, 255, cv2.FILLED)
            cv2.imshow("final_img", final_img)
            
            contour_final, _ = cv2.findContours(final_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if contour_final is None:
              contour_final = np.array([])
            if len(contour_final) > 0:
              out_max_contours = max(contour_final, key = cv2.contourArea)
              
              target_rx, target_ry = self.choose_target(out_max_contours, int(self.height/3.0))
              target_px, target_py = self.choose_target(out_max_contours, int(self.height/4.0*3))
              target_cx, target_cy = (int(self.width/2), int(self.height/4.0*3))
              
              dis = -1
              if target_px != -1:
                d1x,d1y = self.transelt_point(target_cx, target_cy)
                d2x,d2y = self.transelt_point(target_px, target_py)
                dis = math.sqrt((d2x - d1x)**2 + (d2y - d1y)**2) 
              
              self.point_road_pub.publish(Float64MultiArray(data = [ target_rx, target_ry, target_cx, target_cy, target_px, target_py, -1, -1, int(dis*100000) / 100000.0]))              
            else:
              print("cannot find road1.")
              self.point_road_pub.publish(Float64MultiArray(data = [ -1, -1, -1, -1, -1, -1, -1, -1, -1])) 
          else:
            print("cannot find road.")
            self.point_road_pub.publish(Float64MultiArray(data = [ -1, -1, -1, -1, -1, -1, -1, -1, -1]))     
          
        ### detect red mark for mission delivery
        if self.start_detect_red == True:
          target_rx, target_ry, target_cx, target_cy = self.red_extract(self.img)
          if target_rx != -1 and target_cx != -1:
            cv2.circle(res, (target_rx, target_ry), 2, (0,0,255), -1)
            cv2.circle(res, (target_cx, target_cy), 2, (0,0,0), -1)

            self.point_red_pub.publish(Float64MultiArray(data = [ target_rx, target_ry])) 
        
        ### detect landmark ###
        cx, cy = -1, -1
        if self.start_detect_land == True:
          cx, cy = self.run_landDetect(self.img)
          if cx != -1 and cy != -1:
            self.point_land_pub.publish(Float64MultiArray(data = [cx, cy, int(self.width / 2), int(self.height/2), -1, -1]))
        
        ### every 5 secs, print QRCODE mission3 and mission4 data ###
        t = int(rospy.get_time())
        if len(self.mission3_data) > 0 or len(self.mission4_data) > 0:
          if t % 5 == 0:
            print("=========================================")
            print("Mission 2: ")
            print(self.mission3_data)
            print("Mission 3: ")
            print(self.mission4_data)      
            print("=========================================")
          
        ### record video and show img ###
        self.out_ori.write(self.img)
        self.out.write(res)
        cv2.imshow("res", res)  
        cv2.waitKey(1) 
    #def run_mission(self):
    #  while not rospy.is_shutdown():        
         

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
