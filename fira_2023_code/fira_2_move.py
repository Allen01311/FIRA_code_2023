#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import geopy.distance
import math
import numpy as np
import pickle
import roslib
import rospy
import space_mod
import sys
import time
import tingyi_DJI_drone

from behave import *
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pprint import pprint
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from std_msgs.msg import Bool, Empty, Int16MultiArray, Float64MultiArray

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

class btControl_mission:

    # common member
    # mission_list: mission code sequence
    # now_mission: now mission
    # now_direction: now direction
    mission_list = [2,3,4,1]
    now_mission = 2
    now_direction = 0
    direction_text = ["N","E","S","W"]
    
    first_pass = False
    first_off_pass = False
    second_pass = False
    
    drone = tingyi_DJI_drone.Drone()
    isContinue = True
    space = space_mod.space_mod()

    width = 856
    height = 480

    bridge = CvBridge() 

    # Take Off
    take0ff_complete = False

    def __init__(self):
    
        ### bt ###
        # current mission >> FIND mision code QRcode >> do action, else flying
        ###    ###
        self.tree = (
            self.isMission2 >> ((self.isFind >> self.Mission2Action) | self.flying)
            | self.isMission3 >> ((self.isFind >> self.Mission3Action) | self.flying)
            | self.isMission4 >> ((self.isFind >> self.Mission4Action) | self.flying)
            | self.isMission1 >> ((self.isFind >> self.Mission1Action) | self.flying)
             
        )
        print("First run")
    
    def up_to(self, height):
      print("up to " + str(height))
      while btControl_mission.drone.state.alt < height:
        tx = Twist()
        tx.linear.z = 0.5
        btControl_mission.drone.flightCrtl.move(tx,0.5)
      print("up to " + str(btControl_mission.drone.state.alt))
      tx = Twist()
      btControl_mission.drone.flightCrtl.move(tx,1)

    @condition
    def isMission2(self):
        print("condition: isMission2", btControl_mission.now_mission)
        return btControl_mission.now_mission == 2
    
    @condition
    def isMission3(self):
        print("condition: isMission3", btControl_mission.now_mission)
        return btControl_mission.now_mission == 3
        
    @condition
    def isMission4(self):
        print("condition: isMission4", btControl_mission.now_mission)
        return btControl_mission.now_mission == 4
        
    @condition
    def isMission1(self):
        print("condition: isMission1", btControl_mission.now_mission)
        return btControl_mission.now_mission == 1
    
    @condition
    def isFind(self):
        print("condition: isFind", btControl_mission.now_mission, btControl_mission.drone.state.qrcode_mission)
        return btControl_mission.now_mission == btControl_mission.drone.state.qrcode_mission and btControl_mission.now_direction == btControl_mission.drone.state.qrcode_direction
    
    @action
    def flying(self):
      print("action: flying", btControl_mission.now_direction, btControl_mission.drone.state.qrcode_direction, btControl_mission.drone.state.rx)
       
      ### mission 3 ### 
      if btControl_mission.now_mission == 3 and btControl_mission.now_direction == btControl_mission.drone.state.qrcode_direction:
        ### do 1 stage action ###
        if btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_0)] == "W" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_1)] == "E" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_2)] == "N" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_3)] == "E" and int(btControl_mission.drone.state.qrcode_4) == 0 and btControl_mission.first_pass == False:
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch <= -3.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_zero()
              
          t = rospy.get_time()
          while rospy.get_time() - t < 0.5:
            pass
          
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_90()
        
          t = rospy.get_time()
          while rospy.get_time() - t < 1.0:
            pass
          
          ### down hieght
          print("down")  
          height = 0.5
          while True:
            tx = Twist()
            if btControl_mission.drone.state.qrcode_cx != -1:
              dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
              dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
              if abs(dx) <= 30 and abs(dy) <= 30 and btControl_mission.drone.state.indoor_height <= height*1.1:
                t = rospy.get_time()
                while rospy.get_time() - t < 0.5:
                  pass
                break  
              
              elif abs(dx) > 15 or abs(dy) > 15: 
                if abs(dx) > 15:
                  tx.linear.y = (dx / abs(dx)) * 0.1
                if abs(dy) > 15:
                  tx.linear.x = -(dy / abs(dy)) * 0.1
            
            if btControl_mission.drone.state.indoor_height > height*1.1:
              tx.linear.z = -0.3
            btControl_mission.drone.flightCrtl.move_s(tx, 0.2)

          t = rospy.get_time()
          while rospy.get_time() - t < 0.5:
            pass
 
          btControl_mission.first_pass = True
        
        ### do 2 stage action ###  
        if btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_0)] == "S" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_1)] == "S" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_2)] == "N" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_3)] == "S" and int(btControl_mission.drone.state.qrcode_4) == 0 and btControl_mission.first_pass == True and btControl_mission.first_off_pass == False:
          
          print("up")
          height = 1.1
          while True:
            tx = Twist()
            if btControl_mission.drone.state.qrcode_cx != -1:
              dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
              dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
              if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height >= height*0.9:
                t = rospy.get_time()
                while rospy.get_time() - t < 0.5:
                  pass
                break  
              
              elif abs(dx) > 15 or abs(dy) > 15: 
                if abs(dx) > 15:
                  tx.linear.y = (dx / abs(dx)) * 0.1
                if abs(dy) > 15:
                  tx.linear.x = -(dy / abs(dy)) * 0.1
            
            if btControl_mission.drone.state.indoor_height < height*0.9:
              tx.linear.z = 0.3
            btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
          
          t = rospy.get_time()
          while rospy.get_time() - t < 0.5:
            pass
          btControl_mission.first_off_pass = True
      
      ### mission 1 ###    
      if btControl_mission.now_mission == 1 and btControl_mission.now_direction == btControl_mission.drone.state.qrcode_direction:
        ### do 1 stage action ###
        if btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_0)] == "S" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_1)] == "S" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_2)] == "S" and btControl_mission.direction_text[int(btControl_mission.drone.state.qrcode_3)] == "N" and int(btControl_mission.drone.state.qrcode_4) == 0 and btControl_mission.second_pass == False:
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch <= -3.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_zero()
          # tx = Twist()
          #drone.flightCrtl.move_s(tx, 1.0)    
          t = rospy.get_time()
          while rospy.get_time() - t < 0.5:
            pass
          
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_90()
        
          t = rospy.get_time()
          while rospy.get_time() - t < 1.0:
            pass
          
          print("up")
          height = 1.4
          while True:
            tx = Twist()
            if btControl_mission.drone.state.qrcode_cx != -1:
              dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
              dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
              if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height >= height*0.9:
                t = rospy.get_time()
                while rospy.get_time() - t < 0.5:
                  pass
                break  
              
              elif abs(dx) > 15 or abs(dy) > 15: 
                if abs(dx) > 15:
                  tx.linear.y = (dx / abs(dx)) * 0.1
                if abs(dy) > 15:
                  tx.linear.x = -(dy / abs(dy)) * 0.1
            
            if btControl_mission.drone.state.indoor_height < height*0.9:
              tx.linear.z = 0.3
            btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
        
          t = rospy.get_time()
          while rospy.get_time() - t < 0.5:
            pass       
          
          btControl_mission.second_pass = True       
      
      ### road following action ###
      if btControl_mission.drone.state.rx != -1 and btControl_mission.drone.state.qrcode_lux == -1 and btControl_mission.drone.state.qrcode_cx == -1: 
        
        dx = btControl_mission.drone.state.px - btControl_mission.drone.state.cx
        rx = -(btControl_mission.drone.state.rx - btControl_mission.drone.state.px)
        ry = -(btControl_mission.drone.state.ry - btControl_mission.drone.state.py)
        angle = math.degrees(math.atan2(ry, rx))
        if angle < 0 :
          angle = 360 + angle
                 
        anglec = 90 
        
        tx = Twist()
        tx.linear.x = 0.15
        if abs(dx) > 15:
          tx.linear.y = (dx / abs(dx)) * 0.05
        
        if btControl_mission.drone.state.rx != -1 and btControl_mission.drone.state.px != -1:  
          if anglec-3 > int(angle) :
            tx.angular.z = (int(angle) - (anglec-3)) / 360.0 * 1.5 #3.14 / (180.0/(int(angle) - (anglec-2))) * 0.1     #
            
          elif anglec+3 < int(angle) :#and btControl_mission.drone.state.tag_y >= int(btControl_mission.height/2)-40:
            tx.angular.z = (int(angle) - (anglec+3)) / 360.0 * 1.5  
        btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
      
      ### direction correct
      if btControl_mission.now_direction == btControl_mission.drone.state.qrcode_direction:    
        if btControl_mission.drone.state.qrcode_lux != -1 and btControl_mission.drone.state.qrcode_cx != -1:
          tx = Twist()
          
          dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
          #dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5)
          if abs(dx) > 15:
            tx.linear.y = dx / abs(dx) * 0.1
          #if abs(dy) > 15:
          tx.linear.x = 0.15
          btControl_mission.drone.flightCrtl.move(tx, 0.3) 
        
      ### need rotate ###  
      elif btControl_mission.now_direction != btControl_mission.drone.state.qrcode_direction:
       tx = Twist()
       #print("stop detect road!!!")
       #s = False
       #btControl_mission.drone.flightCrtl.r_detection(s)
       
       if btControl_mission.drone.state.qrcode_cx != -1:
         dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
         dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
         if abs(dx) > 15 or abs(dy) > 15: 
           if abs(dx) > 15:
             tx.linear.y = (dx / abs(dx)) * 0.1
           if abs(dy) > 15:
             tx.linear.x = -(dy / abs(dy)) * 0.1
           btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
       
         elif abs(dx) <= 15 and abs(dy) <= 15:
          if btControl_mission.now_direction == 0:
            if btControl_mission.drone.state.qrcode_direction == 1:
              tx.angular.z = math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
            
            elif btControl_mission.drone.state.qrcode_direction == 3:
              if btControl_mission.now_mission == 3:
                tx.angular.z = -math.pi / 2.57
              else:
                tx.angular.z = -math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2) 
            elif btControl_mission.drone.state.qrcode_direction == 2:
              tx.angular.z = math.pi / 2
              #btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
              #print("1st rotate")
              #tx.angular.z = math.pi / 2
              btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
              print("2nd rotate") 
          
          elif btControl_mission.now_direction == 1:
            if btControl_mission.drone.state.qrcode_direction == 2:
              tx.angular.z = math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
            elif btControl_mission.drone.state.qrcode_direction == 0:
              tx.angular.z = -math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
            elif btControl_mission.drone.state.qrcode_direction == 3:
              tx.angular.z = math.pi / 2
              #btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
              #print("1st rotate")
              #tx.angular.z = math.pi / 2
              btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
              print("2nd rotate") 
            
          elif btControl_mission.now_direction == 2:
            if btControl_mission.drone.state.qrcode_direction == 3:
              tx.angular.z = math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
            elif btControl_mission.drone.state.qrcode_direction == 1:
              if btControl_mission.now_mission == 4:
                tx.angular.z = -math.pi / 2.57
              else:
                tx.angular.z = -math.pi / 2.57
              #tx.angular.z = -math.pi / 4
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
            elif btControl_mission.drone.state.qrcode_direction == 0:
              tx.angular.z = math.pi / 2
              #btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
              #print("1st rotate")
              #tx.angular.z = math.pi / 2
              btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
              print("2nd rotate")
         
          elif btControl_mission.now_direction == 3:
            if btControl_mission.drone.state.qrcode_direction == 0:
              tx.angular.z = math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
            elif btControl_mission.drone.state.qrcode_direction == 2:
              tx.angular.z = -math.pi / 2.57
              btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
            elif btControl_mission.drone.state.qrcode_direction == 1:
              tx.angular.z = math.pi / 2
              #btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
              #print("1st rotate")
              #tx.angular.z = math.pi / 2
              btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
              print("2nd rotate")
          
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass           
          btControl_mission.now_direction = btControl_mission.drone.state.qrcode_direction
          print("btControl_mission.now_direction changed", btControl_mission.now_direction)
            
    @action
    def Mission2Action(self):
      print("action: Mission2Action", btControl_mission.drone.state.m2_red_x)
      print("stop detect road!!!")
      s = False
      btControl_mission.drone.flightCrtl.r_detection(s)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass            
      
      print("start detect red!!!")
      s = True
      btControl_mission.drone.flightCrtl.red_detection(s)
      
      if btControl_mission.drone.state.m2_red_x == - 1 and  btControl_mission.drone.state.m2_red_y == -1:
        tx = Twist()
        tx.linear.x = 0.1
        btControl_mission.drone.flightCrtl.move_s(tx, 0.5)
      else:
        tx = Twist()
        dx = btControl_mission.drone.state.m2_red_x - int(856/2)
        dy = btControl_mission.drone.state.m2_red_y - int(480/4*2.5)
        
        if abs(dx) > 15:
          tx.linear.y = (dx / abs(dx)) * 0.1
        if abs(dy) > 15:
          tx.linear.x = -(dy / abs(dy)) * 0.1
        btControl_mission.drone.flightCrtl.move(tx, 0.1)
        
        if abs(dx) <= 15 and abs(dy) <= 15:
          print("get")
          now_alt = btControl_mission.drone.state.indoor_height
          tx = Twist()
          tx.linear.x = 0.1
          tx.linear.z = -0.5
          btControl_mission.drone.flightCrtl.move_s(tx, 1.0)
          
          print("down to throw.")
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass
          
          tx = Twist()
          tx.linear.x = -0.1
          tx.linear.z = -0.5
          btControl_mission.drone.flightCrtl.move_s(tx, 1.0)
          
          print("down to throw.")
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass
          
          print("up to ", now_alt)
          while btControl_mission.drone.state.indoor_height < now_alt*0.95:
            tx = Twist()
            tx.linear.z = 0.3
            btControl_mission.drone.flightCrtl.move_s(tx,0.1)
          print("up to " + str(btControl_mission.drone.state.indoor_height))
          
          #tx = Twist()
          #tx.linear.z = 1.0
          #btControl_mission.drone.flightCrtl.move_s(tx, 2.0)
          
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass
          
          tx = Twist()
          tx.angular.z = -math.pi / 2
          #btControl_mission.drone.flightCrtl.move_s(tx, 1.2)  
          #print("1st rotate")
          #tx.angular.z = -math.pi / 2
          btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
          print("2nd rotate")
          btControl_mission.now_direction = 4 - btControl_mission.now_direction
          if btControl_mission.now_direction % 2 == 0:
            btControl_mission.now_direction -= 2
          
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass
          
          ### if not 2341, need change ####
          btControl_mission.now_mission = 3
          btControl_mission.drone.flightCrtl.update_mission(3)
          
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass
          
          print("stop detect red!!!")
          s = False
          btControl_mission.drone.flightCrtl.red_detection(s)     
          
          nt = rospy.get_time()
          while rospy.get_time() - nt <  0.5:
            pass   
          
          print("start detect road!!!")
          s = True
          btControl_mission.drone.flightCrtl.r_detection(s)
          
    @action
    def Mission3Action(self):
      print("action: Mission3Action", btControl_mission.drone.state.indoor_height)
      
      print("stop detect road!!!")
      s = False
      btControl_mission.drone.flightCrtl.r_detection(s)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass 
      
      print("up")             
      height = 1.3
      
      while True:
        tx = Twist()
        if btControl_mission.drone.state.qrcode_cx != -1:
          dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
          dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
          if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height >= height*0.9:
            t = rospy.get_time()
            while rospy.get_time() - t < 0.5:
              pass
            break  
              
          elif abs(dx) > 15 or abs(dy) > 15: 
            if abs(dx) > 15:
              tx.linear.y = (dx / abs(dx)) * 0.1
            if abs(dy) > 15:
              tx.linear.x = -(dy / abs(dy)) * 0.1
            
        if btControl_mission.drone.state.indoor_height < height*0.9:
          tx.linear.z = 0.3
        btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass 
            
      print("tune camera angle...")
      while btControl_mission.drone.state.gimbal_pitch <= -3.0:#-90.0: 
        btControl_mission.drone.flightCrtl.ser_gimbal_zero()
      #tx = Twist()
      #drone.flightCrtl.move_s(tx, 1.0)    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass      
      
      print("start collect")
      s = True
      btControl_mission.drone.flightCrtl.start_collect_3(s)
      
      t = rospy.get_time()
      while rospy.get_time() - t < 1.0:
        pass    
      
      """height = 1.3
      print("up to " + str(height))
      while btControl_mission.drone.state.indoor_height < height*0.95:
        tx = Twist()
        tx.linear.z = 0.3
        btControl_mission.drone.flightCrtl.move_s(tx,0.5)
      print("up to " + str(btControl_mission.drone.state.indoor_height))

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass"""
      
      tx = Twist()
      tx.angular.z = math.pi / 6 
      tx.linear.y = -0.7
      print(tx.angular.z)
    
      btControl_mission.drone.flightCrtl.move_s(tx, 12.2)

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      print("stop collect")
      s = False
      btControl_mission.drone.flightCrtl.start_collect_3(s)

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      tx = Twist()
      tx.angular.z = math.pi / 2
      btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
      btControl_mission.now_direction = 0
      
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      print("tune camera angle...")
      while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
        btControl_mission.drone.flightCrtl.ser_gimbal_90()
        
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      print("down")             
      height = 1.1
      
      while True:
        tx = Twist()
        if btControl_mission.drone.state.qrcode_cx != -1:
          dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
          dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
          if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height <= height*1.1:
            t = rospy.get_time()
            while rospy.get_time() - t < 0.5:
              pass
            break  
              
          elif abs(dx) > 15 or abs(dy) > 15: 
            if abs(dx) > 15:
              tx.linear.y = (dx / abs(dx)) * 0.1
            if abs(dy) > 15:
              tx.linear.x = -(dy / abs(dy)) * 0.1
            
        if btControl_mission.drone.state.indoor_height > height*1.1:
          tx.linear.z = -0.3
        btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass 
      
      """height = 1.1
      print("down to " + str(height))
      while btControl_mission.drone.state.indoor_height > height*1.1:
        tx = Twist()
        tx.linear.z = -0.3
        btControl_mission.drone.flightCrtl.move_s(tx,0.1)
      print("down to " + str(btControl_mission.drone.state.indoor_height))"""       
      
      print("start detect road!!!")
      s = True
      btControl_mission.drone.flightCrtl.r_detection(s)      

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      ### if not 2341, need change ####
      btControl_mission.now_mission = 4
      btControl_mission.drone.flightCrtl.update_mission(btControl_mission.now_mission)
          
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass
        
      
    @action
    def Mission4Action(self):
      print("action: Mission4Action", btControl_mission.drone.state.indoor_height)
      
      print("stop detect road!!!")
      s = False
      btControl_mission.drone.flightCrtl.r_detection(s)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass            
      
      print("up")             
      height = 1.3
      
      while True:
        tx = Twist()
        if btControl_mission.drone.state.qrcode_cx != -1:
          dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
          dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
          if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height >= height*0.9:
            t = rospy.get_time()
            while rospy.get_time() - t < 0.5:
              pass
            break  
              
          elif abs(dx) > 15 or abs(dy) > 15: 
            if abs(dx) > 15:
              tx.linear.y = (dx / abs(dx)) * 0.1
            if abs(dy) > 15:
              tx.linear.x = -(dy / abs(dy)) * 0.1
            
        if btControl_mission.drone.state.indoor_height < height*0.9:
          tx.linear.z = 0.3
        btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass 
      
      print("tune camera angle...")
      while btControl_mission.drone.state.gimbal_pitch <= -3.0:#-90.0: 
        btControl_mission.drone.flightCrtl.ser_gimbal_zero()
      #tx = Twist()
      #drone.flightCrtl.move_s(tx, 1.0)    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      print("start collect")
      s = True
      btControl_mission.drone.flightCrtl.start_collect_4(s)
      
      t = rospy.get_time()
      while rospy.get_time() - t < 1.0:
        pass
      
      """height = 1.3
      print("up to " + str(height))
      while btControl_mission.drone.state.indoor_height < height*0.95:
        tx = Twist()
        tx.linear.z = 0.3
        btControl_mission.drone.flightCrtl.move_s(tx,0.5)
      print("up to " + str(btControl_mission.drone.state.indoor_height))"""
    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      tx = Twist()
      tx.angular.z = -math.pi / 6 
      tx.linear.y = 0.65
      print(tx.angular.z)
    
      btControl_mission.drone.flightCrtl.move_s(tx, 3.1)
    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
    
      tx = Twist()
      tx.angular.z = math.pi / 6 
      tx.linear.y = -0.65
      print(tx.angular.z)
    
      btControl_mission.drone.flightCrtl.move_s(tx, 3.1)
    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
    
      tx = Twist()
      tx.angular.z = math.pi / 6 
      tx.linear.y = -0.65
      print(tx.angular.z)
    
      btControl_mission.drone.flightCrtl.move_s(tx, 3.1)
    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
    
      tx = Twist()
      tx.angular.z = -math.pi / 6 
      tx.linear.y = 0.65
      print(tx.angular.z)
    
      btControl_mission.drone.flightCrtl.move_s(tx, 3.1)
    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
    
      #tx = Twist()
      #tx.angular.z = -math.pi / 2 
      ##tx.linear.y = 0.587
      #print(tx.angular.z)
    
      #btControl_mission.drone.flightCrtl.move_s(tx, 2.4)
    
      #t = rospy.get_time()
      #while rospy.get_time() - t < 1:
      #  pass 
      
      print("stop collect")
      s = False
      btControl_mission.drone.flightCrtl.start_collect_4(s)

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      tx = Twist()
      tx.angular.z = math.pi / 2
      btControl_mission.drone.flightCrtl.move_s(tx, 1.2)
      btControl_mission.now_direction = 2
      
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass      
      
      print("tune camera angle...")
      while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
        btControl_mission.drone.flightCrtl.ser_gimbal_90()
      #tx = Twist()
      #drone.flightCrtl.move_s(tx, 1.0)    
      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      print("down")             
      height = 1.1
      
      while True:
        tx = Twist()
        if btControl_mission.drone.state.qrcode_cx != -1:
          dx = btControl_mission.drone.state.qrcode_cx - int(856/2.0)
          dy = btControl_mission.drone.state.qrcode_cy - int(480/4.0*2.5) 
          if abs(dx) <= 15 and abs(dy) <= 15 and btControl_mission.drone.state.indoor_height <= height*1.1:
            t = rospy.get_time()
            while rospy.get_time() - t < 0.5:
              pass
            break  
              
          elif abs(dx) > 15 or abs(dy) > 15: 
            if abs(dx) > 15:
              tx.linear.y = (dx / abs(dx)) * 0.1
            if abs(dy) > 15:
              tx.linear.x = -(dy / abs(dy)) * 0.1
            
        if btControl_mission.drone.state.indoor_height > height*1.1:
          tx.linear.z = -0.3
        btControl_mission.drone.flightCrtl.move_s(tx, 0.2)
      
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass 
      
      """height = 1.3
      print("down to " + str(height))
      while btControl_mission.drone.state.indoor_height > height*1.1:
        tx = Twist()
        tx.linear.z = -0.3
        btControl_mission.drone.flightCrtl.move_s(tx,0.1)
      print("down to " + str(btControl_mission.drone.state.indoor_height))

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass"""
        
      print("start detect road!!!")
      s = True
      btControl_mission.drone.flightCrtl.r_detection(s)      

      t = rospy.get_time()
      while rospy.get_time() - t < 0.5:
        pass
      
      ### if not 2341, need change ####
      btControl_mission.now_mission = 1
      btControl_mission.drone.flightCrtl.update_mission(btControl_mission.now_mission)
          
      nt = rospy.get_time()
      while rospy.get_time() - nt <  0.5:
        pass
      
    @action
    def Mission1Action(self):
      print("action: Mission1Action", btControl_mission.drone.state.land_x, btControl_mission.drone.state.land_y)  
      
      print("start detect land!!!")
      s = True
      btControl_mission.drone.flightCrtl.land_detection(s)
      
      tx = Twist()
      if btControl_mission.drone.state.land_x == -1:
        tx.linear.x = 0.1
        btControl_mission.drone.flightCrtl.move_s(tx, 0.5)
      else:
        tx = Twist()
        dx = btControl_mission.drone.state.land_x - btControl_mission.drone.state.land_cx
        dy = btControl_mission.drone.state.land_y - btControl_mission.drone.state.land_cy
        
        if abs(dx) > 15:
          tx.linear.y = (dx / abs(dx)) * 0.05
        if abs(dy) > 15:
          tx.linear.x = -(dy / abs(dy)) * 0.05
        btControl_mission.drone.flightCrtl.move(tx, 0.1)
        
        if abs(dx) <= 15 and abs(dy) <= 15:
          print("get, land")
          btControl_mission.drone.flightCrtl.ser_land()
    
    def startDetect_road(self):
      print("start detect road!!!")
      s = True
      btControl_mission.drone.flightCrtl.r_detection(s)
    
    def run(self):
        while True:
            if btControl_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print ("state = %s\n" % state)
            if btControl_mission.drone.isStop == True:
              exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print ("state = %s\n" % state)
                if btControl_mission.drone.isStop == True:
                  exec("f = open(\"123.txt\",\'rb\')")
            assert state == SUCCESS or state == FAILURE

def main():
    rospy.init_node('btControl_mission', anonymous=True)
    print("start...") 
    btCm_n = btControl_mission()
    
    print("take off...")
    btCm_n.drone.flightCrtl.ser_takeoff()
    
    while btCm_n.drone.state.alt < 1.1:
      pass
      
    print("tune camera angle...")
    while btCm_n.drone.state.gimbal_pitch >= -89.0:#-90.0: 
      btCm_n.drone.flightCrtl.ser_gimbal_90()      
    
    btCm_n.startDetect_road()
    t = rospy.get_time()
    while rospy.get_time() - t < 0.5:
      pass
      
    try:
        btCm_n.run()
        #rospy.spin()
    except (IOError, KeyboardInterrupt):
        print("Shutting down ROS Image feature detector module")
        #btCm_n.saveData()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
