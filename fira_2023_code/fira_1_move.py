#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import cv2
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

#(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

class btControl_mission:

    # common member
    now_gap = -1
    
    drone = tingyi_DJI_drone.Drone()
    isContinue = True
    space = space_mod.space_mod()

    width = 856
    height = 480

    bridge = CvBridge() 

    # Take Off
    take0ff_complete = False
    
    # speed factor
    sp_f = 1
    
    def __init__(self):
    
        ### behavior tree ###
        # finish condition: found landmark pos >>> do land action
        # not finish: tag action or follow action
        ###
        
        self.tree = (
            self.NotFinish >> ((self.FoundTag >> self.tag_action) | self.following) 
            | self.land 
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
    def NotFinish(self):
        print("condition: NotFinish", btControl_mission.drone.state.land_x)
        return btControl_mission.drone.state.land_x == -1

    @condition
    def FoundTag(self):
        print("condition: FoundTag", btControl_mission.drone.state.found)
        return btControl_mission.drone.state.found == 1 and btControl_mission.now_gap == btControl_mission.drone.state.tag_count-1
    
    ### old version ###
    @action
    def pass_rec(self):
      print("action: pass_rec", btControl_mission.drone.state.target_type, btControl_mission.drone.state.index, btControl_mission.drone.state.tp_x, btControl_mission.drone.state.tp_y, btControl_mission.drone.state.grid_x, btControl_mission.drone.state.grid_y)
      if btControl_mission.drone.state.target_type == 0:
        dx = btControl_mission.drone.state.tp_x - btControl_mission.drone.state.grid_x
        dy = btControl_mission.drone.state.tp_y - btControl_mission.drone.state.grid_y

        # ===== vert_gap =====
        if btControl_mission.drone.state.index == 0:
          t = Twist()
          t.linear.x = 0.03 * btControl_mission.sp_f
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.05
          if abs(dy) > int(btControl_mission.height/40):
            t.linear.z = -dy / abs(dy) * 0.05
          btControl_mission.drone.flightCrtl.move(t, 0.1)
        
        elif btControl_mission.drone.state.index == 1:
          #dx = btControl_mission.drone.state.tp_x - int(btControl_mission.width/2)
          #dy = btControl_mission.drone.state.tp_y - (int(btControl_mission.height/4))
          t = Twist()
          t.linear.x = 0.03 * btControl_mission.sp_f
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.04
          if abs(dy) > (int(btControl_mission.height/40)):
            t.linear.z = -dy / abs(dy) * 0.04
          btControl_mission.drone.flightCrtl.move(t, 0.1)
      
        elif btControl_mission.drone.state.index == 2:  
          t = Twist()
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.04
          if abs(dy) > (int(btControl_mission.height/40)):
            t.linear.z = -dy / abs(dy) * 0.04
          btControl_mission.drone.flightCrtl.move(t, 0.1)
        
        elif btControl_mission.drone.state.index == 3:  
 
          s = False
          btControl_mission.drone.flightCrtl.s_detection(s)

          #t = Twist()
          #t.linear.x = 0.8
          #btControl_mission.drone.flightCrtl.move_s(t, 2.0) 
          btControl_mission.drone.state.target_type = -1
          
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_90()   
          
          print("angle", btControl_mission.drone.state.gimbal_pitch)
          
          btControl_mission.drone.state.found = 0
          btControl_mission.drone.state.tag_x = -1
          btControl_mission.drone.state.tag_y = -1
          #btControl_mission.drone.state.ctx = -1
          #btControl_mission.drone.state.cty = -1
          btControl_mission.drone.state.rx == -1
          
          s = True
          btControl_mission.drone.flightCrtl.r_detection(s)
      
      # ===== hori_gap =====
      elif btControl_mission.drone.state.target_type == 1: 
        dx = btControl_mission.drone.state.tp_x - btControl_mission.drone.state.grid_x
        dy = btControl_mission.drone.state.tp_y - btControl_mission.drone.state.grid_y

        if btControl_mission.drone.state.index == 0:
          t = Twist()
          t.linear.x = 0.03 * btControl_mission.sp_f
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.05
          if abs(dy) > int(btControl_mission.height/40):
            t.linear.z = -dy / abs(dy) * 0.05
          btControl_mission.drone.flightCrtl.move(t, 0.1)
        
        elif btControl_mission.drone.state.index == 1:
          #dx = btControl_mission.drone.state.tp_x - int(btControl_mission.width/2)
          #dy = btControl_mission.drone.state.tp_y - (int(btControl_mission.height/4))
          t = Twist()
          t.linear.x = 0.03 * btControl_mission.sp_f
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.04
          if abs(dy) > (int(btControl_mission.height/40)):
            t.linear.z = -dy / abs(dy) * 0.04
          btControl_mission.drone.flightCrtl.move(t, 0.1)  
        
        elif btControl_mission.drone.state.index == 2:  
          t = Twist()
          if abs(dx) > int(btControl_mission.width/40): 
            t.linear.y = dx / abs(dx) * 0.04
          if abs(dy) > (int(btControl_mission.height/40)):
            t.linear.z = -dy / abs(dy) * 0.04
          btControl_mission.drone.flightCrtl.move(t, 0.1)
        
        elif btControl_mission.drone.state.index == 3:  
          #tx = Twist()
          #btControl_mission.drone.flightCrtl.move_s(tx, 0.1)  
          
          s = False
          btControl_mission.drone.flightCrtl.s_detection(s)        
          
          t = Twist()
          t.linear.x = 0.8
          btControl_mission.drone.flightCrtl.move_s(t, 2.0) 
          
          t = Twist()
          
          if btControl_mission.drone.state.grid_y < int(btControl_mission.height/2):
            t.linear.z = 0.5
          elif btControl_mission.drone.state.grid_y >= int(btControl_mission.height/2):
            t.linear.z = -0.5
          btControl_mission.drone.flightCrtl.move(t, 1.5) 
          btControl_mission.drone.state.target_type = -1
          
          print("tune camera angle...")
          while btControl_mission.drone.state.gimbal_pitch >= -89.0:#-90.0: 
            btControl_mission.drone.flightCrtl.ser_gimbal_90() 
          
          print("angle", btControl_mission.drone.state.gimbal_pitch)          
          
          btControl_mission.drone.state.found = 0
          btControl_mission.drone.state.tag_x = -1
          btControl_mission.drone.state.tag_y = -1
          #btControl_mission.drone.state.ctx = -1
          #btControl_mission.drone.state.cty = -1          
          btControl_mission.drone.state.rx == -1
          
          s = True
          btControl_mission.drone.flightCrtl.r_detection(s)
          
     
    @action
    def following(self):
      print("action: following", btControl_mission.drone.state.followType, btControl_mission.drone.state.inContour, btControl_mission.drone.state.move_dis)
      
      if btControl_mission.drone.state.rx == -1 or btControl_mission.drone.state.cx == -1:
        print("no received data...")
      else:
        tx = Twist()
        
        ### forwrad, type for adjust speed ###
        if btControl_mission.drone.state.followType == 0 or btControl_mission.drone.state.followType == 2:
          tx.linear.x = 0.15 * btControl_mission.sp_f
        elif btControl_mission.drone.state.followType == 1:
          tx.linear.x = 0.1 * btControl_mission.sp_f
        elif btControl_mission.drone.state.followType == 3:
          tx.linear.x = 0.05 
        
        # left/right dx
        dx = btControl_mission.drone.state.px - btControl_mission.drone.state.cx
        """if btControl_mission.drone.state.px == btControl_mission.drone.state.cx:
          dx = btControl_mission.drone.state.rx - btControl_mission.drone.state.cx
        elif btControl_mission.drone.state.px != btControl_mission.drone.state.cx:
          dx = btControl_mission.drone.state.px - btControl_mission.drone.state.cx
          
        if btControl_mission.drone.state.py == btControl_mission.drone.state.cy:  
          dy = btControl_mission.drone.state.ry - btControl_mission.drone.state.cy
        elif btControl_mission.drone.state.py != btControl_mission.drone.state.cy:
          dy = btControl_mission.drone.state.py - btControl_mission.drone.state.cy"""
        
        # angle compute   
        rx = -(btControl_mission.drone.state.rx - btControl_mission.drone.state.px)
        ry = -(btControl_mission.drone.state.ry - btControl_mission.drone.state.py)
               
        angle = math.degrees(math.atan2(ry, rx))
        if angle < 0 :
          angle = 360 + angle
        
        # center angle         
        anglec = 90     
        
        ### left/right  
        if abs(dx) > 15 and (btControl_mission.drone.state.followType == 0 or btControl_mission.drone.state.followType == 2):
          tx.linear.y = (dx / abs(dx)) * btControl_mission.drone.state.move_dis
        elif abs(dx) > 15 and btControl_mission.drone.state.followType == 1:
          tx.linear.y = (dx / abs(dx)) * btControl_mission.drone.state.move_dis * 0.66
        elif btControl_mission.drone.state.followType == 3:
          dx = btControl_mission.drone.state.rx - btControl_mission.drone.state.cx
          #dy = btControl_mission.drone.state.ry - btControl_mission.drone.state.cy
          if btControl_mission.drone.state.ry < btControl_mission.drone.state.cy and dx != 0:
            tx.linear.y = (dx / abs(dx)) * btControl_mission.drone.state.move_dis * 0.5
        
        ### angle ###  
        if angle != 0 and btControl_mission.drone.state.followType != 3: # and btControl_mission.drone.state.inContour == 1 and btControl_mission.drone.state.followType != 2
          
          ### +- 1 degrees ###
          if anglec-1 > int(angle):
            tx.angular.z = (int(angle) - (anglec-1)) / 360.0 * 0.8 * btControl_mission.sp_f#3.14 / (180.0/(int(angle) - (anglec+1))) * 0.1    #
            
          elif anglec+1 < int(angle):
            tx.angular.z =  (int(angle) - (anglec+1)) / 360.0 * 0.8 * btControl_mission.sp_f#3.14 / (180.0/(int(angle) - (anglec+1))) * 0.1    #
        
        # error: no rotate    
        if btControl_mission.drone.state.ry >= btControl_mission.drone.state.cy and abs(btControl_mission.drone.state.rx - btControl_mission.drone.state.cx) < 30:
          tx.angular.z = 0
        
        print(tx)
        ### publish 1s and stop ###
        btControl_mission.drone.flightCrtl.move_s(tx, 1.0) 

    @action
    def tag_action(self):
        print("action: tag_action", btControl_mission.now_gap, btControl_mission.drone.state.tag_count, btControl_mission.drone.state.tag_height, btControl_mission.drone.state.indoor_height)
        if btControl_mission.now_gap == btControl_mission.drone.state.tag_count-1:
          tx = Twist()
          
          print("stop detect road!!!")
          #tx = Twist()
          #btControl_mission.drone.flightCrtl.move_s(tx, 0.1) 
          s = False
          btControl_mission.drone.flightCrtl.r_detection(s)
          
          ### vertical ###  
          if btControl_mission.drone.state.tag_type == 0:
            tx = Twist()
            #print()
            #tx = Twist()
            if btControl_mission.drone.state.tag_y < int(btControl_mission.height/2)-40:
              tx.linear.x = 0.04
              
            rx = -(btControl_mission.drone.state.tc_lux - btControl_mission.drone.state.tc_ldx)
            ry = -(btControl_mission.drone.state.tc_luy - btControl_mission.drone.state.tc_ldy)
            angle = math.degrees(math.atan2(ry, rx))
            if angle < 0 :
              angle = 360 + angle
                 
            anglec = 90
            print(angle)
            if anglec-2 > int(angle):
              tx.angular.z =  (int(angle) - (anglec-2)) / 360.0 * 1.5 #3.14 / (180.0/(int(angle) - (anglec-2))) * 0.1    #
            
            elif anglec+2 < int(angle):
              tx.angular.z = (int(angle) - (anglec+2)) / 360.0 * 1.5 #3.14 / (180.0/(int(angle) - (anglec+2))) * 0.1     #
            
            dx = btControl_mission.drone.state.tag_x - int(btControl_mission.width/2)
            if abs(dx) > 5:
              tx.linear.y = (dx / abs(dx)) * 0.04
            btControl_mission.drone.flightCrtl.move(tx, 0.1)
            
            ### ok ###
            if (abs(dx) <= 5 and anglec-2 <= angle <= anglec+2):
              print("get!")
              dz = btControl_mission.drone.state.tag_height - btControl_mission.drone.state.indoor_height
              if abs(dz) > 0.3:
                tx.linear.z = dz / 2.0
                btControl_mission.drone.flightCrtl.move_s(tx, 2.2) 
              
              btControl_mission.drone.forward(0.4)
              btControl_mission.drone.reset_height(btControl_mission.drone.state.indoor_height)
              btControl_mission.drone.state.found = 0
              btControl_mission.drone.state.tag_x = -1
              btControl_mission.drone.state.tag_y = -1
              btControl_mission.now_gap += 1  
              s = True
              print("start detect road!!!")
              btControl_mission.drone.flightCrtl.r_detection(s)
          
          ### hori ###  
          elif btControl_mission.drone.state.tag_type == 1:
            
            tx = Twist()
            #print()
            #tx = Twist()
            if btControl_mission.drone.state.tag_y < int(btControl_mission.height/2)-40:
              tx.linear.x = 0.04
            
            rx = -(btControl_mission.drone.state.tc_lux - btControl_mission.drone.state.tc_ldx)
            ry = -(btControl_mission.drone.state.tc_luy - btControl_mission.drone.state.tc_ldy)
            angle = math.degrees(math.atan2(ry, rx))
            if angle < 0 :
              angle = 360 + angle
                 
            anglec = 90
            print(angle)
            if anglec-2 > int(angle) : #and btControl_mission.drone.state.tag_y >= int(btControl_mission.height/2)-40:
              tx.angular.z = (int(angle) - (anglec-2)) / 360.0 * 1.5 #3.14 / (180.0/(int(angle) - (anglec-2))) * 0.1     #
            
            elif anglec+2 < int(angle) : #and btControl_mission.drone.state.tag_y >= int(btControl_mission.height/2)-40:
              tx.angular.z = (int(angle) - (anglec+2)) / 360.0 * 1.5 #3.14 / (180.0/(int(angle) - (anglec+2))) * 0.1     #
            
            dx = btControl_mission.drone.state.tag_x - int(btControl_mission.width/2)
            if abs(dx) > 5:
              tx.linear.y = (dx / abs(dx)) * 0.04
            btControl_mission.drone.flightCrtl.move(tx, 0.1)
            
            ### ok ###
            if (abs(dx) <= 5 and anglec-2 <= angle <= anglec+2):
              print("get!")
              dz = btControl_mission.drone.state.tag_height - btControl_mission.drone.state.indoor_height
              print("--------------------------------------------------------------")
              print("--------------------------------------------------------------")
              print('dz: ', dz)
              print("tag height: ", btControl_mission.drone.state.tag_height) 
              print("indoor height: ", btControl_mission.drone.state.indoor_height)
              print("--------------------------------------------------------------")
              print("--------------------------------------------------------------")
              if abs(dz) > 0.25:
                tx.linear.z = dz / 2.0
                btControl_mission.drone.flightCrtl.move_s(tx, 2.2) 
                
              tx = Twist()
                
              move_dis = btControl_mission.drone.state.tag_dis + 0.55
              tx.linear.x = move_dis / 2.0
              btControl_mission.drone.flightCrtl.move_s(tx, 2.0)  
               
              ### height adjust ###  
              # 0 => down to up
              # 1 => up to down
              if btControl_mission.drone.state.tag_direction == 0:
                tx = Twist()
                tx.linear.z = 0.65
                btControl_mission.drone.flightCrtl.move_s(tx, 1.5)

              elif btControl_mission.drone.state.tag_direction == 1:
                tx = Twist()
                tx.linear.z = -0.6
                btControl_mission.drone.flightCrtl.move_s(tx, 1.5)
                btControl_mission.drone.forward(0.3)
                btControl_mission.drone.reset_height(btControl_mission.drone.state.indoor_height)
              
              btControl_mission.drone.state.found = 0
              btControl_mission.drone.state.tag_x = -1
              btControl_mission.drone.state.tag_y = -1
              btControl_mission.now_gap += 1  
              s = True
              print("start detect road!!!")
              btControl_mission.drone.flightCrtl.r_detection(s)
    @action
    def land(self):
        print("action: land", btControl_mission.drone.state.land_x, btControl_mission.drone.state.land_y, btControl_mission.drone.state.now_lap, btControl_mission.drone.state.laps)
        
        ### not equal: 3, 2, land ###
        if btControl_mission.drone.state.now_lap > btControl_mission.drone.state.laps:
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
        ### continue, need adjust ###
        else:
          tx = Twist()
          tx.linear.x = 0.05
          btControl_mission.drone.flightCrtl.move(tx, 0.1)
          
        #  tx = Twist()
        #  tx.angular.z = 3.14 / 2.0
        #  btControl_mission.drone.flightCrtl.move_s(tx, 1.5)
        #  btControl_mission.drone.state.land_x = -1
        #  btControl_mission.drone.state.land_y = -1
        #  btControl_mission.now_gap = -1
          #print("down to " + str(2.0))
          #while btControl_mission.drone.state.alt > 2.5:
          #  tx = Twist()
          #  tx.linear.z = -1.0
          #  btControl_mission.drone.flightCrtl.move(tx,1)
          #print("down to " + str(btControl_mission.drone.state.alt))
        

    def startDetect(self):
      print("start detect!!!")
      s = True
      btControl_mission.drone.flightCrtl.s_detection(s)
      
    def stopDetect(self):
      print("stop detect!!!")
      s = False
      btControl_mission.drone.flightCrtl.s_detection(s)
      
    def startDetect_road(self):
      
      s = True
      btControl_mission.drone.flightCrtl.r_detection(s)
      print("start detect road!!!")

    def stopDetect_road(self):
      print("stop detect road!!!")
      s = False
      btControl_mission.drone.flightCrtl.r_detection(s)

    def run(self):
        while True:
            if btControl_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print("state = %s\n" % state)
            if btControl_mission.drone.isStop == True:
              exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print("state = %s\n" % state)
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
    

    t = rospy.get_time()
    while rospy.get_time() - t < 1.0:
      pass

    print("start detect road...")
    btCm_n.startDetect_road()
    
    try:
        ### run bt ###
        btCm_n.run()
        #rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    #cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
