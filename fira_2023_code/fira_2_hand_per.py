import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from qrcode_yolo3.get_qr import get_qrcode_range
from pyzbar import pyzbar
rospy.init_node('fira_2_hand_node', anonymous=True)

width = 856
height = 480
global code_list
code_list = "None"

def image_cb(ros_data):
    global code_list
    #print(rospy.get_time())
    try: 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        img = cv2.resize(img, (int(width), int(height)), interpolation=cv2.INTER_CUBIC)
        # img_cut,img_label,pos = get_qrcode_range(img)
        # img_cut = cv2.GaussianBlur(img_cut, (5,5), 1)
        barcodes = pyzbar.decode(img)
        if len(barcodes) > 0:
            barcodeData = barcodes[0].data.decode("utf-8")
            print(barcodeData)
            code_list = barcodeData
        print(code_list)
        # cv2.imwrite("get_img.png", img)
    except CvBridgeError as e:
        print(e)
                
    #res = img.copy()
    #img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

subscriber = rospy.Subscriber("/image/compressed", CompressedImage, image_cb,  queue_size = 1, buff_size=2**24)
rospy.spin()