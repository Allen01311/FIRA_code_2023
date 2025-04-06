import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('get_img_node', anonymous=True)

width = 856
height = 480

def image_cb(ros_data):
    #print(rospy.get_time())
    try: 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        img = cv2.resize(img, (int(width), int(height)), interpolation=cv2.INTER_CUBIC)
        cv2.imwrite("get_img.png", img)
    except CvBridgeError as e:
        print(e)
                
    #res = img.copy()
    #img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

subscriber = rospy.Subscriber("/image/compressed", CompressedImage, image_cb,  queue_size = 1, buff_size=2**24)
rospy.spin()