import rospy
from clover import srv
from std_srvs.srv import Trigger
from pyzbar.pyzbar import decode as qr_read
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import math
import numpy as np
from clover.srv import SetLEDEffect

rospy.init_node('flight') 
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal) 
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity) 
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude) 
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates) 
land = rospy.ServiceProxy('land', Trigger) 
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

bridge = CvBridge() 
 
qr_debug = rospy.Publisher("/qr_debug", Image, queue_size=10) 
image_color = rospy.Publisher("/Debuuuuuuuug",Image,queue_size=10) 

colorQR = '' 
color = '' 
cup = True 
XY_Color = {'red': [0, 0], 'green': [0, 0], 'blue': [0, 0]} 
red_low = np.array([0,0,240]) 
red_high = np.array([10,10,255]) 
green_low = np.array([0,240,0])  
green_high = np.array([10,255,10]) 
blue_low = np.array([240,0,0]) 
blue_high = np.array([255,10,10]) 
coordinatesColor = [[1, 1.5], [2, 1.5], [3, 1.5]] 
coordinatesQR = [[1, 0.5], [2, 0.5], [3, 0.5], [4, 0.5]] 

def led(color):  
    red, green, blue = 0, 0, 0 
    if color == 'green':
        red, green, blue = 0, 255, 0
    elif color == 'blue':
        red, green, blue = 0, 0, 255
    elif color == 'red':
        red, green, blue = 255, 0, 0
    elif color == 'no':
        red, green, blue = 0, 0, 0
    set_effect(r=red, g=green, b=blue)


def qr_check(data): 
    frame = bridge.imgmsg_to_cv2(data, 'bgr8') 
    barcodes = qr_read(frame) 
    global colorQR
    global cup
    if barcodes: 
        result = barcodes[0].data 
        colorQR = result
        cup = False 
        image_sub.unregister() 
        
def colorDetect(data): 
    global color 
    global cup
    global red_low, red_high
    global green_low, green_high
    global blue_low, blue_high
    img = bridge.imgmsg_to_cv2(data, "bgr8") 
    mask_red = cv.inRange(img, red_low, red_high)     
    mask_green = cv.inRange(img, green_low, green_high) 
    mask_blue = cv.inRange(img, blue_low, blue_high)
    st1 = cv.getStructuringElement(cv.MORPH_RECT, (21, 21), (10, 10))
    st2 = cv.getStructuringElement(cv.MORPH_RECT, (11, 11), (5, 5))
    thresh = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, red, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Red
    for c in red:
        try: # Red
            moments =  cv.moments(c, 1) 
            sum_pixel = moments['m00'] 
            if sum_pixel > 4000: 
                color = 'red' 
                cup = False 
        except:pass
    thresh = cv.morphologyEx(mask_green, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, green, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Green
    for c in green:
        try:   # Green
            moments = cv.moments(c, 1)
            sum_pixel = moments['m00']
            if sum_pixel > 4000:
                cup = False
                color = 'green'
        except:pass    
    thresh = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, blue, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Blue
    for c in blue:
        try: # Blue
            moments = cv.moments(c, 1)
            sum_pixel = moments['m00']
            if sum_pixel > 4000:
                color = 'blue'
                cup = False
        except:pass    
    if cup == False: 
        image_pub.unregister() 

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2): 
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target') 
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True) 
for xy in coordinatesColor: 
    x, y = xy[0], xy[1]
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map')
    rospy.sleep(1) # Ждем одну секунду
    image_pub = rospy.Subscriber('main_camera/image_raw', Image, colorDetect, queue_size=1) 
    while cup == True:
        rospy.sleep(0.5) 
    print(color) 
    cup = True
    XY_Color[color] = xy 

print("\n{}\n".format(XY_Color))
print('\n||||||||||||||||||||||||||||||||||||||||||||||||\n')
mas = ['', '', '', ''] 
for i in range(4):
    x, y = coordinatesQR[i][0], coordinatesQR[i][1] 
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map') 
    rospy.sleep(1) 
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, qr_check, queue_size=1)  
    while cup == True:
        rospy.sleep(0.5) 
    cup = True 
    print(colorQR) 
    x_d, y_d = XY_Color[colorQR] 
    navigate_wait(x=x_d, y=y_d, z=0.5, speed=0.5, frame_id='aruco_map')
    led(colorQR) 
    rospy.sleep(3) 
    led('no') 
    mas[i] = colorQR 
    
print('\n finish')
    
navigate_wait(x=0, y=0, z=0.5, speed=0.5, frame_id='aruco_map') 
print('program successful') 
land() 
f = open('Report.txt', 'w')
for i in range(1, 5):
    f.write("Department {}: {}\n".format(i, mas[i-1])) 
f.close() 
