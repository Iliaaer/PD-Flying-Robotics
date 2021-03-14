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

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) # Получить полную телеметрию коптера
navigate = rospy.ServiceProxy('navigate', srv.Navigate)   # Прилететь в обозначенную точку по прямой.
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal) # Полет по прямой в точку в глобальной системе координат (широта/долгота).
set_position = rospy.ServiceProxy('set_position', srv.SetPosition) # Установить цель по позиции и рысканью. Данный сервис следует использовать при необходимости задания продолжающегося потока целевых точек, например, для полета по сложным траекториям (круговой, дугообразной и т. д.).
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity) # Установить скорости и рысканье
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude) # Установить тангаж, крен, рысканье и уровень газа (примерный аналог управления в режиме STABILIZED). Данный сервис может быть использован для более низкоуровнего контроля поведения коптера либо для управления коптером при отсутствии источника достоверных данных о его позиции.
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates) # Установить угловые скорости по тангажу, крену и рысканью и уровень газа (примерный аналог управления в режиме ACRO). Это самый низкий уровень управления коптером (исключая непосредственный контроль оборотов моторов). Данный сервис может быть использован для автоматического выполнения акробатических трюков (например, флипа).
land = rospy.ServiceProxy('land', Trigger) # Перевести коптер в режим посадки (AUTO.LAND или аналогичный).
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect) # определить прокси для ROS-сервиса светодиодной ленты

bridge = CvBridge()  # От сюда брать изображение с камеры
 
qr_debug = rospy.Publisher("/qr_debug", Image, queue_size=10) # Сюда можно опубликовать изобрадение
image_color = rospy.Publisher("/Debuuuuuuuug",Image,queue_size=10) #  определить прокси для ROS-сервиса


colorQR = ''
color = ''
cup = True
XY_Color = {'red': [0, 0], 'green': [0, 0], 'blue': [0, 0]} # Чтобы хранить координаты определенного цвета


red_low = np.array([0,0,240]) # Минимальное значение для распознование крассного 
red_high = np.array([10,10,255]) # Максимальное значение для распознование крассного 

green_low = np.array([0,240,0]) # Минимальное значение для распознование зеленого   
green_high = np.array([10,255,10]) # Минимальное значение для распознование зеленого 

blue_low = np.array([240,0,0]) # Минимальное значение для распознование синего 
blue_high = np.array([255,10,10]) # Минимальное значение для распознование синего 


coordinatesColor = [[1, 1.5], [2, 1.5], [3, 1.5]] # Координаты оборудование 
coordinatesQR = [[1, 0.5], [2, 0.5], [3, 0.5], [4, 0.5]] # Координаты QR кодов


def led(color):  # функция для включени/выключение LED ленты
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


def qr_check(data): # Функция для распознование QR кодов
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = qr_read(frame)  # read the barcode using zbar
    global colorQR
    global cup
    if barcodes:
        result = barcodes[0].data 
        colorQR = result
        cup = False
        # draw rect and publish to topic
        #(x, y, w, h) = barcodes[0].rect
        #cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        #qr_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
        image_sub.unregister()

def colorDetect(data): # Функция для распознование цветных маркеров
    global color
    global cup
    global red_low, red_high
    global green_low, green_high
    global blue_low, blue_high

    try:    
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except:pass

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
            x, y = 0, 0
            moments = cv.moments(c, 1)
            sum_y = moments['m01']
            sum_x = moments['m10']
            sum_pixel = moments['m00']
            if sum_pixel > 4000:
                color = 'red'
                cup = False
                y = int(sum_x / sum_pixel)*320//70
                x = int(sum_y / sum_pixel)*240//70
                #cv.putText(img, 'Red', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))     
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
        except:pass
    thresh = cv.morphologyEx(mask_green, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, green, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Green
    for c in green:
        try:   # Green
            x, y = 0, 0
            moments = cv.moments(c, 1)
            sum_y = moments['m01']
            sum_x = moments['m10']
            sum_pixel = moments['m00']
            if sum_pixel > 4000:
                cup = False
                color = 'green'
                y = int(sum_x / sum_pixel)*320//70
                x = int(sum_y / sum_pixel)*240//70
                #cv.putText(img, 'Green', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
        except:pass
            
    thresh = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, blue, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Blue
    for c in blue:
        try: # Blue
            x, y = 0, 0
            moments = cv.moments(c, 1)
            sum_y = moments['m01']
            sum_x = moments['m10']
            sum_pixel = moments['m00']
            if sum_pixel > 4000:
                color = 'blue'
                cup = False
                y = int(sum_x / sum_pixel)*320//70
                x = int(sum_y / sum_pixel)*240//70
                #cv.putText(img, 'Blue', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
        except:pass

        
    #try:
    #    image_color.publish(bridge.cv_to_imgmsg(img, "bgr8"))
    #except:pass
    
    if cup == False:
        image_pub.unregister()

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):  # Полет в точку и ожидание окончания полета
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)



navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True) # Взлет с старта 

for xy in coordinatesColor: # Летаем по координатам оборудованием, и распознаем их цвет 
    x, y = xy[0], xy[1]
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map')
    rospy.sleep(1)
    image_pub = rospy.Subscriber('main_camera/image_raw', Image, colorDetect, queue_size=1) 
    while cup == True:
        rospy.sleep(0.5)
    print(color)
    cup = True
    XY_Color[color] = xy

print("\n{}\n".format(XY_Color))
print('\n||||||||||||||||||||||||||||||||||||||||||||||||\n')

mas = ['', '', '', '']

for i in range(4): # Летаем по координатам QR кодов, и распознаем и тд
    x, y = coordinatesQR[i][0], coordinatesQR[i][1]
    
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map')
    rospy.sleep(1)
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, qr_check, queue_size=1)  # try to read qr
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
    
    
print('finish')
    
    
navigate_wait(x=0, y=0, z=0.5, speed=0.5, frame_id='aruco_map') # Летим на старт 
print('program successful') 
land() # Садимся
f = open('Report.txt', 'w') # Делаем отчет в 'Report.txt'
for i in range(1, 5):
    f.write("Department {}: {}\n".format(i, mas[i-1]))
f.close()

