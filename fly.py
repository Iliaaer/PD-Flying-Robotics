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

rospy.init_node('flight') # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
'''
В рамках вызова init_node () вы передадите имя вашего узла по умолчанию. 
Когда вы запускаете свой код, это имя, которое ваш узел будет отображаться как подключенный, если он не будет переопределен путем переназначения аргументов. 
Имена имеют важные свойства в ROS. Самое главное, они должны быть уникальными. 
В случаях, когда вам не нужны уникальные имена для конкретного узла, вы можете инициализировать узел анонимным именем.
'''

'''
Про все rospy.ServiceProxy написано здесь 
https://clover.coex.tech/ru/simple_offboard.html
'''

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
 
qr_debug = rospy.Publisher("/qr_debug", Image, queue_size=10) # Сюда можно опубликовать изображение чтобы можно было его посмотерть 
image_color = rospy.Publisher("/Debuuuuuuuug",Image,queue_size=10) #  Сюда можно опубликовать изображение чтобы можно было его посмотерть 
'''
Про rospy.Publisher
http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
'''



colorQR = '' # Сюда будем сохранять с функции то значени, которое было зашифровано в QR коду
color = '' # Сюда будем сохранять какой цвет дрон только распознал
cup = True # Переменная для отслеживания выхода из функции
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
    red, green, blue = 0, 0, 0 # В эти переменные сохраням по каждому компоненту RGB и потом передаем в функцию set_effect Чтобы включит ленту
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
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')  # Берем изображение с камеры 
    barcodes = qr_read(frame)  # Находим QR коды в изображении 
    global colorQR # Подключаем глобальные переменные 
    global cup
    if barcodes: # Если в изображении присутствует QR код 
        result = barcodes[0].data # Берем значение, что было зашифровано в QR коде 
        colorQR = result # Сохраняем значение в глобольную переменную 
        cup = False # Изменяем на False, чтобы можно было понять что он распознал QR код
        #(x, y, w, h) = barcodes[0].rect
        #cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        #qr_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
        image_sub.unregister() # Завершает работу этой функции, чтобы она не работала бесконечно 

def colorDetect(data): # Функция для распознование цветных маркеров
    global color # Подключаем глобальные переменные 
    global cup
    global red_low, red_high
    global green_low, green_high
    global blue_low, blue_high

    img = bridge.imgmsg_to_cv2(data, "bgr8") # Берем изображение с камеры 

    mask_red = cv.inRange(img, red_low, red_high)     
    mask_green = cv.inRange(img, green_low, green_high) 
    mask_blue = cv.inRange(img, blue_low, blue_high)
 '''
 Функция inRange позволяет наложить на кадр цветовой фильтр в заданном диапазоне.
 # img — изображение, на которое мы накладываем фильтр;
 red_low/green_low/blue_low — начальный цвет диапазона;
 red_high/green_high/blue_high — конечный цвет диапазона.
 '''

    st1 = cv.getStructuringElement(cv.MORPH_RECT, (21, 21), (10, 10))
    st2 = cv.getStructuringElement(cv.MORPH_RECT, (11, 11), (5, 5))
   '''   cv.getStructuringElement
   https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc
   Возвращает структурирующий элемент указанного размера и формы для морфологических операций. 
   Функция конструирует и возвращает элемент структурирования, который в дальнейшем может быть передан в функции erode, dilate или morphologyEx. 
   Но вы также можете самостоятельно создать произвольную двоичную маску и использовать ее в качестве элемента структурирования.
   '''

   
   '''cv.morphologyEx
   https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#ga67493776e3ad1a3df63883829375201f
   Выполняет сложные морфологические преобразования. 
   Функция cv :: morphologyEx может выполнять расширенные морфологические преобразования, используя в качестве основных операций эрозию и расширение.
   Любые операции можно выполнить на месте. В случае многоканальных изображений каждый канал обрабатывается независимо.
   '''
    thresh = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, st1)
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, st2)
    _, red, hier = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)   #Red
    for c in red:
        try: # Red
            moments =  cv.moments(c, 1) 
         ''' cv.moments
         https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga556a180f43cab22649c23ada36a8a139
         Вычисляет все моменты до третьего порядка многоугольника или растрированной формы. 
         Функция вычисляет моменты до 3-го порядка векторной или растеризованной формы. 
         Результаты возвращаются в структуре cv :: Moments.
         '''
            sum_pixel = moments['m00'] # https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html#ab8972f76cccd51af351cbda199fb4a0d
            if sum_pixel > 4000: # Если сумма пикселя в этом обекте больше 4000 то
                color = 'red' # Сохраням цвет который он распознал
                cup = False  # Изменяем на False, чтобы можно было понять что он распознал QR код

                #cv.putText(img, 'Red', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))     
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
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
             
                #cv.putText(img, 'Green', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
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
             
                #cv.putText(img, 'Blue', (y, x), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                #cv.drawContours(img, [c], 0, (193,91,154), 2)
        except:pass
    '''    cv.findContours
    https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html
    Контуры можно объяснить просто как кривую, соединяющую все непрерывные точки (вдоль границы), имеющие одинаковый цвет или интенсивность. 
    Контуры - полезный инструмент для анализа формы и обнаружения и распознавания объектов
    '''
        
    #try:
    #    image_color.publish(bridge.cv_to_imgmsg(img, "bgr8"))
    #except:pass
    
    if cup == False: # Если он что то распознал, тоесть перевел эту переменную в значение False, то
        image_pub.unregister() # Завершает работу этой функции, чтобы она не работала бесконечно 

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):  # Полет в точку и ожидание окончания полета
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm) # Летит в определеную точку

    while not rospy.is_shutdown(): # Ожидает пока дрон не будет в определенной точке  
        telem = get_telemetry(frame_id='navigate_target') # Получить полную телеметрию коптера
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)



navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True) # Взлет с старта 

for xy in coordinatesColor: # Летаем по координатам оборудованием, и распознаем их цвет 
    x, y = xy[0], xy[1] # Сохраняем координаты цветного маркера
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map') # Летим до цветного маркера
    rospy.sleep(1) # Ждем одну секунду
    image_pub = rospy.Subscriber('main_camera/image_raw', Image, colorDetect, queue_size=1)  # Создаем Subscriber и вызываем функцию colorDetect
    while cup == True:
        rospy.sleep(0.5) # Ожидает переменую cup, через которую он понимает что функция что то нашла
    print(color) # Выводит в терминал что распознал 
    cup = True # Переводит переменую в True чтобы можно было дальше пользоваться функциями для определение цвета/QR кодов
    XY_Color[color] = xy # Сохраняем с словарь координаты этого цвета, который он распознал 

print("\n{}\n".format(XY_Color))
print('\n||||||||||||||||||||||||||||||||||||||||||||||||\n')

mas = ['', '', '', ''] # Создаем список, куда будем сохранять что было зашифровано в QR кодах 

for i in range(4): # Летаем по координатам QR кодов, и распознаем и тд
    x, y = coordinatesQR[i][0], coordinatesQR[i][1] # Сохраняем координаты QR кода
    
    navigate_wait(x=x, y=y, z=0.5, speed=0.5, frame_id='aruco_map') # Летим до QR кода
    rospy.sleep(1) # Ждем одну секунду
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, qr_check, queue_size=1)  # Создаем Subscriber и вызываем функцию qr_check
    while cup == True:
        rospy.sleep(0.5) # Ожидает переменую cup, через которую он понимает что функция что то нашла
    cup = True # Переводит переменую в True чтобы можно было дальше пользоваться функциями для определение цвета/QR кодов
    print(colorQR) # Выводит в терминал что распознал 
    x_d, y_d = XY_Color[colorQR] # Берем из списка координаты цветного маркера, цвет маркера которого мы узнали в QR коду
    navigate_wait(x=x_d, y=y_d, z=0.5, speed=0.5, frame_id='aruco_map') # Летим до цветного маркера
    led(colorQR) # Включаем подцветку, цвет которого равен цвету маркера
    rospy.sleep(3) # Ждем 3 секунду
    led('no') # Выключаем подцветку
    mas[i] = colorQR # Сохраняем в список, чтобы было зашифровано в QR коду
    
    
print('\n finish')
    
    
navigate_wait(x=0, y=0, z=0.5, speed=0.5, frame_id='aruco_map') # Летит на старт 
print('program successful') 
land() # Садимся
f = open('Report.txt', 'w') # Делаем отчет в 'Report.txt'
for i in range(1, 5):
    f.write("Department {}: {}\n".format(i, mas[i-1])) # Записываем в файл Department и то что он распознал в QR коду 
f.close() # Закрываем отчёт

