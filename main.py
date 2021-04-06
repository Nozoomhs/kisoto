from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
from time import sleep
# color range
lower_red1 = np.array([0,160,220])
upper_red1 = np.array([5,255,255])
lower_red2 = np.array([170,160,220])
upper_red2 = np.array([180,255,255])
lower_green = np.array([35,80,100])
upper_green = np.array([70,255,255])
lower_yellow = np.array([23,100,150])
upper_yellow = np.array([35,255,255])
kernel = np.ones((5, 5), np.uint8)  ## maszk kreálása
##---------------------------------------------------------------------------------
##Serial-----
ser = serial.Serial(
    port='/dev/ttyS0',  # Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
ser.flush()
##-----------
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT)
GPIO.output(40, GPIO.HIGH)

camera = PiCamera()
camera.resolution = (640, 480)
camera.rotation = 0
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
font = cv2.FONT_HERSHEY_SIMPLEX

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    ##line detection----------------------------------------
    image = frame.array
    hsv = image[0:240, 0:640] #roi-lámpa kereséshez.
    imgcopy = image  ##ne rontsuk el az eredeti képet a táblafeldolgozáshoz
    roi = image[400:480, 0:640]
    Blackline = cv2.inRange(roi, (0, 0, 0), (80, 80, 80))
    kernel = np.ones((3, 3), np.uint8)
    Blackline = cv2.erode(Blackline, kernel, iterations=2)
    Blackline = cv2.dilate(Blackline, kernel, iterations=3)
    cv2.imshow("dilated", Blackline)
    img, contours, hierarchy = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area > 5500:
            x, y, w, h = cv2.boundingRect(contours[i])
            cv2.rectangle(image[300:480, 0:640], (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.line(image, (x + (w // 2), 400), (x + (w // 2), 480), (255, 0, 0), 3)
            dist = (x + (w // 2))
        ser.write(str(dist).encode('ascii'))
        ser.write('k'.encode('ascii'))
        print(dist)
        cv2.imshow("orginal with line", image)
    ##------------------------------------------------------------------------------

    hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    maskr = cv2.add(mask1, mask2)
    maskg = cv2.inRange(hsv, lower_green, upper_green)

    maskr = cv2.dilate(maskr, kernel, iterations=2)
    maskg = cv2.dilate(maskg, kernel, iterations=1)
    cv2.imshow("maskr", maskr)
    cv2.imshow("maskg", maskg)
    cv2.imshow("eredeti", image)
    # hough circle detect
    r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
                                 param1=50, param2=5, minRadius=60, maxRadius=100)

    g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 80,
                                 param1=50, param2=5, minRadius=60, maxRadius=60)
    #

    if r_circles is not None:
        print("Red")
    if g_circles is not None:
        print("green")

    threshold1 = 70
    threshold2 = 255
    roi_table = imgcopy[0:240, 320:640]
    cv2.imshow("roi_table", roi_table)

    grayvideo = cv2.cvtColor(roi_table, cv2.COLOR_BGR2GRAY)
    # blur =cv2.bilateralFilter(grayvideo,7,200,200)
    blur2 = cv2.medianBlur(grayvideo, 11)
    # bilblur=cv2.bilateralFilter(grayvideo,7,50,50)

    imgCanny = cv2.Canny(blur2, 55, 255)
    # ret3,th3 = cv2.threshold(grayvideo,threshold1,threshold2,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow("Canny", imgCanny)
    blur2 = cv2.medianBlur(imgCanny, 1)
    # cv2.imshow("blur2", blur2)

    dilated = cv2.dilate(imgCanny, kernel, iterations=1)

    _, contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # print(len(contours))
    ##area of conotur and boundig rect
    for contour in contours:  ##area of conotur and boundig rect
        area = cv2.contourArea(contour)
        if area > 5000:
            peri = cv2.arcLength(contour, True)

            approx = cv2.approxPolyDP(contour, 0.01 * peri, True)
            x1, y1, w1, h1 = cv2.boundingRect(approx)
            objCor = len(approx)
            if objCor == 8:

                Type = "octa"
                cv2.imshow("imgcopy", imgcopy)
                roi2 = roi_table[y1:y1 + h1, x1:x1 + w1]
                ser.write('s'.encode('ascii'))
                cv2.imshow("roi", roi2)

            else:
                Type = "None"
            # cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.rectangle(roi_table, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 0), 1)
            cv2.putText(roi_table, Type, (x1, y1), font, 0.5, (0, 255, 255), 2)

    rawCapture.truncate(0)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
cv2.destroyAllWindows()
GPIO.output(40, GPIO.LOW)