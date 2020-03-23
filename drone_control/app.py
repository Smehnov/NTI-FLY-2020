import cv2
import numpy as np

from test import *

for im_num in range(200, 1000):

    drone_x = 0
    drone_y = 0
    drone_z = 0
    drone_h = 1.7

    print("IMAGE", im_num)
    im = cv2.imread("./image2/image{0}.jpg".format(im_num))
    print(im.shape)
    im = cv2.GaussianBlur(im, (7, 7), 0)

    GRAY_LOWER = np.array([0, 0, 35])
    GRAY_UPPER = np.array([255, 68, 240])

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GRAY_LOWER, GRAY_UPPER)
    mask = cv2.bitwise_not(mask)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    img_without_gray_light = cv2.bitwise_and(im, im, mask=mask)

    conts = []
    CONT_AREA_MAX_SIZE = 400
    CUBE_AREA_MAX_SIZE = 4000
    BOX_AREA_MIN_SIZE = 15000
    for cnt in contours:

        if cv2.contourArea(cnt) > cont_area_max_size(drone_h):
            print(cv2.contourArea(cnt))
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                M["m00"] == 0.00000001
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img_without_gray_light, (cX, cY), 5, (0, 0, 255), -1)
            name_of_object = "UNKNOWN"
            if cv2.contourArea(cnt) < cube_area_max_size(drone_h):
                name_of_object = "CUBE"
            elif cv2.contourArea(cnt) > box_area_min_size(drone_h):
                name_of_object = "BOX"

            cv2.putText(img_without_gray_light, name_of_object, (cX - 30, cY - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # print(cv2.contourArea(cnt))
            conts.append(cnt)

    cv2.drawContours(img_without_gray_light, conts, -1, (0, 255, 0), 3)

    cv2.putText(img_without_gray_light, "x:{0} y:{1} z:{2}".format(drone_x, drone_y, drone_z), (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("im", img_without_gray_light)
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break
