#!/usr/bin/env python
# coding: utf-8

from drone_control import *
from geometry_msgs.msg import Point
import rospy
import numpy as np
import cv2, time

CONT_AREA_MAX_SIZE = 400
CUBE_AREA_MAX_SIZE = 4000
BOX_AREA_MIN_SIZE = 15000

SHOW_IMAGE = True


def delta_dist(e_dist, h=1.7):
    a = 0.09 / 993
    b = a * 7 - 0.01
    return abs(e_dist) * a + b


def camera_center_x(h=1.7):
    return 350


def camera_center_y(h=1.7):
    return 300


def cont_area_max_size(h=1.7):
    if 1.55 < h < 1.85:
        return 300

    return 400


def box_area_min_size(h=1.7):
    return (1.7 * 3500 / (h ** 2))


def cube_area_max_size(h=1.7):
    return (1.7 * 1800 / (h ** 2))


def main():
    box_x = 0
    box_y = 0

    rospy.init_node("test_node", anonymous=10)


    drone = drone_client()

    rate = rospy.Rate(20)

    k_p = 0.4

    epsilon_dist = 0.1

    epsilon_cam = 0.01

    aim_dots = []

    aim_x, aim_y, aim_z = aim_dots.pop(0)

    cur_state = -1
    '''
    -1 - box finding
    0 - getting world coords of box
    1 - box found,, we know coords, finding cubes
    2 - one cube found, we know where to place it, going down to cube
    3 - using gripper 1
    4 - going up
    5 - go to box by probably coords
    6 - aiming to box
    7 - going down
    8 - gripper 0
    9 - go up
    10 - GOTO 1
    11 - TODO finish checking
    '''
    n = 0

    def update_aims():
        global aim_dots
        aim_dots = []
        aim_dots.append([1.5, -1.5, 1.7])
        aim_dots.append([0.4, -0.6, 1.7])
        aim_dots.append([2.5, -0.6, 1.7])
        aim_dots.append([2.5, -2.5, 1.7])
        aim_dots.append([0.4, -2.5, 1.7])
        aim_dots.append([0.4, -0.6, 1.7])

    update_aims()

    def set_goal(goal_pose, angle):
        print("GOAL:", goal_pose)
        if 0.3 < goal_pose.x < 2.6 and -0.3 > goal_pose.y > -2.7:
            drone.SetGoal(goal_pose, angle)
            pass
        else:
            print("WRONG GOAL")

    while (rospy.is_shutdown() is False):
        n += 1
        x0 = drone.current_position.x
        y0 = drone.current_position.y
        z0 = drone.current_position.z

        if cur_state == -1 or cur_state == 1:

            e_x = aim_x - x0
            e_y = aim_y - y0
            e_z = aim_z - z0
            e_dist = np.sqrt(e_x ** 2 + e_y ** 2 + e_z ** 2)

            goal_pose = Point()

            goal_pose.x = x0 + (e_x / e_dist) * k_p
            goal_pose.y = y0 + (e_y / e_dist) * k_p
            goal_pose.z = z0 + (e_z / e_dist) * k_p

            if e_dist > epsilon_dist:

                set_goal(goal_pose, 0)
            else:
                if len(aim_dots) != 0:
                    aim_x, aim_y, aim_z = aim_dots.pop(0)
                else:
                    print("FINISH STATE")
                    update_aims()
                    cur_state = -1

        # time.sleep(0.1)

        im = drone.cv_image
        if not im is False:
            # cv2.imwrite("./image2/image{0}.jpg".format(n), im)
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
            cubes = []
            for cnt in contours:

                if cv2.contourArea(cnt) > cont_area_max_size(z0):
                    M = cv2.moments(cnt)
                    if M["m00"] == 0:
                        M["m00"] = 0.00000001
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(img_without_gray_light, (cX, cY), 5, (0, 0, 255), -1)
                    name_of_object = "UNKNOWN"

                    if cv2.contourArea(cnt) < cube_area_max_size(z0):
                        name_of_object = "CUBE"
                        cubes.append([cX, cY])




                    elif cv2.contourArea(cnt) > box_area_min_size(z0):
                        name_of_object = "BOX"
                        if cur_state == -1:
                            cur_state = 0
                            print("CHANGING STATE TO 0")
                        if cur_state == 0:

                            print("BOX FOUND")
                            e_y = -(cX - camera_center_x(z0))
                            e_x = -(cY - camera_center_y(z0))
                            print("e_x {0} e_y {1} ".format(e_x, e_y))
                            e_dist = np.sqrt(e_x ** 2 + e_y ** 2)
                            if z0 > 0.8:
                                d_z = -0.02
                            else:
                                d_z = 0
                                # TODO CHECKING STATE CHANCE
                                if delta_dist(e_dist) * e_dist < epsilon_cam:
                                    box_x = x0
                                    box_y = y0

                                    update_aims()
                                    cur_state = 1
                                    continue

                            goal_pose = Point()

                            goal_pose.x = x0 + (e_x / e_dist) * delta_dist(e_dist)
                            goal_pose.y = y0 + (e_y / e_dist) * delta_dist(e_dist)
                            goal_pose.z = z0 + d_z

                            set_goal(goal_pose, 0)

                    if cur_state == 1 and len(cubes) > 0:
                        cur_state = 2
                    if cur_state == 2:
                        if len(cubes) > 0:
                            cubes.sort(
                                key=lambda c: np.sqrt(
                                    (c[0] - camera_center_x()) ** 2 - (c[1] - camera_center_y()) ** 2))

                            cube_x, cube_y = cubes[0]

                            print("CUR STATE 2")
                            print("GOING TO CUBE", cube_x, cube_y)
                            e_y = -(cube_x - camera_center_x(z0))
                            e_x = -(cube_y - camera_center_y(z0))
                            e_dist = np.sqrt(e_x ** 2 + e_y ** 2)

                            if z0 > 0.4:  # TODO CHECK VYRAVNIVANIE

                                d_z = -0.04
                            elif z0 > 0.05:
                                d_z = -0.07
                                if e_dist * delta_dist(e_dist) < 0.005:  # TODO EPSILON CUBE CAMERA
                                    e_y = 0
                                    e_x = 0

                            else:

                                cur_state = 3
                                time.sleep(0.2)
                                print("GRIPPER TIME!!!!!!")

                                continue

                            print("e_x {0} e_y {1} ".format(e_x, e_y))
                            goal_pose = Point()

                            goal_pose.x = x0 + (e_x / e_dist) * delta_dist(e_dist)
                            goal_pose.y = y0 + (e_y / e_dist) * delta_dist(e_dist)
                            goal_pose.z = z0 + d_z

                            set_goal(goal_pose, 0)
                        else:
                            cur_state = 1
                            print("CHANGING STATE TO 1")
                    cv2.putText(img_without_gray_light, name_of_object, (cX - 30, cY - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    # print(cv2.contourArea(cnt))
                    conts.append(cnt)

            cv2.drawContours(img_without_gray_light, conts, -1, (0, 255, 0), 3)

            cv2.putText(img_without_gray_light, "x:{0} y:{1} z:{2}".format(x0, y0, z0), (20, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.circle(img_without_gray_light, (camera_center_x(z0), camera_center_y(z0)), 5, (255, 255, 255), -1)

            new_img = np.vstack((im, img_without_gray_light))
            cv2.imwrite("./test/image{0}.jpg".format(n), new_img)
            if SHOW_IMAGE:
                cv2.imshow("im", img_without_gray_light)
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
        else:
            print("NO CAMERA")
        rate.sleep()


if __name__ == "__main__":
    main()
