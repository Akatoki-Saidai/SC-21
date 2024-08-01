# camera lib

import cv2
import numpy as np


# 測定値の出力用
import csv_print as csv


class Camera:
    def red_detect(self, frame):
        # HSV色空間に変換
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 赤色のHSVの値域1
        hsv_min = np.array([0, 117, 104])
        hsv_max = np.array([11, 255, 255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色のHSVの値域2
        hsv_min = np.array([169, 117, 104])
        hsv_max = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        return mask1 + mask2


    def analyze_red(self, frame, mask):
            
        camera_order = 0
        # 画像の中にある領域を検出する
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
        #画像の中に赤の領域があるときにループ
        if 0 < len(contours):
                        
            # 輪郭群の中の最大の輪郭を取得する-
            biggest_contour = max(contours, key=cv2.contourArea)

            # 最大の領域の外接矩形を取得する
            rect = cv2.boundingRect(biggest_contour)

            # #最大の領域の中心座標を取得する
            center_x = (rect[0] + rect[2] // 2)
            center_y = (rect[1] + rect[3] // 2)
            csv.print('camera_center', [center_x, center_y])

            # 最大の領域の面積を取得する-
            area = cv2.contourArea(biggest_contour)
            csv.print('camera_area', area)

            # 最大の領域の長方形を表示する
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

            # 最大の領域の中心座標を表示する
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # 最大の領域の面積を表示する
            cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

            cv2.putText(frame, str(center_x), (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)


            frame_center_x = frame.shape[1] // 2
            csv.print('camera_frame_size_x', frame.shape[1])

            # 中心座標のx座標が画像の中心より大きいか小さいか判定
            if area > 12000:
                print("Close enough to red")
                csv.print('camera_order', 'close')
                camera_order = 4

            elif area > 150:
                if frame_center_x -  50 <= center_x <= frame_center_x + 50:
                    print("The red object is in the center")#直進
                    csv.print('camera_order', 'center')
                    camera_order = 1
                elif center_x > frame_center_x + 50:
                    print("The red object is in the right")#右へ
                    csv.print('camera_order', 'right')
                    camera_order = 2
                elif center_x < frame_center_x - 50:
                    print("The red object is in the left")#左へ
                    csv.print('camera_order', 'left')
                    camera_order = 3

            else:
                print("The red object is too minimum")
                csv.print('camera_order', 'not found')

            # red_result = cv2.drawContours(mask, [biggest_contour], -1, (0, 255, 0), 2)
        
        else:
            print("The red object is None")
            csv.print('camera_order', 'none')
            
        cv2.imshow("Frame", frame)
        # cv2.imshow("Mask", mask)

        return camera_order

