# 종합 지금까지 모든 걸 종합해서 만든 노드
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from util import BEVTransform, CURVEFit, draw_lane_img

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None
        self.img_lane = None
        self.edges = None

        print("you need to find the right value : line 23 ~ 29")
        self.lower_wlane = np.array([0,0,0])
        self.upper_wlane = np.array([0,0,0])    #

        self.lower_ylane = np.array([0,0,0])
        self.upper_ylane = np.array([0,0,0])

        self.crop_pts = np.array([[[0,0],[0,0],[0,0],[0,0]]])

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

    def binarize(self, img):    # 이진화

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
        img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)

        self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        return self.img_lane

    def mask_roi(self, img):    #mask를 만들자 mask는 차선 넓이 일것이다. img는 밑에서 정의한다.

        h = img.shape[0]    #shape는 [높이,너비,채널]
        w = img.shape[1]
        if len(img.shape)==3:   #img.shape의 채널 수가 3이면
        # num of channel = 3

            c = img.shape[2]    #c는 채널 수
            mask = np.zeros((h, w, c), dtype=np.uint8)  #np.zero(높,너비,채널) 안에 있는 이미지 정보와 똑같은 0인 배열을 만듦 즉 여기서는 이미지와 똑같은 검정 이미지 생성

            mask_value = (255, 255, 255)    #마스크는 3채널의 흰색

        else:
        # grayscale

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)   #cv2.fillPoly(이미지, 다각형의 꼭짓점 좌표 배열, 색상) 

        mask = cv2.bitwise_and(mask, img)

        return mask

if __name__ == '__main__':
    rp = rospkg.RosPack()  # ros 통신을 위해서 토픽 주고 받을 떄 씀
    currentPath = rp.get_path("beginner_tutorials") # 딴 노드에서 받아올때 쓰자 (절대경로 받아오는 함수)
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp: # open() 파일열때 "r" 파일을 읽기 모드로 열다
        sensor_params = json.load(fp)   #열린파일(fp)에서 json 파일을 파이썬 객체로 변환
    # 파이썬에서 with는 파일을 자동으로 닫아줌 즉 파일 자원 사용과 안전하게 사용가능
    #os.path.join(currentPath, 'sensor/sensor_params.json') 지금 파일경로(currentPath는 절대경로다) + "sensor/sensor_params.json" 경로를 합쳐줌
    
    params_cam = sensor_params["params_cam"]    #sener_parms에서 parms_came이라는 키에 해당하는 값을 가져옴
    # 참고 sensor_params는 json 파일로 키와 값이 쌍으로 저장되었잇음 // 카메라 관련 매게 변수 사용
    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)    #딴데서 사용할 가능성이 많다
    curve_learner = CURVEFit(order=3, lane_width=3.5 ,y_margin=1, x_range=30, min_pts=50)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():  #노드 종료시 true 반환, 종료 전까지 반복

        if  image_parser.img_bgr is not None:   #노드 할당 시만 반복한다는 뜻

            img_crop = image_parser.mask_roi(image_parser.img_bgr)

            img_warp = bev_op.warp_bev_img(img_crop)

            img_lane = image_parser.binarize(img_warp)  #이진화

            img_f = bev_op.warp_inv_img(img_lane)

            lane_pts = bev_op.recon_lane_pts(img_f)

            x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)

            curve_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)

            curve_learner.pub_path_msg()

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_lane_fit = draw_lane_img(img_lane, xyl[:, 0].astype(np.int32),
                                            xyl[:, 1].astype(np.int32),
                                            xyr[:, 0].astype(np.int32),
                                            xyr[:, 1].astype(np.int32))

            cv2.imshow("birdview", img_lane_fit)
            cv2.imshow("img_warp", img_warp)
            cv2.imshow("origin_img", image_parser.img_bgr)

            cv2.waitKey(1)

        rate.sleep()
