import numpy as np
import cv2, random, math, copy

Width = 1280
Height = 720

cap = cv2.VideoCapture("./xycar_track1.mp4") #이 비디로를 읽음
window_title = 'camera' #window 창이름은 camera

warp_img_w = 320
warp_img_h = 240

warpx_margin = 20
warpy_margin = 3

nwindows = 9
margin = 12
minpix = 5

lane_bin_th = 97

warp_src  = np.array([
    [654-warpx_margin, 385-warpy_margin],  #주어진 영상의 사다리꼴 좌표 값
    [370-warpx_margin,  687+warpy_margin],  #순서 우하단    좌상단    좌하단    우상단
    [445+warpx_margin, 385-warpy_margin],
    [691+warpx_margin, 687+warpy_margin]
], dtype=np.float32)

warp_dist = np.array([  #임의의 직사각형 좌표값
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], #1번째 축
        [0.0, 435.589734, 163.625535], #2번째 축
        [0.0, 0.0, 1.0]                #3번째 축    #따로 구해줘야함
    ]) # np.array([[],[],[]]) 배열만 드는 함수 [[],[],[]]인경우 2차원 배열 -> 여기선 3*3 배열 
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])#np.array([,,,])인 경우는 1차원배열
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))    #cv2.getOptmaNewCameraMatrix(카메라 메트릭스(mtx),왜곡 계수(dst),이미지크기(w,h),1,이미지 크기)
    #새로운 카메라 메트릭스, 관심공간(roi) 반환, 캘리브레이션 결과를 활용하여, 새로운 카메라 매트릭스를 구하는데 있어서 최적화된 값을 구해주는 함수
def calibrate_image(frame): #def 함수명(지역변수): #카메라 왜곡을 없앤(평평하게 만든) 이미지(tf_image)를 반환하는 함수
    global Width, Height #(640,480)   #global 변수 -> 전역변수(이 파이썬 파일에서 모두 사용할 변수) 선언 
    global mtx, dist        #width:너비 , Height 높이, mix : matrix 회전변환 행렬, dist : distance 거리, roi : ROI 이미지 내 관심영역
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx) #undistort(src, cemerMtrix,disCoeffs(왜곡계수)) 왜곡을 없애주는 함수 #여기서 frame ->src, mix(행렬) -> cemerMtrix,     
    x, y, w, h = cal_roi          #관심 영역의 x(x축),y(y축),w(너비),h(높이) 받아옴   #cemerMtrix : 촬영한 카메라 내부 파라미터인 중심점(cx,cy)와 초점거리(fx,fy)(dist 거리) 보통 행렬
    tf_image = tf_image[y:y+h, x:x+w]   #위에서 할당한 의미지(왜곡없엔 이미지)의 y값을 높이(h)로 x값을 너비(w)로 설정하여 이미지 바꿈

    return cv2.resize(tf_image, (Width, Height))    #resize(이미지, 절대크기,상대크기(fx,fy),보간법)이미지 크기 조절함수 tf_image를 width와 height로 변환

def warp_image(img, src, dst, size):     #원근맵 행렬(M,Minv)과 원근맵 행렬을 적용한 이미지(warp_img)를 반환하는 함수
    M = cv2.getPerspectiveTransform(src, dst)       #M(원근맵행렬) : getPerspectiveTransfrom(src(원본),dst(변환 후 결과))로 원근 맵 행렬 생성함 #M = src-> dst인 원근 맵 행렬 
    Minv = cv2.getPerspectiveTransform(dst, src)    #Minv -> dst -> src인 원근 맵 행렬
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR) #warpPerspective(src, M(원근맵행렬),dsize(출력이미지크기),flag(플레그,보간법 사용, 쌍 선형보관법이 가장 자주 사용)) 
               #이미지 기하학적 변환하는 함수         #쌍 선형 보간법(cv2.INTER_LINEAR) 확대할때 거의 이 플래그 사용
    return warp_img, M, Minv

def warp_process_image(img): #입력한 이미지를 차선을 흰색으로 나머지를 검정색으로 추출한 이미지에 왼쪽 차선의 여분 공간인 사각형을 그리고 차선을 왼쪽 차선은 파랑으로 오른쪽 차선을 빨강으로 칠한 이미지를 만들고 띄우는 함수 
    global nwindows #9 #global 앞 과 똑같이 
    global margin
    global minpix
    global lane_bin_th #145
                                            #sigmaX x방향 가우시안 커널 표준 편차
    blur = cv2.GaussianBlur(img,(5, 5), 0)  #cv2.GaussianBlur(src,ksize(커널크기 width, height ),sigmaX) 이미지에 가우신안 필터(gaussaian filter) 적용하는 함수-> 이미지에 잡음(노이즈)제거 이미지 소프트하게 만들어줌
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))#split 배열(채널)분리 함수 HSL에서 H는 색상, L은 밝기, S은 채도 여기선 L만 쓰겠다 나머지는 다 버리겠다라는 의미(_) 
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY) #밝기 채널 이진화시킴(흰색 차선 검출) 설정임계값, 결과이미지 = cv2.threshold(src,임계값, 최대값, 임계값 형식) 즉 lane은 결과이미지
    # 이때 hls 채널을 쓰는 이유 L 값을 사용하여 흰색 차선을 구분하기가 쉽다. 밝은 영역 255즉 흰색(차선),나머지 배경은 0(검은색 어두움) 숫자 클수록 밝다
    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)      #lane.shape[0] -> shape[] 배열은 [높이,너비,채널] 따라서 높이를 의미한다 "//" 은 정수 나눗셈(소수점 버림 즉 표시 X, 몫만표시)
    midpoint = np.int(histogram.shape[0]/2)      #midpoint-> 위 histogram의 높이 나누기 2 값 근데 int형 이므로 정수형   #axis = 0 x축방향(행방향,y좌표) =1 y축방향(열방향,x좌표) 
    leftx_current = np.argmax(histogram[:midpoint])   #왼쪽 차선의 위치 값(x좌표)          #[:숫자] 배열 앞에서부터 숫자번째까지만 추출(나머지 제거) -> 즉 여기선 histogram의 높이 나누기 2값(중간값)까지 추출한다는 뜻
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint #오른쪽 차선의 위치 값(x좌표) #[숫자 :] 숫자번째 부터 끝까지만 추출(나머지 제거) -> 즉 여기선 중간값부터 끝까지 추출한다는 뜻
    #np,sum([],축) axis =0은 x축 기준으로 합계 한 "행"(가로로)을 더함       #np.argmax(배열, 축) 축 기준으로 배열에서 가장 큰 값이 있는 차원(위치값)(여기선 가장 큰 값이 차선(255)값이다. 왜? 나머지값은 검정(0)이기 때문이다) 
                                                                # 즉 여기선 x축 기준으로 그 축(행)에서 가장 큰 값의 차원(y축, 행)의 값 의미 단 여기선 1이 아닌 0부터 시작한다.
    window_height = np.int(lane.shape[0]/nwindows)      #lane(L을 이진화시킨 이미지)의 높이를 nwindows 값으로 나눔(window_height 값)
    nz = lane.nonzero() #lane에서 0(검은색) 아닌 요소의 인덱스를 요소를 추출(즉 nz는 차선의 인덱스(위치)를 배열, 왜? lane이 차선을 제외한 모든것을 검정색(0)으로 바꾼 이미지이기 때문)
    #lane은 2차원 이미지(배열)이므로 lane.nonzero는 두 개의 튜플(두 개의 1차원 배열)(행 인덱스, 열 인덱스)을 반환한다 이때 nz[0]은 행 인덱스(y좌표들)를 nz[1]은 열 인덱스(x좌표들)를 의미한다  
    left_lane_inds = [] # 변수 = [] 아무것도 포함되지 않는 비어있는 리스트[]    
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], []

    out_img = np.dstack((lane, lane, lane))*255 #np.dstack((배열1,배열2, ...)) 3차원 배열을 만드는 함수 배열1,배열2를 깊이방향(axis =2)으로 쌓아서 3차원 배열만듬
    #out_img는 lane크기의 깊이가 3인 흰색 이미지이다.
    for window in range(nwindows):#window에 0-8까지 집어넣음(nwindow 값은 9)

        win_yl = lane.shape[0] - (window+1)*window_height       #win_y1 값은 lane 높이 - (window+1)*(lane높이/9) 즉 lane높이 - (0~9)*(lane높이/9)
        win_yh = lane.shape[0] - window*window_height           #win_yh 값은 lane 높이 - (window)*(lane 높이 / 9)     
        #win_y1<win_yh window_height만 큼 크다
        win_xll = leftx_current - margin   #win_xll은 좌측 차선의 일정한 여분 공간의 왼쪽 끝(시작 부분의미) # leftx_current 76줄 참고 histogram 배열에서 처음부터 midpoint값의 가장 큰 값이 있는 열의 행(y축)의 값
        win_xlh = leftx_current + margin   #win_x1h은 좌측 차선의 일정한 여분 공간의 오른쪽 끝(끝부분 의미)    win_x11(좌측)< win_x1h(우측)
        win_xrl = rightx_current - margin   #win_xr1은 우측 차선의 일정한 여분 공간의 왼쪽 끝(시작 부분)
        win_xrh = rightx_current + margin   #win_xrh은 우측 차선의 일정한 여분 공간의 오른쪽 끝(끝부분 의미)
        # margin = 12(일정의 여분의 공간)
        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) #cv2.rectangle(src, 좌상단 모서리좌표,우하단 모서리좌표,색상,두께)(결과 값에서 왼쪽 차선의 네모를 의미)
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) #결과이미지에서 오른쪽 차선의 네모를 의미
        #y축은 숫자가 작을 수록 높은 값이다. 좌상단 꼭짓점이 (0,0)
        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0] #왼쪽 네모의 차선(0이 아닌 값)의 y좌표(행방향)의 배열 구함
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0] #오른쪽 네모의 차선의 y좌표(행방향)의 배열 구함
        #nz[0]는 차선의 인덱스 행 방향 위치 배열(y좌표 배열)->여기선 차선의 y좌표 배열 #nz[1]은 차선의 인덱스 열 방향 위치 배열(x좌표 배열)->여기선 차선의 x좌표 배열
        left_lane_inds.append(good_left_inds)   #left_lane_inds는 83줄에서 빈 리스트로 선언함 즉 left_lane_inds 배열에 good_left_inds 추가 
        right_lane_inds.append(good_right_inds)
        #리스트1.append(리스트2) 리스트1의 끝에 리스트2 추가
        if len(good_left_inds) > minpix: #만약 good_left_inds의 길이(왼쪽 네모의 차선의 y좌표가 0이 아닌 값의 개수)가 minpix(5)보다 크면 (왜 굳이 if 를 쓰나? ->너무 작은 값은 차선이 아닐 가능성이 있기 때문)
            leftx_current = np.int(np.mean(nz[1][good_left_inds])) #mp.mean(배열) 배열의 모든 원소들의 평균값 반환, nz[1][good_left_inds]은 nz[1](열 인덱스) 중 good_left_inds를 인덱스로 갖는 값만 나옴 즉 window의 사이즈를 만족하는 x값만 나옴   
        if len(good_right_inds) > minpix:                          #leftx_current는 nz[x](왼쪽 사각형) 중 window 사이즈를 만족하고 차선일만큼 충분히 큰 값들의 x좌표들의 평균의 정수형이다.(소수점 버린 정수형태)= 새로운 기준점
            rightx_current = np.int(np.mean(nz[1][good_right_inds])) #rightx_current도 동일하게 오른쪽 사각형의 새로운 기준점이다.

        lx.append(leftx_current) #lx배열에 새로운 x좌표 기준점(왼쪽 차선의 사각형)을 추가 
        ly.append((win_yl + win_yh)/2)  # ly배열에 그린 사각형의 y길이의 가운데 좌표(세로의 중간점)를 붙임

        rx.append(rightx_current)   #rx배열에 새로운 x좌표 기준점(오른쪽 사각형)을 추가
        ry.append((win_yl + win_yh)/2) #ly 배열과 동일

    left_lane_inds = np.concatenate(left_lane_inds) #for문으로 반복하면서 늘어난 left_lane_inds을 모두 연결하여 left_lane_inds 배열을 깔끔하게 하나로 만듬
    right_lane_inds = np.concatenate(right_lane_inds)#위와 동일

    #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
    #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
    
    lfit = np.polyfit(np.array(ly),np.array(lx),2) #왼쪽 차선의 사각형에서 가장 적합한 x좌표 기준점들과 y좌표 기준점들의 2차 함수 (y = ax^2 +bx +c의 형태임)
    rfit = np.polyfit(np.array(ry),np.array(rx),2) #마찬가지로 오른쪽 차선의 사각형에서 가장 적합한 x좌표,y좌표 기준점들의 2차함수
    #np.polyfit(x,y,deg) deg-다항식 차수, 선형 회귀 모델을 구하는 함수, 즉 여러쌍의 (X,Y)에 가장 적합한 deg차 함수를 구함
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0] #nz[0][left_lane_inds]과 nz[1][left_lane_inds]에 해당하는 픽셀 값들을 파랑색[b,g,r]으로 바꿔줌
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]#nz[0][right_lane_inds]와 nz[1][right_lane_inds]에 해당하는 픽셀 값을 빨간색[b,g,r]으로 바꿔줌
    cv2.imshow("viewer", out_img) #창 이름이 viewer인 out_img 창을 보여줘라
    
    #return left_fit, right_fit
    return lfit, rfit   #lfit,rfit 값을 반환함

def draw_lane(image, warp_img, Minv, left_fit, right_fit):#차선 사이에 초록색으로 색칠한 후 기하학적 변환한 이미지인 warp_img를 다시 원상복구하고 newwarp와 image 합쳐서 반환하는 함수
    global Width, Height
    yMax = warp_img.shape[0]    #warp_img : 60줄에 존재 원근맵 배열인 M을 적용한 이미지, yMax는 이 이미지의 높이
    ploty = np.linspace(0, yMax - 1, yMax) #np.linspace(시작값,끝값, 생성할 샘플 수) 0~높이까지 1씩 샘플 생성
    color_warp = np.zeros_like(warp_img).astype(np.uint8) #warp_img 와 동일한 크기와 데이터 타입을 가지는 값이 0인 배열(np.zeros_likes) 타입(astype)을 np.uint8로 바꿈(8비트 0~255)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]  #left_fit 왼쪽 사각형에서 x좌표 기준점들과 y좌표 기준점들의 2차 함수
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2] #right_fit 오른쪽 x좌표 기준점들과 y좌표 기준점들의 2차 함수, * : 곱셈, ** : 제곱
    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))]) #np.vstack([배열1, 배열2]) 수직 방향으로 쌓임 배열1이 0행로 배열2가 2행으로 간다. 
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) #np.transpose(배열) 배열 축을 바꿈 1행 -> 1열,2행->2열 2*3행렬은 3*2행렬로, np.filpud(배열) 배열을 상하(y축)를 반전시킴 1행->3행,3행->1행(3by3)
    pts = np.hstack((pts_left, pts_right)) #np.hstack([배열1,배열2]) 수평방향으로 쌓임 <-> np.vstack 배열1이 0열로 배열2가 1열로 쌓인다.
    #pts_left가 왼쪽 1열로 pis_right가 오른쪽 2열로 쌓인다. #color_warp은 배열값이 모두 0인 wap_img와 동일한 배열
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0)) #cv2.fillPoly(src,[꼭짓점 배열],색깔) 꼭짓점들로 다각형을 만든뒤 색깔을 채우는 함수 #여기선 파란색과 빨간색(차선들)사이를 녹색으로 채운다.
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))#warpPerspective(src, M(원근맵행렬),dsize(출력이미지크기)) 기하학적 변환한 color_warp을 다시 원상복구 시킴

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)   #cv2.addWeight(이미지1,이미지1의 가중치,이미지2,두번째이미지)

def start():
    global Width, Height, cap   #전역변수 선언

    _, frame = cap.read() #cap은 동영상 #프레임 읽기 메서드(capture.read)를 이용하여 카메라의 상태 및 프레임을 받아옴, flame 현재시점 프레임 저장
    while not frame.size == (Width*Height*3): #프레임의 사이즈가 너비*높이*3이 아니면
        _, frame = cap.read() #다시 동영상 읽고 프레임 다시 저장함
        continue    #다시 반복을 시작함

    print("start")

    while cap.isOpened():   #cap이 올바르게 열렸는지 확인 ->올바르게 열리면 true반환 반복함 -> 잘못 열리면 false 값 반환 반복되지 않음
        
        _, frame = cap.read() #frame에 동영상 읽고 프레임 저장

        image = calibrate_image(frame)   #카메라 왜곡을 없앤(평평하게 만든) 이미지(tf_image)를 반환하는 함수
        warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))  #원근맵 행렬(M,Minv)과 원근맵 행렬을 적용한 이미지(warp_img)를 반환하는 함수
        left_fit, right_fit = warp_process_image(warp_img) #입력한 이미지를 차선을 흰색으로 추출한 이미지에 왼쪽 차선의 사각형을 그리고 차선을 왼쪽 차선은 파랑으로 오른쪽 차선을 빨강으로 칠한 이미지를 만들고 띄우는 함수
        #lfit, rfit :x좌표 기준점들과 y좌표 기준점들의 2차 함수(오른쪽 왼쪽 사각형)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)#차선 사이에 초록색으로 색칠한 후 기하학적 변환한 이미지인 warp_img를 다시 원상복구하고 newwarp와 image 합쳐서 반환하는 함수

        cv2.imshow(window_title, lane_img) # window_title = cremra로 창이름으로 lane_img를 보여줌

        cv2.waitKey(0)

if __name__ == '__main__':
    start()
