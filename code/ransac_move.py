import numpy as np
import cv2, random, math, copy
import sys
import sympy as sy
import warnings
Width = 1280
Height =720

cap = cv2.VideoCapture("./xycar_track1.mp4")
window_title = 'camera'

warp_img_w = 320
warp_img_h = 240

warpx_margin = 20
warpy_margin = 3

nwindows = 11
margin = 12
minpix = 5

lane_bin_th = 111

warp_src  = np.array([  # 좌상단  좌하단    우상단    우하단
    [620, 393],  
    [335,  676],
    [740,393],
    [1145, 668]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(img):
    global nwindows
    global margin
    global minpix
    global lane_bin_th

    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    
    return lane

def getCurveFitting(src_img):
    
    height = src_img.shape[0]
    white_pixels = np.empty((0,2),dtype=np.int32)
    c_max = 0
    
    y,x = np.where(src_img > 0)
    white_pixels = np.array([[xi,height - yi] for yi, xi in zip(y,x)])
    
    for i in range(N):
        while True:
            try:
                n1,n2,n3 = np.random.randint(0,len(white_pixels -1),3)
            
            except ValueError:
                print("Can't detect lane")
            
            else:
                if (n1 != n2) and (n2 != n3) and (n3 != n1):
                    p1,p2,p3 = (
                        white_pixels[n1],
                        white_pixels[n2],
                        white_pixels[n3],
                    )
                    f_temp = getParabola(p1,p2,p3)
                    
                    if f_temp != -1:
                        break
        if c_max < amount:
            c_max = amount
            f_fitted = f_temp
    
    dst_img = drawParabola(src_img,f_fitted)
    return dst_img
def getParabola(p1,p2,p3):

    b1,b2,b3 =sy.symbols("b1,b2,b3")
    x = sy.symbols("x")
    
    x1,x2,x3 = p1[0],p2[0],p3[0]
    y1,y2,y3 = p1[1],p2[1],p3[1]
    with warnings.catch_warnigs(
        record = True
    ) as w:
        coefficient = sy.solve(
            [
                b1 -y1.
                b2 - ((y2-y1)/(x2-x1)),
                b3 -((1/(x3-x1))*(((y3-y2)/(x3-x2))-((y2-y1)/(x2-x1)))),
            ]
        )

    if len(w)>0:
        return -1
    else:
        result = sy.simplify(
            coefficient[b1]+ coefficient[b2] * (x-x1)+ coefficient[b3]  (x-x1) * (x-x2)
        )
        return result
def calcInlier(smaples,parabola):
    x = sy.symbols("x")
    amount = 0
    for i in range(smaples.shape[0]):
        xi, yi = smaples[i]
        fxi = parabola.subs(x,xi)
        ri = np.abs(yi-fxi)
        
        if ri < T:
            amount +=1
    
    return amount
def drawParabola(src_img,parabola):
    height,width = src_img.shape
    dst_img = cv2.cvtColor(src_img,cv2.COLOR_GRAY2BGR)
    x = sy.symbols("x")
    vertex = sy.nsolve(parabola,0)
    
    for xi in range(width):
        for yi in reversed(range(1,height)):
            if int(parabola.subs(x,xi))==yi:
                dst_img[height-yi,xi]==(0,0,255)
            elif yi == parabola.subs(x,vertex):
                break
    return dst_img

def start():
    global Width, Height, cap

    _, frame = cap.read()
    while not frame.size == (Width*Height*3):
        _, frame = cap.read()
        continue

    print("start")

    while cap.isOpened():
        
        _, frame = cap.read()
        image=frame
        warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        lane = warp_process_image(warp_img)
        lane_img =getCurveFitting(lane)

        cv2.imshow(window_title, lane_img)
        cv2.imshow("ride",warp_img)
        if cv2.waitKey(1)& 0xFF ==ord("x"):
            cv2.destroyAllWindows()
            break
        
if __name__ == '__main__':
    start()