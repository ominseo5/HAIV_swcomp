import sys
import cv2  # pip install opencv
import time
import serial  # pip install serial
import numpy as np  # pip install numpy
import matplotlib.pyplot as plt  # pip install matplotlib
from rplidar import RPLidar  # pip install rplidar-roboticia
import math
from numpy.linalg import norm
# import torch
from ultralytics import YOLO

np.set_printoptions(threshold=sys.maxsize, linewidth=150)

"""------------------Arduino Variable------------------"""
WAIT_TIME = 2
"""----------------------------------------------------"""

"""-------------------LIDAR Variable-------------------"""
SCAN_TYPE = "normal"
SAMPLE_RATE = 10
MAX_BUFFER_SIZE = 3000
MIN_DISTANCE = 0
"""----------------------------------------------------"""

"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
FORWARD_THRESHOLD = 0.3
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
FORWARD, LEFT, RIGHT = (0, 1, 2)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
HUE_THRESHOLD = ([4, 176], [40, 80], [110, 130], [20, 40])
b_value1 = 50  # 밝기 감소 값
s_value1 = 100 # 채도 감소 값
alpha1 = 1.0# 대비 증가 값
"""-----------------------------------------------------"""

# noinspection PyMethodMayBeStatic
class libARDUINO(object):
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.wait_time = WAIT_TIME  # second unit

    # Arduino Serial USB Port Setting
    def init(self, port, baudrate):
        ser = serial.Serial()
        ser.port, self.port = port, port
        ser.baudrate, self.baudrate = baudrate, baudrate
        ser.open()
        time.sleep(self.wait_time)
        return ser
    def close(self):
        ser = serial.Serial()
        ser.close()


######################################################

class libLIDAR(object):
    def __init__(self, port):
        self.rpm = 0
        self.lidar = RPLidar(port)
        self.scan = []

    def init(self):
        info = self.lidar.get_info()
        print(info)

    def getState(self):
        health = self.lidar.get_health()
        print(health)

    def scanning(self):
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan_list) > SAMPLE_RATE:
                    np_data = np.array(list(scan_list))
                    yield np_data[:, 1:]
                scan_list = []
            if distance > MIN_DISTANCE:
                scan_list.append((quality, angle, distance))

    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def setRPM(self, rpm):
        self.lidar.motor_speed = rpm

    def getRPM(self):
        return self.lidar.motor_speed

    def getAngleRange(self, scan, minAngle, maxAngle):
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle))
        return data[condition]

    def getDistanceRange(self, scan, minDist, maxDist):
        data = np.array(scan)
        condition = np.where((data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

    def getAngleDistanceRange(self, scan, minAngle, maxAngle, minDist, maxDist):
        data = np.array(scan)
        condition = np.where(
            (data[:, 0] < maxAngle) & (data[:, 0] > minAngle) & (data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

########################################
# noinspection PyMethodMayBeStatic
class libCAMERA(object):
    def __init__(self):
        self.capnum = 0
        self.row, self.col, self.dim = (0, 0, 0)

    def loop_break(self):
        n = 1
        if cv2.waitKey(n) & 0xFF == ord('q'):
            print("Camera Reading is ended.")
            return True
        elif cv2.waitKey(n) & 0xFF == 27:
            print("Camera Reading is ended.")
            return True
        else:
            return False

    def file_read(self, img_path):
        return np.array(cv2.imread(img_path))

    def rgb_conversion(self, img):
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)

    def hsv_conversion(self, img):
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    def gray_conversion(self, img):
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)

    def color_extract(self, img, idx):
        result = img.copy()

        for i in range(RED + GREEN + BLUE):
            if i != idx:
                result[:, :, i] = np.zeros([self.row, self.col])

        return result


    def image_setting(self, image):
        image = image.astype(np.uint8)
        return image

    def video_setting(self, videopath):
        channel0 = None
        channel0 = cv2.VideoCapture(videopath)
        return channel0

    def initial_setting2(self, cam0port=0, cam1port=1, capnum=1, width=1920, height=1080):
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        channel2 = None
        self.capnum = capnum

        def set_resolution(channel, width, height):
            if channel is not None and channel.isOpened():
                channel.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                channel.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                return True
            return False

        if capnum == 1:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if set_resolution(channel0, width, height):
                print("Camera Channel0 is enabled with resolution 1920x1080!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if set_resolution(channel0, width, height):
                print("Camera Channel0 is enabled with resolution 1920x1080!")
            channel1 = cv2.VideoCapture(cv2.CAP_DSHOW + cam1port)
            if set_resolution(channel1, width, height):
                print("Camera Channel1 is enabled with resolution 1920x1080!")
        elif capnum == 3:
            channel1 = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
            if set_resolution(channel1, width, height):
                print("Camera Channel2 is enabled with resolution 1920x1080!")
            channel2 = cv2.VideoCapture(2 + cv2.CAP_DSHOW)
            if set_resolution(channel2, width, height):
                print("Camera Channel3 is enabled with resolution 1920x1080!")
        return channel0, channel1, channel2

    def initial_setting(self, cam0port=0, cam1port=1, capnum=1):
        # OpenCV Initial Setting
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        channel2 = None
        self.capnum = capnum

        if capnum == 1:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")

            channel1 = cv2.VideoCapture(cv2.CAP_DSHOW + cam1port)
            if channel1.isOpened():
                print("Camera Channel1 is enabled!")
        elif capnum == 3:
            channel1 = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
            if channel1.isOpened():
                print("Camera Channel2 is enabled!")
            channel2 = cv2.VideoCapture(2 + cv2.CAP_DSHOW)
            if channel1.isOpened():
                print("Camera Channel3 is enabled!")

        return channel0, channel1, channel2

    def camera_read(self, cap1, cap2=None, cap3=None):
        result, capset = [], [cap1, cap2, cap3]

        for idx in range(0, self.capnum):
            ret, frame = capset[idx].read()
            result.extend([ret, frame])

        return result

    def camera_read3(self, cap1, cap2):
        result, capset = [], [cap1, cap2]

        for idx in range(0, self.capnum-1):
            ret, frame = capset[idx].read()
            result.extend([ret, frame])

        return result

    def image_show(self, frame0, name0, frame1=None, name1=None):
        if frame1 is None:
            #cv2.imshow('frame0', frame0)
            cv2.imshow(name0, frame0)

        else:
            # cv2.imshow('frame0', frame0)
            # cv2.imshow('frame1', frame1)
            cv2.imshow(name0, frame0)
            cv2.imshow(name1, frame1)

    def color_filtering(self, img, roi=None, print_enable=False):
        self.row, self.col, self.dim = img.shape

        hsv_img = self.hsv_conversion(img)
        h, s, v = cv2.split(hsv_img)

        s_cond = s > SATURATION
        if roi is RED:
            h_cond = (h < HUE_THRESHOLD[roi][0]) | (h > HUE_THRESHOLD[roi][1])
        else:
            h_cond = (h > HUE_THRESHOLD[roi][0]) & (h < HUE_THRESHOLD[roi][1])

        v[~h_cond], v[~s_cond] = 0, 0
        hsv_image = cv2.merge([h, s, v])
        result = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        if print_enable:
            self.image_show(result, 'color')

        return result

    def gaussian_blurring(self, img, kernel_size=(None, None)):
        return cv2.GaussianBlur(img.copy(), kernel_size, 0)

    def canny_edge(self, img, lth, hth):
        return cv2.Canny(img.copy(), lth, hth)

    def histogram_equalization(self, gray):
        return cv2.equalizeHist(gray)

    def hough_transform(self, img, rho=None, theta=None, threshold=None, mll=None, mlg=None, mode="lineP"):
        if mode == "line":
            return cv2.HoughLines(img.copy(), rho, theta, threshold)
        elif mode == "lineP":
            return cv2.HoughLinesP(img.copy(), rho, theta, threshold, lines=np.array([]),
                                   minLineLength=mll, maxLineGap=mlg)
        elif mode == "circle":
            return cv2.HoughCircles(img.copy(), cv2.HOUGH_GRADIENT, dp=1, minDist=80,
                                    param1=200, param2=10, minRadius=40, maxRadius=100)

    def morphology(self, img, kernel_size=(None, None), mode="opening"):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

        if mode == "opening":
            dst = cv2.erode(img.copy(), kernel)
            return cv2.dilate(dst, kernel)
        elif mode == "closing":
            dst = cv2.dilate(img.copy(), kernel)
            return cv2.erode(dst, kernel)
        elif mode == "gradient":
            return cv2.morphologyEx(img.copy(), cv2.MORPH_GRADIENT, kernel)

    def point_analyze(self, gray, line, point_gap, len_threshold):
        disparity = [0, 0]

        for idx in range(2):
            yplus = line[idx + 1] + point_gap if line[idx + 1] + point_gap < self.row else self.row - 1
            yminus = line[idx + 1] - point_gap if line[idx + 1] - point_gap >= 0 else 0

            if yplus < 0 or yminus >= self.row:
                break
            elif yplus >= self.row or yminus < 0:
                break

            disparity[idx] = np.abs(gray[yplus][line[idx]] - gray[yminus][line[idx]])

        if np.average(disparity) > len_threshold:
            return True
        else:
            return False

    def brightness(self, img):
        if len(img.shape) == 3:
            # Colored RGB or BGR (*Do Not* use HSV images with this function)
            # create brightness with euclidean norm
            return np.average(norm(img, axis=2)) / np.sqrt(3)
        else:
            # Grayscale
            return np.average(img)

    # 밝기 낮추기
    def decrease_brightness(self, image, b_v, s_v):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.add(v, -b_v)  # 밝기 감소
        s = cv2.add(s, s_v)  # 채도 감소
        #v[v < 0] = 0  # 값의 범위를 유지
        v = np.clip(v, 0, 255)
        s = np.clip(s, 0, 255)
        final_hsv = cv2.merge((h, s, v))
        image_brightness_decreased = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return image_brightness_decreased

    # 대비 올리기
    def increase_contrast(self, image, alpha):
        new_image = np.clip(alpha * image + 0, 0, 255).astype(np.uint8)
        return new_image

    def object_detection(self, img,  b_value, s_value, alpha, sample=0, mode="circle", print_enable=False):
        result = None
        img = cv2.resize(img, (320,320))
        replica = img.copy()
        replica = self.decrease_brightness(replica, b_value, s_value)
        replica = self.increase_contrast(replica, alpha)
        for color in (RED, YELLOW, GREEN):
            extract = self.color_filtering(img, roi=color, print_enable=True)
            gray = self.gray_conversion(extract)
            circles = self.hough_transform(gray, mode=mode)
            if circles is not None:
                for circle in circles[0]:
                    center, count = (int(circle[0]), int(circle[1])), 0

                    hsv_img = self.hsv_conversion(img)
                    h, s, v = cv2.split(hsv_img)

                    # Searching the surrounding pixels
                    for res in range(sample):
                        x, y = int(center[1] - sample / 2), int(center[0] - sample / 2)
                        s_cond = s[x][y] > SATURATION
                        if color is RED:
                            h_cond = (h[x][y] < HUE_THRESHOLD[color][0]) | (h[x][y] > HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count
                        else:
                            h_cond = (h[x][y] > HUE_THRESHOLD[color][0]) & (h[x][y] < HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count

                    if count > sample / 2:
                        result = COLOR[color]
                        cv2.circle(replica, center, int(circle[2]), (0, 0, 255), 2)

        if print_enable:
            # if result is not None:
            #     print("Traffic Light: ", result)
            cv2.imshow('color',replica)

        return result


    def nothing(self, x):
        pass

    def linear_tr(self, n, mini, maxi, mod):
        m = max(190, min(220, n))
        o = max(80, min(138, n))
        lvmax=170
        lvmin=80
        if mod == 1: # for throtle
            th = mini + math.log((15 - abs(205 - m)) / 15 * (math.exp(1)-1)+1) * (maxi - mini)
            th = int(th)
        elif mod == 0: # for base
            th = mini + (15 - abs(205 - m)) / 15 * (maxi - mini)
        elif mod == 2: # for L-V
            th = mini + (o-lvmin) / (lvmax-lvmin) * (maxi - mini)
        elif mod == 3: # for L-V logarimatric
            th = mini +math.log((o-lvmin) / (lvmax-lvmin) * (math.exp(1)-1)+1) * (maxi - mini)
        return th


    def angle_to_steering_value(self, m, angle):
        # Normalize the angle to a steering value between 190 and 220
        min_angle = -m  # Minimum angle for left turn
        max_angle = m  # Maximum angle for right turn
        min_steering = 190
        max_steering = 220
        # Ensure the angle is within the expected range
        angle = max(min_angle, min(max_angle, angle))
        steering_value = int(
            max_steering + (angle - min_angle) * (min_steering - max_steering) / (max_angle - min_angle)) # 내사법
        return steering_value

    # 실선과 점선 구분 함수
    def classify_line(self, zero_count, total_windows):
        if zero_count > total_windows / 4:
            return 'dashed'
        return 'solid'

    def value_trans(self, origin, mini, maxi, mino, maxo, mod):
        origin = max(mini, min(maxi, origin))
        if mod:
            trans_origin = mino + (maxo-mino)*(origin-mini)/(maxi-mini)
        else:
            trans_origin = mino + (maxo - mino) * math.log((origin - mini) / (maxi - mini) * (math.exp(1) - 1) + 1)
        return trans_origin

    # 경로에 따른 차량 제어 알고리즘(pure pursuit)
    def pure_pursuit_control(self, current_pos, path, wheelbase, pos):
        target_point = None
        min_distance = float('inf')
        '''
        for point in path:
            if point[0]>-1 and point[0]<641 and point[1]>-1 and point[1]<641:
                target_point = point 
                break
        if target_point is None:
            target_point = path[-1] # 만약 위에서 결정 안되면 그냥 path의 맨아래 지점을 타겟으로
        '''
        target_point = path[pos]
        dx = target_point[0] - current_pos[0]
        dy = -target_point[1] + current_pos[1]

        angle_to_target = np.arctan2(dx, dy)
        ld = np.hypot(dx, dy)
        if angle_to_target:
            R = ld /(2*np.sin(angle_to_target))
            steering_angle = np.arctan2(wheelbase/R, 1)
            steering_angle = np.degrees(steering_angle)
        else:
            steering_angle = 0
        return target_point, steering_angle, angle_to_target

    def detect_and_draw_midline(self, frame, model, device, dis, last_edge):
        frame = cv2.resize(frame, (640, 640))

        # 버드아이뷰 변환을 위한 포인트 선택 (더 넓은 영역으로 설정)
        tl, bl, tr, br = (82, 382), (0, 480), (558, 382), (640, 480)
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, 640], [640, 0], [640, 640]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

        # 버드아이뷰 변환 적용
        transformed_frame = cv2.warpPerspective(frame, matrix, (640, 640))
        points = [tl, bl, tr, br]
        for point in points:
            frame = cv2.circle(frame, point, 10, (0, 0 ,255), -1)

        # YOLOv8 모델로 프레임을 예측
        results = model(transformed_frame, device=device, conf=0.25, iou=0.45)
        steering_angle = 0
        adj_angle = 0
        curv = 0
        steering_value = 205

        # 결과에서 마스크 가져오기
        mask = np.zeros((640, 640), dtype=np.uint8)
        if results[0].masks is not None:
            for seg in results[0].masks.data:
                seg_resized = cv2.resize(seg.cpu().numpy(), (640, 640))
                mask[seg_resized > 0.5] = 255

        # 검출된 부분은 하얀색, 나머지는 검정색
        output_frame = np.zeros_like(transformed_frame)
        output_frame[mask == 255] = [255, 255, 255]
        # alternative present
        # output_frame = transformed_frame

        # 차선 검출 및 스티어링 각도 계산
        histogram_R = np.sum(mask[320:, :], axis=0) # y는 320부터 아래까지
        midpoint = len(histogram_R) // 2
        rightx_base = np.argmax(histogram_R[midpoint:]) + midpoint
        histogram_L = np.sum(mask[320:, :], axis=0) # y는 320부터 아래까지
        leftx_base = np.argmax(histogram_L[midpoint:])
        left_edge = leftx_base
        right_edge = rightx_base
        print(f"right edge: {640-right_edge}",end="/ ")
        nwindows, window_height = 20, 640 // 20
        nonzero = mask.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
        rightx_current = rightx_base
        leftx_current = leftx_base
        margin, minpix = 100, 50
        right_lane_inds = []
        left_lane_inds = []

        for window in range(nwindows):
            win_y_low, win_y_high = 640 - (window + 1) * window_height, 640 - window * window_height
            win_xleft_low, win_xleft_high = leftx_current - margin, leftx_current + margin
            win_xright_low, win_xright_high = rightx_current - margin, rightx_current + margin
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

            right_lane_inds.append(good_right_inds)
            left_lane_inds.append(good_left_inds)

            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))

        right_lane_inds = np.concatenate(right_lane_inds)
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
        left_lane_inds = np.concatenate(left_lane_inds)
        leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]

        ploty = np.linspace(0, 639, num=640)
        # if leftx.size > 0:
        #     left_fit = np.polyfit(lefty, leftx, 2)
        #     left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        #     # 차선 그리기
        #     pathL = [(int(left_fitx[i]), int(ploty[i])) for i in range(len(ploty) - 1)]
        #     for i in range((len(ploty) - 1)):
        #         cv2.circle(output_frame, (int(left_fitx[i]), int(ploty[i])), 5, (255, 0, 0), -1)

        if rightx.size > 0:
            right_fit = np.polyfit(righty, rightx, 2)
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            # 차선 그리기
            path = [(int(right_fitx[i]), int(ploty[i])) for i in range(len(ploty) - 1)]
            shift_x = -dis
            # cloned_path: 따라갈 곡선
            cloned_path = [(int(right_fitx[i] + shift_x), int(ploty[i])) for i in range(len(ploty) - 1)]
            for i in range((len(ploty) - 1)):
                cv2.circle(output_frame, (int(right_fitx[i]), int(ploty[i])), 5, (0, 0, 255), -1)
                cv2.circle(output_frame, (int(right_fitx[i] + shift_x), int(ploty[i])), 5, (0, 255, 0), -1)

            cv2.line(output_frame,(last_edge-100,540),(last_edge+100,540) ,color=[255, 0, 255], thickness=4)
            # Pure Pursuit 알고리즘을 이용한 조향 각도 계산
            wheelbase = 167  # 차량의 휠베이스 (단위: px) 0.612m /167px
            current_pos = (midpoint, mask.shape[0] + wheelbase)
            #  화면 최하단 중심이 전륜축 중심이라고 가정할때 차량 후륜 중앙의 위치, shape[0]는 프레임 높이, 640

            # 우측 차선의 기울기 계산 -> 기울기 바탕으로 Lookahead distance 결정(target point가 되는 path의 index 결정)
            _, _, curv = self.pure_pursuit_control(path[-1], path, wheelbase, 400) # 400 이값도 매우 중요함/이전에 320으로 했는데 실패, 또 실패하면 640까지 키워볼것
            print(f"curv:{curv:.3f}", end=" ")
            pos = int(self.value_trans(abs(curv), 0, 0.5, 490, 400, 0)) #490~400 index (클수록 아래쪽)
            target, steering_angle, _ = self.pure_pursuit_control(current_pos, cloned_path, wheelbase, pos)

            adj_angle = steering_angle
            angle_text = f"ST Angle: {adj_angle:.2f} degrees / dis: {dis}"
            cv2.circle(output_frame, target, 5, (0, 0, 255), -1)
            steering_value = int(self.value_trans(adj_angle, -28, 28, 220, 190, 1))  # 전동차 자체의 최대 조향 각도 28도
            steering_value_text = f"ST Value: {steering_value} / curv: {curv:.3f}"

            # 각도 텍스트 표시
            cv2.putText(output_frame, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(output_frame, steering_value_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # 원본 프레임 출력
        cv2.imshow('Original', frame)

        # 출력 프레임 출력
        cv2.imshow('output', output_frame)

        # BEV 출력
        # cv2.imshow('BEV', transformed_frame)

        return right_edge, adj_angle, steering_value, curv

    def detect_traffic_light(self, frame, model, device):
        class_names = ['green', 'red', 'yellow']
        colors = [(0,255,0),(0,0,255),(0,255,255)]
        frame = cv2.resize(frame, (640, 640))
        width = 0
        # 예측 수행
        # results = model.predict(source=frame,  verbose=False, conf=0.50)
        results = model.predict(source=frame, device=device, conf=0.5, iou=0.45)

        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            print("No detection results found")
            cv2.imshow('traffic', frame)
            return "none", width
        if len(results[0].boxes) > 1:
            print("Too many detection")
            cv2.imshow('traffic', frame)
            return "none", width

        for result in results:
            for box in result.boxes:
                # print("Box coordinates and class:", box.xyxy[0], box.cls)
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                class_id = int(box.cls)
                label = class_names[class_id]
                color = colors[class_id]
                # 박스를 그리고 라벨을 표시
                width = xmax-xmin
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                cv2.putText(frame, f"Sign: {label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.imshow('traffic',frame)
        return label, width

    def detect_car(self, frame, model, device):
        class_names = ['BlueCar','GreenCar', 'RedCar', 'YellowCar']
        colors = [(255, 255, 102), (0, 255, 0), (0, 0, 255), (0, 255, 255)]
        frame = cv2.resize(frame, (640, 640))
        labels = []
        x = []
        # 예측 수행
        results = model.predict(source=frame, device=device, conf=0.6, iou=0.45)

        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            print("No detection results found")
            cv2.imshow('car', frame)
            return "none", x

        for result in results:
            for box in result.boxes:
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                class_id = int(box.cls)
                label = class_names[class_id]
                labels.append(label)
                color = colors[class_id]
                center = (xmin + xmax)/2
                x.append(center)
                # 박스를 그리고 라벨을 표시
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.imshow('car', frame)
        return labels, x

    def detect_obstacle(self, frame, model, device):
        class_names = ['BlueCar','GreenCar','Obstacle','RedCar', 'YellowCar']
        colors = [(255, 255, 102), (0, 255, 0),(255, 255, 255), (0, 0, 255), (0, 255, 255)]
        frame = cv2.resize(frame, (640, 640))
        labels = []
        width = []
        x = []
        # 예측 수행
        results = model.predict(source=frame, device=device, conf=0.5, iou=0.45)

        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            print("No detection results found")
            cv2.imshow('obstacle', frame)
            return 0, 0

        for result in results:
            for box in result.boxes:
                xmin, ymin, xmax, ymax = map(int, box.xyxy[0])
                class_id = int(box.cls)
                label = class_names[class_id]
                if (xmax-xmin)<500:
                    labels.append(label)
                    color = colors[class_id]
                    width.append(xmax-xmin)
                    center = (xmin + xmax) / 2
                    x.append(center)
                    # 박스를 그리고 라벨을 표시
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        if width == []:
            print("No detection results found")
            cv2.imshow('obstacle', frame)
            return 0, 0
        maxwidth = max(width)
        ind = width.index(maxwidth)
        maxX = x[ind]
        cv2.putText(frame, f"x: {maxX}/Width: {maxwidth}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow('obstacle', frame)
        return maxX, maxwidth
