import cv2
import function_lib as fl
from ultralytics import YOLO
import time
import keyboard

# @For arduino
arduino_port = 'COM4' # port 변경 필수
ser = fl.libARDUINO()
comm = ser.init(arduino_port, 9600)
def serial_one(da): # @전송만 할때
    sent = da + "\n"
    comm.write(sent.encode())
def serial_comm(da): # @da=아두이노로 보내고싶은 str값/read=아두이노에서 받은값 # 타임아웃을 방지하기 위해 모든 데이터 전송은 돌아오는 반송 값이 있어야 함
    serial_one(da)
    read = comm.readline().rstrip()
    read = read.decode()
    return read

env = fl.libCAMERA()

# Roboflow에서 학습한 YOLOv8 모델 로드
device = 'cuda'
# device = 'cpu'
model = YOLO('perfect_lanedetection.pt').to(device)
model_traffic = YOLO('traffic_best4.pt')
# model_traffic = YOLO('perfect_trafficlight.pt')
# model_car = YOLO('best_car.pt')
# model_obstacle = YOLO('best_obs2.pt')

# @FOR video
# ch0 = env.video_setting("video\car1.mp4")
# frame_skip = 10  # 건너뛸 프레임 수
# frame_count = 0  # 프레임 카운터

# @For image
# ch0 = env.file_read("video\sample_f.jpg")
# frame1 = env.image_setting(ch0)

# @For WEBCAM(3 carmeras but ignore built in notebook webcam)
# _, ch1, ch2 = env.initial_setting(capnum=3)

# @For WEBCAM(2 carmeras)
ch0, ch1, _ = env.initial_setting(capnum=2)

# @Control bar
cv2.namedWindow("control", flags=cv2.WINDOW_NORMAL)
cv2.resizeWindow("control",(300,200))
cv2.createTrackbar("throtel", "control",0, 245, env.nothing)
cv2.createTrackbar("ignore", "control",68, 140, env.nothing)

# @ 변수 초기값 목록
dis = 150 # 초기값
adj_dis = 0
steering_value = 205
get = 7000
mod = 1 # 시작은 우측->좌측차선
condition = 0
speed = 190
last_edge = 520
last_steer = 205
ws = 0

n = 7.32   # n초동안 조향 알고리즘을 따라가기 위한 변수
mission = 0

while True:
    # @For Video
    # success, frame2 = ch0.read()
    # if not success:
    #     print("Video End")
    #     break
    # if frame_count % frame_skip != 0:
    #     frame_count += 1
    #     continue
    # frame_count += 1

    # @For WEBCAM(3 carmeras)
    # _, frame1, _, frame2 = env.camera_read3(ch1, ch2)

    # @For WEBCAM(2 carmeras)
    _, frame1, _, frame2 = env.camera_read(ch0, ch1)

    # @ 조향 알고리즘
    # Input: frame, 사용하는 model, 사용하는 device, follow-line이 우측차선에서 좌측으로 떨어진 정도(px)
    # Output: 우측차선 베이스의 x좌표(px), 조향각도(degree), 조향값(저항값), 우측 차선의 기울기(음수면 좌상향)
    # 마지막으로 인식한 우측차선 base 좌표와 새로운 좌표가 100px 이상 차이나면 무시하고, 바로 이전 steer 값 유지. # ignore가 1이면 무시했다는 뜻
    edge, _, steering_value, curv = env.detect_and_draw_midline(frame1, model, device, int(dis), last_edge)
    if mission == 1 or mission ==0:
        if abs(last_edge-edge) > 100:
            steering_value = last_steer
            edge = last_edge
        else:
            last_steer = steering_value
            last_edge = edge
    elif mission == 2:
        last_steer = steering_value
        last_edge = edge
    send = "t" + str(int(steering_value)) # 220 조향 불가 이슈
    serial_one(send)

    # @value_trans 함수
    # Input: 인풋 값, 인풋 값 최소/최대값, 아웃풋 값 최소/최대값, mod(1이면 선형변환, 0이면 로그변환)
    # Output: 아웃풋 값
    # 640- edge는 우측 끝과 우측 차선의 거리이므로 클수록 언더스티어->dis 커야함/ 작을수록 오버스티어 # 115, 131, 1
    min_dis = cv2.getTrackbarPos("ignore","control")
    dis = env.value_trans(640 - edge, 320, 640, min_dis, min_dis+16, 1)  # 이 값은 주행하면서 변경해야 함 105, 121, 1)
    adj_dis = env.value_trans(abs(curv),0.1,0.25,0,4,1)
    dis = dis - adj_dis

    # @ 가속 알고리즘: steering 값이 크면 감속(10만큼)
    # ws = cv2.getTrackbarPos("throtel", "control")
    if mission == 0:
        speed = 245
    elif mission ==2:
        speed = 160
    if keyboard.is_pressed('up'):
        ws = 1
        if mission == 0:
            start_time = time.time()  # 시작 시간
        mission = 1
    elif keyboard.is_pressed('space'):
        ws = 0
    if ws and condition == 0:
        w = env.value_trans(abs(steering_value-205), 0, 15, speed+10 , speed, 1)
        send2 = "w" + str(int(w))
        serial_one(send2)
    else:
        serial_one("x")

    # @ 장애물 회피 알고리즘
    current_time = time.time()
    if mission == 1:
        if (current_time - start_time) >= n:
            serial_one('t219')
            serial_one('w200')
            time.sleep(2)
            serial_one('t190')
            time.sleep(2.1)
            serial_one('t199')
            time.sleep(1.2)
            serial_one('t190')
            time.sleep(1.8)
            serial_one('t219')
            time.sleep(1.7)
            serial_one('x')
            start_time = current_time  # 시간 초기화
            mission = 2
            last_steer = 200
            last_edge = 520
            min_dis = 90

    # @ 신호등 알고리즘
    traffic, width = env.detect_traffic_light(frame1,model_traffic,device)
    print(f"label:{traffic}, width:{width}")
    if width > 240 and traffic == 'red' and condition == 0:
        # serial_one('t210')
        # time.sleep(0.4)
        serial_one('x')
        # serial_one('c')
        condition = 1 # 멈춘상태
        ws = 0
    elif traffic != 'red' and condition == 1:
        ws = 1
        condition = 0

    time.sleep(0.02)

    # 'q' 키를 누르면 종료
    if env.loop_break():
        print("hi")
