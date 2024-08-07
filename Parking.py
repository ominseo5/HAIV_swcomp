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
# model = YOLO('perfect_lanedetection.pt').to(device)
# model_traffic = YOLO('traffic_best4.pt')
model_car = YOLO('best_car.pt')
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
# cv2.namedWindow("control", flags=cv2.WINDOW_NORMAL)
# cv2.resizeWindow("control",(300,200))
# cv2.createTrackbar("throtel", "control",0, 245, env.nothing)
# cv2.createTrackbar("ignore", "control",78, 140, env.nothing)

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
get = 7000
stage = 0 # 0단계: 시작/ 1단계 첫번째 차 인식 (1300mm)/ 2단계 하드코딩 진행 /3단계 주차 완료/ 4단계 빠져나가기 완료
n = 5  # n초동안 조향 알고리즘을 따라가기 위한 변수


print("start!!!!!!!!!!!")
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


    # @ 가속 알고리즘: steering 값이 크면 감속(10만큼)
    # ws = cv2.getTrackbarPos("throtel", "control")
    speed = 65
    if keyboard.is_pressed('up'):
        ws = 1
        stage = 1
        serial_one('t201')
    elif keyboard.is_pressed('space'):
        ws = 0
    if ws and condition == 0:
        w = speed
        send2 = "w" + str(int(w))
        serial_one(send2)
    else:
        serial_one("x")

    # @ 후진주차 알고리즘
    car, carX = env.detect_car(frame2, model_car, device)
    print(f"car:{car}/ X: {carX}")
    if ws ==1 and len(carX)==1 and stage ==1:
        if min(carX) > 130:
            stage = 2
            time.sleep(0.5)
            stage = 1
            serial_one('x')
            time.sleep(2)
            serial_one('t200')
            serial_one('w95')
            time.sleep(3.75)
            serial_one('t219')
            time.sleep(4.5)
            serial_one('x')
            # 후진시작
            serial_one('t190')
            serial_one('s95')
            time.sleep(8.3)
            serial_one('t200')
            serial_one('s50')
            time.sleep(0.7)
            # serial_one('t200')
            # time.sleep(1.9)
            serial_one('x')
            time.sleep(5)
            serial_one('t200')
            serial_one('w95')
            time.sleep(2)
            serial_one('t190')
            serial_one('w75')
            time.sleep(7)
            serial_one('t219')
            serial_one('s95')
            time.sleep(4)
            serial_one('t190')
            serial_one('w75')
            time.sleep(4.8)
            serial_one('t200')
            serial_one('w150')
            time.sleep(8)
            serial_one('x')
    elif ws ==0:
        serial_one('x')

    # time.sleep(0.02)

    # 'q' 키를 누르면 종료
    if env.loop_break():
        print("hi")
        # serial_one("x")
        # break
# ser.close()
