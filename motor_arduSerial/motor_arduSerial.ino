#include <Car_Library.h>

int val;            // 수신된 값 저장할 변수
String sig;

int motorA1 = 3;    // 모터 드라이버 IN1 - for left
int motorA2 = 4;    // 모터 드라이버 IN2
int motorB1 = 5;    // 모터 드라이버 IN1 - for right
int motorB2 = 6;    // 모터 드라이버 IN2
int motorC1 = 7;    // 모터 드라이버 IN1 - for direction
int motorC2 = 8;    // 모터 드라이버 IN2
int analogPin = A5; // 가변저항 Output Pin
int tilt_l=220;     //좌측 최대회전 저항값
int tilt_r=190;     //우측 최대회전 저항값
int tilt_c=203; //중앙
int dz=1;           //데드존
//블투연결핀 RX19 TX18 (보드기준)


void stop(){ //모터정지
  motor_hold(motorA1, motorA2);
  motor_hold(motorB1, motorB2);
  motor_hold(motorC1, motorC2);
}

void forward(int n,int m){//속도n 지속시간m
  if(n>255){n=255;}
  else if(n<0){n=0;}
  motor_forward(motorA1, motorA2, n);
  motor_forward(motorB1, motorB2, n);
  // delay(m);
  // stop();
}

void backward(int n,int m){
  if(n>255){n=255;}
  else if(n<0){n=0;}
  motor_backward(motorA1, motorA2, n);
  motor_backward(motorB1, motorB2, n);
  // delay(m);
  // stop();
}


void tilt(int n){//n=목표 저항값, forward가 좌
  if(n>tilt_l){n=tilt_l;}
  else if(n<tilt_r){n=tilt_r;}
  while(true){
    val = potentiometer_Read(analogPin);
    if(n-dz<=val && n+dz>=val){//목표값
      motor_hold(motorC1, motorC2);
      break;
    }else if(n-dz>val){//좌회전 필요
      motor_forward(motorC1, motorC2, 170);
      delay(60);
    }else if(n+dz<val){//우회전 필요
      motor_backward(motorC1, motorC2, 170);
      delay(60);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  //pinMode(19, INPUT_PULLUP); // fix Serial1
  
  Serial.begin(9600);               // 시리얼 통신 시작, 통신 속도 설정
  //Serial1.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);     // LED 핀 모드 설정
  pinMode(analogPin, INPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorC1, OUTPUT);
  pinMode(motorC2, OUTPUT);
  tilt(tilt_c); //중앙  
  delay(10);
}

void loop() {
  //적외선받아오는코드위치
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    //Serial.println("get");
    //가변저항
    //val = potentiometer_Read(analogPin);
    //Serial.println(val);
    //시리얼통신
    sig = Serial.readStringUntil('\n');
    char command = sig.charAt(0);
    int data = sig.substring(1,4).toInt();
    //직접 시리얼인풋
    //char command = Serial.read();

    switch (command) {
      case 'w':  //forward
        forward(data,1500);
        Serial.println("forward");
        break;
      case 's':  //back
        backward(data,1500);
        Serial.println("backward");
        break;
      case 'a':  //left
        Serial.println("left");
        tilt(tilt_l);
        break;
      case 'd':  //right
        Serial.println("right");
        tilt(tilt_r);
        break;
      case 'x':  //stop
        stop();
        Serial.println("stop");
        break;
      case 'c':  //center
        Serial.println("center");
        tilt(tilt_c);
        break;    
      case 't':  //tilted
        Serial.print("tilt");
        Serial.println(data);
        tilt(data);
        break;   
      default: //default
        Serial.print("default");
        break;
    }
    // Serial.print(command);
    // Serial.print(",");
    // Serial.println(data);
  }
}
