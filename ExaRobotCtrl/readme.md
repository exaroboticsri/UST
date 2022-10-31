# ExaRobotCtrl

This is the repository of the Control SW for the Exa Robot project

-실행

```
python3 ControlCore.py
```

## Include
  로봇 제어시 사용되는 노드 모음

#### AudioRecognition
  - STT 사용을 위한 파이썬 패키지
  - 사용 패키지 : speech_recognition
  - 사용 엔진 : recognize_google
  
  `
  sst = self._sttEngine.recognize_google(audio, language='ko-KR')
  `

#### Dialog
  - 디버거 프로그램상 Vision 출력 윈도우

#### DockWidgets
  - 디버거 프로그램 패키지 파일
  - QT
  
#### MobileRobot
  - 로봇 특성, stm32에서 받아온 패킷 파싱, 모터 제어
  - TMRDiff.py
    - 모터 속도 제어를 위한 패키지
    - IMU 사용 안함
    - 바퀴의 속도 차이만을 이용하여 각도 계산
  - TMRExaRobot.py
    - 시리얼 통신 파라미터 설정
    - 시리얼 수신 패킷 처리
    - 명령어 패킷 생성 및 송신
    - 기계적, 제어 특성 입력
```
- 기구적 특성
fWheelBase = 0.38  # according to FW
fWheelRadius = 0.08255  # wheel diameter is 6.5 inches
nEncoderRes = 4096  # PPR = 13
fGearRatio = 1
```
```
- 제어 특성
fMaxVel = 0.2#0.1 #for safe ..  #0.7
fMaxAcc = 0.1#0.4  # arbitrary, cannot find anywhere
fMaxJerk = 0.1#0.2  # arbitrary, cannot find anywhere
fMaxYaw = 1.9199#1.9199  # 180 deg/s (>110 deg/s : gyro performance will degrade)
```
```
 - 적용
super(CTMRExaRobot, self).__init__(fWheelRadius, fWheelBase, fMaxVel, fMaxAcc, fMaxJerk, fMaxYaw)
self.set_wheel_spec(nEncoderRes, fGearRatio)
```

#### MultiProcessing
  - 프로세싱 분할을 위한 패키지
  - 외부 연결, 로봇 이동, 카메라, 속도, AI 등 패키지에 따라 프로세스 분할
  - 각 프로세스간 파이프 및 큐로 통신
  
#### PathPlanning
  - 경로 생성 패키지
  - Astar, Dijkstra, plotting
  
#### ROSIntegration
  - ROS Launch 실행 패키지
  - 각 패키지 실행을 위한 런치 파일 및 해당 패키지에 해당하는 파라메터 변경
  
#### SpeechSynthesis
  - TTS 패키지
  - 사용 패키지 및 엔진 : pyttsx3 및 libespeak


## Library

#### Devices/Camera/CameraCommon.py
  - Vision Detection 실행 시 RealSense의 효율을 높이기 위해 작성한 패키지
  - RealSense SDK를 이용하여 카메라가 ROS와 상관없이 별도로 실행
  
#### SerialPort.py
  - 시리얼 연결 관리 ( 보드레이트, 포트, 바이트크기 등 )
