# Overview

- Include

|       Module        |    Description     |  Etc  |
| :-----------------: | :----------------: | :---: |
|       Commons       |   기본적으로 사용되는 공통 모듈            |       |
|     ControlCore     |   메인 컨트롤러로서 장치, 인터페이스 및 제어 모듈           |
|     MainWindow      |   사용자 인터페이스(GUI)의 신호 및 슬롯 모듈   |       |
|      UiCommon       |   GUI 공통 모듈                |       |
|  AudioRecognition   |   음성 인식 모듈               |       |
|       Dialog        |   카메라 뷰어 및 얼굴 인식 모듈             |       |
|     DockWidgets     |   기능별 GUI를 구성하는 독 위젯 모듈      |       |
|    ExternalComm     |   소켓 사용 외부 통신모듈 (TCP, UDP..)                 |       |
|    MdiBackground    |   GUI 배경 구성 모듈          |       |
|        Misc         |   상태 표시를 위한 GUI 모듈  |   |
|     MobileRobot     |   로봇 제어 를 위한 인터페이스 모듈           |       |
|   MultiProcessing   |   멀티 프로세서 구현을 Base 및 기능별 구현 프로세스 모듈   |       |
|    PathPlanning     |   경로 계획 모듈   |  AStar, Dijkstra 구현      |
|   ROSIntegration    |   ROS2를 인터페이스 를 위한 모듈   |       |
|    TextToSpeech     |   음성 안내를 위한 모듈                |       |
|      SubWindow      |   서브 윈도우 GUI 모듈   | TTS, Serial Term, Trajectort Gen 등.     |
|     SerialTerm      |   Serial Term 기능 및 서브 윈도우를 구성하는 위젯 GUI 모듈               |       |
| TrajectoryGenerator |    TrajectoryGenerator 기능 및 서브 윈도우를 구성하는 위젯 GUI 모듈                 |       |

- Library

|       Module        |    Description     |  Etc  |
| :-----------------: | :----------------: | :---: |
|       Camera        |   카메라 장치 인터페이스를 위한 모듈      |      |
|       Serial        |  Serial 인터페이스를 위한 모듈|    |
|       Socket        |  외부 통신을 위한 소켓 모듈          |       |