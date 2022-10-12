ROS2 패키지
====

Navagation, SLAM 등 복잡한 응용 기술을 사용한 서비스 기능을 구현하기위해서
ROS2 패키지를 integration 하여 로봇 소프트웨어를 구현한다.

EXA 로봇에 적용 및 구현된 ROS 패키지
------

|패키지|설명|기타|
|---|---|---|
|navigation2| 장애물 회피 및 목적지 이동 기능 구현을 위한 패키지||
|realsense-ros|장애물 감지 및 카메라 활용을 위한 패키지||
|exa_robot|사용되는 ROS패키지의 parameter 및 Lauch 파일로 구성| 직접 구현 |
|BehaviorTree.CPP|navigation2 의존성 패키지||
|urg_node|Hokuyo Lidar 통신 제어 패키지||
|yldiar_ros2_driver|Ydlidar G6 Lidar 통신 제어 패키지||
|veocity_smoother|부드러운 주행 구현을 위한 패키지||
||||
