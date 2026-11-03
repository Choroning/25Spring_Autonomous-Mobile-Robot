# 강의 노트

[레포지토리 안내](../README.ko.md) | [English](README.md) | [한국어](README.ko.md)

> **최종 수정일:** 2026-11-04

| 단원 | 주제 | 문서 |
|:-----|:-----|:-----|
| [1주차 이동 로봇의 최근 동향과 ROS](Week01_Mobile-Robot-Trends-and-ROS/) | 이동 로봇의 다양한 메커니즘과 최근 동향, ROS의 개념과 특징, ROS 버전 선택과 설치, turtlesim 동작 테스트, 마스터와 노드 사이의 메시지 통신(XMLRPC, TCPROS) | [강의 노트](Week01_Mobile-Robot-Trends-and-ROS/Concepts_Lecture.ko.md) |
| [2주차 오픈 소스 기반 이동 로봇 핵심 기술](Week02_Open-Source-Mobile-Robot-Technologies/) | Autoware와 ROS, rosbag, Autoware의 기능과 사용자 인터페이스, Autoware 패키지 구조와 노드 개발 순서, 실차 운용에 필요한 노드 구성, 데이터 기록과 재생, 3D 지도와 벡터 지도 작성, NDT 매칭 기반 위치 인식 | [강의 노트](Week02_Open-Source-Mobile-Robot-Technologies/Concepts_Lecture.ko.md) |
| [3주차 Autoware의 위치 인식과 경로 계획](Week03_Autoware-Localization-and-Path-Planning/) | NDT 매핑과 NDT 매칭 위치 인식, 벡터 지도 기반 경로 생성, 경로 계획 노드(lane_rule, lane_stop, lane_select, velocity_set), pure pursuit 경로 추종, Ubuntu 18.04 설치(USB, 가상 머신) | [강의 노트](Week03_Autoware-Localization-and-Path-Planning/Concepts_Lecture.ko.md) |
| [4주차 리눅스, ROS, Autoware 설치](Week04_Linux-ROS-and-Autoware-Installation/) | Ubuntu 업데이트와 초기 설정, VS Code와 Eclipse 개발 환경, ROS Melodic 설치와 rosdep 초기화, turtlesim과 rqt_graph, ROS 네트워크 환경 변수, Qt 5.12.12와 Autoware.ai 1.14.0 설치, Velodyne 드라이버와 Kvaser CANlib 설치 | [강의 노트](Week04_Linux-ROS-and-Autoware-Installation/Concepts_Lecture.ko.md) |
| [5주차 ROS Autoware 노드 개발](Week05_ROS-Autoware-Node-Development/) | Qt와 Autoware 빌드, catkin 작업 공간과 패키지 생성, package.xml과 CMakeLists.txt, `hello_dsc` 퍼블리셔 노드의 작성과 빌드, 노드 실행 확인 | [강의 노트](Week05_ROS-Autoware-Node-Development/Concepts_Lecture.ko.md), [예제 코드](Week05_ROS-Autoware-Node-Development/Example_hello_dsc.cpp) |
| [6주차 ROS 노드 간 메시지 송수신](Week06_Message-Passing-Between-ROS-Nodes/) | 퍼블리셔와 서브스크라이버, `controller` 수신 노드와 콜백 함수, 여러 토픽의 송수신(`std_msgs::String`, `std_msgs::Int16`), 세 노드로 확장한 응용, rqt를 이용한 노드 그래프 확인 | [강의 노트](Week06_Message-Passing-Between-ROS-Nodes/Concepts_Lecture.ko.md), [예제 코드](Week06_Message-Passing-Between-ROS-Nodes/Example_controller.cpp) |
| [7주차 멀티스레드를 이용한 ROS 노드 메시지 송수신](Week07_Multithreaded-ROS-Node-Messaging/) | POSIX 스레드(pthread)의 생성과 분리, 스레드 함수와 스레드 간 정보 공유, 스레드마다 다른 주기의 토픽 발행, 여러 노드 사이에서 주기가 다른 메시지 흐름 | [강의 노트](Week07_Multithreaded-ROS-Node-Messaging/Concepts_Lecture.ko.md), [예제 코드](Week07_Multithreaded-ROS-Node-Messaging/Example_thread_prac.cpp) |
| [9주차 Autoware의 지도 작성과 위치 인식](Week09_Autoware-Mapping-and-Localization/) | Moriyama rosbag을 이용한 Autoware Quick Start, 포인트 클라우드 지도와 벡터 지도, NDT 매핑을 이용한 지도 작성, voxel_grid_filter 다운샘플링, NDT 매칭 위치 인식과 rqt의 노드 구성 | [강의 노트](Week09_Autoware-Mapping-and-Localization/Concepts_Lecture.ko.md) |
| [10주차 경로 계획](Week10_Path-Planning/) | 차량 경로 계획과 비홀로노믹 제약, 전역 경로 계획과 지역 경로 계획, A\* 알고리즘과 비용 함수, 대각선 이동을 허용한 A\*, Hybrid A\* 알고리즘의 비용 계산과 경로 탐색 | [강의 노트](Week10_Path-Planning/Concepts_Lecture.ko.md) |
| [11주차 자율주행차의 횡방향 제어 I](Week11_Lateral-Control-of-Autonomous-Vehicles-I/) | Hybrid A\* 경로 평활화와 속도 프로파일 생성, 장애물 회피를 위한 swerve와 nudge, 횡방향 제어기의 상태 정의, 타이어 각도 관계와 힘 관계, 차량 파라미터 추정 | [강의 노트](Week11_Lateral-Control-of-Autonomous-Vehicles-I/Concepts_Lecture.ko.md) |
| [12주차 자율주행차의 횡방향 제어 II](Week12_Lateral-Control-of-Autonomous-Vehicles-II/) | 횡방향 차량 운동의 기구학(자전거 모델), 차량 폭을 고려한 기구학, 히스테리시스에 따른 기구학 모델과 동역학 모델의 전환, 시스템 동역학과 상태 공간 모델, 제어기와 상태 관측기 설계, wf_simulator를 이용한 Autoware 시뮬레이션 | [강의 노트](Week12_Lateral-Control-of-Autonomous-Vehicles-II/Concepts_Lecture.ko.md) |
| [13주차 자율주행 센서와 CAN 버스](Week13_Autonomous-Driving-Sensors-and-CAN-Bus/) | GPS, DGPS, RTK와 위치 오차 요인, LiDAR와 레이더의 비교, CAN을 쓰는 이유, CAN 개요와 데이터 프레임 구조, CAN의 한계 | [강의 노트](Week13_Autonomous-Driving-Sensors-and-CAN-Bus/Concepts_Lecture.ko.md) |
| [14주차 Autoware CAN 노드](Week14_Autoware-CAN-Node/) | Kvaser linuxcan 드라이버와 CANlib 설치, ROS CAN 노드 개발, Autoware ROS 노드의 작성 순서와 구조, colcon 빌드와 Runtime Manager, `can_mobility_test` 노드 작성 | [강의 노트](Week14_Autoware-CAN-Node/Concepts_Lecture.ko.md) |
