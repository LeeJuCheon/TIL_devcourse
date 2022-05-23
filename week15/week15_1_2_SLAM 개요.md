# Week 15 Day1, SLAM 개요(Localization, Mapping, SLAM)

1. SLAM
* Simultaneous Localization and Mapping(동시적 위치추정 및 지도작성)
* 센서를 이용(Camera, Depth camera, LIDAR,GNSS,Rader. Ultrasound, IMU)

* 사전정보가 없어도 정확한 위치를 추정할 수 있는 기술
* 확률적인 프로세스

1. 로보틱스 기술의 진화
* Mobile robotics
  * 사람이 가지 못하는 곳을 대신 탐색하는 기술
  * '이동' 자체를 자동화
  * 고정된 로봇의 workspace 확장
  * 자율 이동체는 인지, 결정, 행동을 할 수 있어야 함

* Mobility의 인지
  * 이동 가능한 공간인지, 이동이 불가능한 공간인지 인지
  * 자기 자신의 움직임을 인지(IMU, GPS 등)

2. Localization, Mapping, SLAM
* Proprioceptive sensor(나 자신의 위치를 인지) + Exteroceptive sensor(외부의 위치를 인지)를 조합

* 두 센서 모두 확률분포를 가지고 있음

* Proprioceptive sensor가 부정확할 때는 Localization의 값으로 보정
* Exteroceptive sensor가 부정확할 때는 Mapping의 값으로 보정

* Mapping : 나 자신의 위치를 정확히 알고 있을 때 지도를 생성할 수 있음
* Localization : 외부 환경을 정확히 알고 있을 때 내 위치를 변경할 수 있음



### Question
#### Localization과 SLAM의 다른점은 무엇인가?