# Week 15 Day 1, SLAM의 종류

1. SLAM에 사용할 수 있는 센서
* Proprioceptive sensor
  1) Wheel encoder
    * 바퀴의 회전량(RPM)과 이동량(=바퀴의 회전량 * 바퀴의 둘레)을 측정하는 센서
    * 장점
      * 자동차에는 기본적으로 탑재
    * 단점
      * Odometry를 할 시 drift에 약함
      * 바퀴가 헛도는 경우 잘못된 센서의 값이 생길 수 있음
      * 바퀴의 둘레가 주행 중 자주 바뀜(탑승자의 무게, 코너링, 바람빠짐, 마찰열로 인한 타이어 팽창)
      * 에러가 누적됨
    * 환경을 catch하여 반영하는 알고리즘이 존재하기도 함
  2) IMU
    * 관성을 측정하는 센서
    * 선형가속도 측정센서와 각속도 측정 센서가 혼합된 센서
    * Spring-damper system의 원리를 이용
      * Optical system -> 빛을 이용한 시스템, 더운날에도 추운날에도 가능

    * 장점
      * Consumer grade 제품은 저렴함(자동차는 안전규정 때문에 가격이 저렴하지 않음)
      * 높은 sensitivity
      * 높은 FPS
    * 단점
      * 엄청나게 빠른 drift 누적
      * 기준값이 시간이 지남에 따라 바뀜
      * GNSS / LIDAR / Camera 등의 센서로 빠르게 보정해줘야함
* Exteroceptive sensor
  * GNSS(Global navigation satellite system)
    * GPS라고도 불림(GPS는 미국의 GNSS를 지칭함)
    * 비콘 기반의 위치추정 센서
    * 나라마다 시스템이 다름
    * 장점
      * 싸고 사용하기 쉬움
    * 단점
      * 일정 거리의 오차가 존재함
      * 고층빌딩 사이에서 multi-path 문제가 있음
      * 실내 혹은 지하에서는 사용이 불가능함
      * 벽에 의해 막히는 경우가 잦음
      * 국내 GNSS(KPS)는 아직 미구현
  * LiDAR
    * 적외선 레이저를 쏘고 반사 시간을 측정하여 거리를 추정하는 센서
    * 주변 환경을 3D point cloud 형태로 바로 알 수 있다
    * 장점
      * Exteroceptive 센서 중 정확한 편
      * 낮/밤에 관계없이 잘 동작함
    * 단점
      * 가격이 비쌈
      * 카메라에 비해 resolution 낮음
      * 날씨에 영향을 받음
      * Multi-path 문제
  * RADAR
    * 반사되어 돌아오는 전파를 측정하여 radial 거리를 재는 센서
    * LiDAR보다 정확도는 낮음
    * Doppler 효과를 이용해서 이동중인 물체의 속도 추정 가능
    * 전파의 종류를 바꿈으로써 near-range와 far-range 선택 가능
    * 장점
      * 날씨의 영향을 받지 않음
      * 속도 값 추정 가능
    * 단점
      * 작은 물체들은 detection 실패
      * LiDAR보다 낮은 resolution
      * Multi-path 문제
  * Ultrasound
    * 초음파를 이용하며 RADAR와 작동방식이 동일하다
    * 장점
      * 저렴함
      * 가까운 거리에도 잘 작동함
    * 단점
      * 물체의 형태를 잘 추정하지 못함(거리 센서로 활용하는 이유)
      * 노이즈가 심함
  * Camera
    * 광센서를 이용해 빛 신호를 받고, debayering 프로세스를 통해 RGB 색 재구성
    * 장점
      * 저렴함
      * 좋은 성능(Dense data, Texture,Color 등)
      * 렌즈 교환을 통해 시야각 변경 가능
      * 사람이 보는 시야와 가장 유사함(디버깅 용이성)
    * 단점
      * Depth 정보 손실
      * 조명의 영향을 받음
  * Microphones
    * 공기의 진동을 transducer 센서를 통해 전기신호로 변환하는 센서
    * 여러개의 마이크를 통해 소리의 근원에 대한 위치를 계산 가능
    * 장점
      * 저렴한 가격
    * 단점
      * Geometry가 부정확
      * noise가 심함
      * 
2. SLAM의 종류
* Visual-SLAM/VSLAM -> 카메라 사용
* LiDAR SLAM -> 라이다 사용
* RADAR SLAM -> 레이더 사용
* Visual-SLAM
  * 장점
    * 저렴한 센서를 사용
    * 센서의 성능을 조절하기 쉬움
    * 센서 속도가 빠른편(30~60FPS)
    * 이미지 기반 딥러닝 적용 가능
    * 이미지로 사람이 이해하기 쉬운 시각화 가능
  * 단점
    * 갑작스러운 빛 변화에 대응 불가능
    * 시야가 가려지거나 어두운 곳에서 사용 불가능
* VSLAM Sensors
  * Camera configuration
    * RGB camera
    * Grayscale camera
    * Multi-spectral camera
    * Polarized camera
    * Event camera
  * Lens configuration
    * Perspective camera
    * Wide FOV camera
    * Telecentric camera
    * Fisheye camera
    * 360 degree camera
* Types of camera configuration
  * Monocular camera - 1 카메라
    * 특징
      * 1대의 카메라에서만 이미지를 받음
      * 연구용 알고리즘이라는 인식이 있음
    * 장점
      * 매우 저렴
    * 단점
      * Scale ambigulty : 3D 공간을 실제 스케일로 추정할 수 없음
      * 이 문제를 해결하려면 실제세상에서 통용되는 스케일을 가진 proprioceptive sensor가 필요

  * Stereo camera - 2카메라 / Multi camera - N 카메라
    * 특징
      * 2대 혹은 N대의 카메라를 사용
      * 인접한 카메라들간의 baseline 거리를 이용하여 삼각측량을 통해 거리/길이 추정 가능
    * 장점 
      * 두 이미지 간의 disparity 정보를 이용해서 픽셀마다 depth를 추정할 수 있음
      * Metric scale의 3D 공간을 복원 가능
    * 단점
      * 카메라 설정 및 캘리브레이션이 어려움
    
  * RGB-D camera(Depth camera)
    * 특징 
      * 구조광 또는 ToF 센서를 이용한 카메라를 사용
      * 센서가 Depth 값을 직접 얻어주기 때문에 계산필요 X
      * Dense mapping을 많이 하는 편
    * 장점
      * Depth 데이터를 통해 3D 공간을 metric scale로 실시간 복원 가능
    * 단점
      * ~10m 정도에서만 depth 데이터가 정확함 
      * Field of view가 작음
      * 적외선 파장이 햇빛과 간섭하여실외에서 사용 불가