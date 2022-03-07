# 자이카 ROS 패키지

> 자이카의 ROS Package 구성
    
    1) 모터제어기     

    2) 라이다

    3) 카메라

    4) 초음파 센서

    5) IMU

    6) Depth 카메라

1. 자동차 구동부

전후진 구동모터 + 좌우회전 조향모터로 이루어짐

* 구동모터(wheel motor):
    + 모터의 회전 속도는 공급되는 전력의 양에 따라 결정
    + 배터리 전력이 모터 제어기를 거쳐 모터로 전달됨
    + 모터의 회전력이 기어 박스를 통해 바퀴의 회전력으로 변환됨

* 조향 모터(Streeing Motor)

    + 핸들 조작을 위한 모터

* 모터 제어 구조
    + VESC 장치에서 2개의 제어 신호 생성(구동모터 제어신호, 조향모터 제어 신호)

* 모터 제어를 위한 ROS Package
    + xycar_device / xycar_motor

    ```bash
    $roscd xycar_motor # 경로접속
    $rostopic list     # 토픽정보 확인
    $rostopic info /xycar_motor #xycar_motor 노드가 구독하는 메시지 타입은 [조향각, 속도] 정보를 담은 custom 메시지

    $rostopic type /xycar_motr  # 타입 확인
    $rostopic show xycar_msgs/xycar_motor   # seq, stamp, frame_id, angle, speed

    $rostopic echo /xycar_motor # 토픽 메세지 출력

    $rqt_graph              # 현재 노드와 메시지 상태를 시각적으로 확인
    ```



    

    

