# 모터제어기 프로그래밍

1. 모터제어 토픽

    * /xycar_motor 토픽
    * 타입 : xycar_msgs/xycar_motor
    * 구성 
        + 헤더(시퀸스번호, 시간, 아이디, 정보)
        + 조향각도(좌회전, 우회전)
        + 주행속도(전진, 후진)
    
    * 토픽 발행이 없으면 (0.7초 동안 도착하는 토픽이 없으면) 속도와 조향각도를 0으로 세팅

2. 자이카 8자 주행 실습

    * 차량 속도 고정, 핸들만 조종

    1) 패키지 생성

    * my_motor 패키지 만들기
    ```bash
    $catkin_create_pkg my_motor std_msgs rospy

    ```

    * 서브폴더 만들기
        + /launch 폴더 생성 및 8_drive.launch 만들기
    
    * 파이썬 코딩
        + src 폴더에 8_drive.py 만들기
        + ROS 프로그래밍, 코딩

    * 조건
        + 구동속도를 3으로 설정
        + 핸들을 꺾어서 8자 모양으로 주행 (좌회전 + 직진 + 우회전 + 직진 ....)

    * 8_drive.py
    ```python
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    # -*- coding: euckr -*-

    # import 필요한 모듈 가져오기
    import rospy
    import time
    from xycar_motor.msg import xycar_motor


    motor_control = xycar_motor()

    # 노드 새로 만들기
    rospy.init_node('my_motor')

    # 토픽의 발행을 준비
    pub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)


    # 토픽을 발행하는 함수 만들기

    def motor_pub(angle,speed):
        global pub
        global motor_control
        motor_control.angle=angle
        motor_control.speed=speed
        
        pub.publish(motor_control)

    # 차량의 속도는 고정시킨다
    speed = 3

    # 차량의 조향각을 바꿔가면서 8자로 주행시킨다.

    while not rospy.is_shutdown():
        angle = -50
        for i in range(40):
            motor_pub(angle,speed)
            time.sleep(0.1)

        angle = 0
        
        for i in range(40):
            motor_pub(angle,speed)
            time.sleep(0.1)

        angle = 50

        for i in range(40):
            motor_pub(angle,speed)
            time.sleep(0.1)

        angle=0
        for i in range(40):
            motor_pub(angle, speed)
            time.sleep(0.1)

            
    ```

    * 8_drive.launch

    ```html
    <launch>
        <include file="$(find xycar_motor)/launch/xycar_motor_a2.launch"/>
        <node pkg ="my_motor" type="8_drive.py" name="auto_driver" output="screen"/>
    </launch>

    ```
            