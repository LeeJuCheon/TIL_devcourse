# RVIZ 오도메트리 활용

1. Odometry 기초

    * Odometer 오도미터
        + 차량이나 로봇이 주행하며 이동한 거리를 측정하는 기기

    * Odometry 오도메트리
        + 오도미터 등의 기기의 측정값으로 움직이는 사물의 위치를 측정하는 방법

        + 바퀴의 회전수로 이동거리를 계산

    * 구성 : 자동차 핸들과 앞 바퀴
        + 앞 바퀴는 회전시 꺾이는 각도가 다름
    
    * Ackermann Streering
        + 간단한 기계식 구조
        + 회전시 꺾이는 각도가 다르도록
        + 안쪽과 바깥쪽 회전 원의 중심이 일치한다 
        
    * 자동차의 위치 정보
        + 현재 위치 : (x,y) 좌표 + ʘ 세타
        + 이동속도 : 선속도 v + 각속도 w
        + 조항각 델타


2. Odometry 토픽
    
    * Odometry 토픽
        + /odom

    * 메세지 타입
        + nav_msgs/Odometry
        + $rosmsg show nav_msgs/Odometry

    * odom 토픽 발행 예제 코드
        + 파이썬 코드 : ros_odometry_publisher_example.py
        + https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

        + 해당 내용 : odometry_publisher 노드 생성 /odom 토픽을 1초에 한번씩 발행

    * 토픽 발행 파이썬 코드(odom_publisher_ex.py)
    ```python
    #!/usr/bin/env python
    import math
    from math import sin, cos,pi

    import rospy
    import tf
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

    rospy.init_node('odometry_publisher') # init odometry_publisher node

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) # odom 토픽 발행 준비
    odom_broadcaster = tf. TransformBroadcaster()

    x=0.0       # 초기 위치 (0,0,0)
    y=0.0
    th=0.0

    vx=0.1      # X축 속도 10cm/s
    vy=-0.1     # Y축 속도 =10cm/s
    vth=0.1     # 주행 방향은 0.1라디안 (5.7도)

    current_time =rospy.Time.now()      # 시간 정보 계산용 변수
    last_time = rospy.Time.now()

    r=rospy.Rate(1.0)                   # 1초에 한번 루프돌기
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt=(current_time = last_time).to_sec()
    delta_x = (vx * cos(th)-vy*sin(th))* dt
    delta_y = (vx * sin(th) + vy*cos(th))* dt
    delta_th = vth * dt

    x+= delta_x
    y+= delta_y
    th+= delta_th

    odom_broadcaster.sendTransform(
        (x,y,0.),               # 위치정보(자세정보)에 대한 발행을 준비
        odom_quat,              # odom과 base_link를 연결하는 효과
        current_time,
        "base_link",
        "odom"
    )

    odom=Odometry()
    odom.header.stamp = current_time        #Odometry 메시지의 헤더 만들기
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x,y,0.), Quaternion(*odom_quat))        # Position 값 채우기

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy,0),Vector3(0,0,vth))                # 속도 값 채우기

    odom_pub.publish(odom)  # /odom 토픽 발행하기

    last_time = current_time    # 잠깐 쉬었다가 루프 처음으로 되돌아감
    r.sleep()
    ```

    * 실행 및 확인
    ```bash
    $roscore
    $rosrun ex_urdf odom_publisher_ex.py
    $rostopic list
    $rostopic info odom
    $rqt_graph

    $rosmsg show nav_msgs/Odometry
    $rostopic echo odom
    ```

3. Odometry 활용
    * odom_pub.launch
    ```html
    <launch>
        ....
        <node name="odom_publisher" pkg ="ex_urdf" type="odom_publisher_ex.py"/>

    </launch>
    ```

    ```bash
    $roslaunch ex_urdf odom_pub.launch
    ```
    + Fixed Frame 을 base_link에서 odom으로 변경
