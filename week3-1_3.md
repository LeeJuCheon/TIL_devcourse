# ROS 프로그래밍 기초(1)

1. ROS Package

    * 패키지 : ROS에서 개발되는 소프트웨어를 논리적 묶음으로 만든 것

    > ROS가 제공하는 편리한 명령들

    * rospack list
        + 어떤 패키지들이 있는지 나열
    * rospack find [package_name]
        + 이름을 이용해서 패키지 검색
    * roscd [location_name[/subdir]]
        + ROS 패키지 디렉토리로 이동
    * rosls [location_name[/subdir]]
        + Linux ls 와 유사
    * rosed [file_name]
        + 에디터로 파일을 편집

    > ROS 패키지 만들기

    * 패키지를 담을 디렉토리로 이동
        ```bash
        $cd ~/xycar_ws/src
        ```
    * 패키지 새로 만들기
        ```bash
        $catkin_create_pkg my_pkg1 std_msgs rospy
        ```

    * ROS 패키지 빌드
        ```bash
        $cd ~/xycar_ws
        $catkin_make
        ```
    * 만들어진 패키지 확인
        ```bash
        $rospack find my_pkg1
        $rospack depends1 my_pkg1
        $roscd my_pkg1
        ```

2. ROS 프로그램의 실행과 검증 확인
    > 코드 작성

    ```python
    #! /usr/bin/env python

    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node('my_node', anonymous=True)  # 노드 생성
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publisher 객체 생성 (topic, message format, size)

    msg = Twist()
    msg.linear.x=2.0
    msg.linear.y=0.0
    msg.linear.z=0.0
    msg.angular.x=0.0
    msg.angular.y=0.0
    msg.angular.z=1.8

    rate = rospy.Rate(1)  # 1초에 n번

    while not rospy.is_shutdown():  # 종료되지 않으면
        pub.publish(msg)    # publish
        rate.sleep()        
    
    ```

    > 프로그램 실행 권한
    * 작성한 파이썬 코드를 실행시키려면 실행권한이 있어야함

        ```bash
        $chmod +x pub.py  # pub.py에 x(실행권한) 부여
        $ls -l              # 권한 여부 확인 
        ```
    
    > 프로그램 실행과 확인


    1) 마스터(roscore)의 실행 [terminal 1]

        ```bash
        $roscore
        ```
    2) ROS node 실행 [terminal 2]
        
        ```bash
        $rosrun turtlesim turtlesim_node
        ```

    3) pub.py 프로그램 실행 [terminal 3]
        
        ```bash
        $chmod +x pub.py
        $rosrun my_pkg1 pub.py
        ```

    4) 노드 상태 확인 [terminal 4]
        
        ```bash
        $rqt_graph
        $rosnode list
        ```

    > 구독자(Subscriber) 확인
    * 토픽 및 메세지 발행 확인
        ```bash
        $rostopic list
        $rostopic type /turtle1/pose # 타입확인
        $rosmsg show turtlesim/Pose
        $rostopic echo /turtle1/pose    # 토픽에 어떤 메시지를 발행하고 있는지 출력
        ```

    * 코드 작성
        ```python
        #!/usr/bin/env python
        import rospy
        from turtlesim.msg import Pose
        def callback(data):
            s = "Location : %.2f, %.2f" % (data.x, data.y)
            rospy.loginfo(rospy.get_caller_id()+s)

        rospy.init_node("my_listener", anonymous=True)
        rospy.Subscriber("/turtle1/pose",Pose,callback)
        rospy.spin()
        ```

3. 요약
    ```bash
    $cd ~/xycar_ws/src
    $catkin_create_pkg my_pkg1 std_msgs rospy
    $cm                 #bashrc alias
    $cd ~/xycar_ws/src/my_pkg1/src
    $vi sub.py
    $vi pub.py
    $chmod +x sub.py pub.py
    $rosrun my_pkg1 pub.py
    $rosrun my_pkg1 sub.py
    ```