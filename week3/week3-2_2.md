# ROS 예제코드 분석

1. teacher.py
    ```python
    #!/usr/bin/env python                       # 쉬뱅(!#)을 사용하여 스크립트 실행환경을 python으로 

    import rospy                                # python의 ros library
    from std_msgs.msg import String             # std_msgs내에 있는 string 자료형 import 

    rospy.init_node('teacher')                  # teacher라는 이름의 노드 생성

    pub = rospy.Publisher('my_topic', String)   # my_topic이라는 이름의 publisher(발행자) 객체 생성, 메세지 포맷은 String

    rate = rospy.Rate(2)                        # 1초에 2번

    while not rospy.is_shutdown():              # rospy가 종료되기 전까지 반복
        pub.publish('call me please')           # call me please publish
        rate.sleep()                            # 반복 1번에 0.5초를 채우도록 sleep
    ```

2. student.py
    
    ```python
    #!/usr/bin/env python                                   # 쉬뱅(!#)을 사용하여 스크립트 실행환경을 python으로 

    import rospy                                            # python의 ros library
    from std_msgs.msg import String                         # std_msgs내에 있는 string 자료형 import 

    def callback(msg):                                      # callback 함수 정의
        print msg.data                                      # 메세지의 내용 print
        
    rospy.init_node('student')                              # student라는 이름의 노드 생성

    sub = rospy.Subscriber('msg_to_student', String, callback)    # my_topic이라는 이름의 Subscriber(구독자) 객체 생성, String 타입의 메세지 구독, 새로운 메세지를 받으면 callback 함수 실행

    rospy.spin()                                            # 종료되기 전까지 반복수행
    ```


3. 동작 시나리오

    1) 마스터(roscore) 실행

    2) Subscriber 노드 구동 : String 타입의 메세지를 수신하기를 마스터에 요청

    3) Publisher 노드 구동 : 토픽 메세지를 발행한다는 내용을 마스터에게 전달한다

    4) 노드간 연결이 이루어진뒤 0.5초에 한번 call me please를 메세지로 받아 callback 함수에 의해 출력된다(단방향 통신)