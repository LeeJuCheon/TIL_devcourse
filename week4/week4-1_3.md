# RVIZ 기반 3D 자동차 제어 프로그래밍

1. 3D 자동차 모델링 패키지
    * rviz_xycar 패키지
        + RVIZ에서 자동차를 3D 형상으로 모델링
        ```bash
        $roslaunch rviz_xycar_3d.launch
        ```
        

    * joint_states 토픽 확인
    ``` bash
    $rostopic info joint_states
    $rosmsg show sensor_msgs/JointState
    $rostopic echo joint_states
    ```

2. 프로그래밍으로 바퀴 움직이기
    * move_joint.py 파일로 joint_state_publisher 제어창 대신

    * move_joint.launch
    ```html
    <launch>
        <param name="robot_description" textfile="$(find rviz_xycar)/urdf/xycar_3d.urdf"/>
        <param name="use_gui" value="true"/>
        <!-- rivz display-->
        <node name="rviz_visualizer" pkg="rviz" type ="rviz required="true" args="-d $(find rviz_xycar)/rviz/xycar_3d.rviz"/>
        <node name  ="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
        <node name="move_joint" pkg="rviz_xycar" type="move_joint.py"/>
    </launch>

    ```

    * move_joint.py
    ```python
    #!/usr/bin/env python
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg  import Header
    rospy.init_node('move_joint')
    pub=rospy.Publisher('joint_states', JointState, queue_size=10)
    hello_xycar = JointState()
    hello_xycar.header=Header()

    hello_xycar.name = ['front_right_hinge_joint','front_left_hinge_joint','front_right_wheel_joint','front_left_wheel_joint','rear_right_wheel_joint','rear_left_wheel_joint']
    hello_xycar.velocity=[]
    hello_xycar.effort=[]

    a=-3.14
    b=-3.14

    rate= rospy.Rate(50)
    whilen ot rospy.is_shutdown():
        hello_xycar.header.stamp=rospy.Time.now()
        if a>=3.14:
            a=-3.14
            b=-3.14
        else:
            a+=0.01
            b+=0.01

    hello_xycar.position=[0,0,a,b,0,0]
    pub.publish(hello_xycar)
    rate.sleep()
    ```

    * 기타 bash coding
    ```bash
    $chmod +x move_joint.py
    $cm
    $loslaunch rviz_xycar move_joint.launch
    ```

3. RVIZ로 3D 자동차 제어 프로그래밍

    * rviz_drive.launch
    ```html
    <launch>
        <param name="robot_description" textfile="$(find rviz_xycar)/urdf/xycar_3d.urdf"/>
        <param name="use_gui" value="true"/>

        <!-- rviz display -->
        <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
                    args="-d $(find rviz_xycar)/rviz/rviz_drive.rviz"/>

        <node name="robot_state_publisher" pkg="robot_state_publisher" 
                    type="state_publisher"/>

        <node name="driver" pkg="rviz_xycar" type="rviz_8_drive.py" /> 
        <node name="converter" pkg="rviz_xycar" type="converter.py" />

    </launch>
    ```

    * rviz_8_drive.py
    ``` python
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    # -*- coding: euckr -*-

    # import 필요한 모듈 가져오기
    import rospy
    import time
    from xycar_motor.msg import xycar_motor


    motor_control = xycar_motor()

    # 노드 새로 만들기
    rospy.init_node('driver')

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
        
        for i in range(30):
            motor_pub(angle,speed)
            time.sleep(0.1)

        angle = 50

        for i in range(40):
            motor_pub(angle,speed)
            time.sleep(0.1)

        angle=0
        for i in range(30):
            motor_pub(angle, speed)
            time.sleep(0.1)

            
    ```

    * converter.py
    ```python
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-

    # import 필요한 모듈 가져오기

    import rospy, rospkg
    import math
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from xycar_motor.msg import xycar_motor

    #def callback xycar_motor 토픽을 받을 때마다 불려지는 콜백 함수 정의하기

    def callback(msg):

        global hello_xycar, l_wheel, r_wheel, pub
        Angle = msg.angle

        steering = math.radians(Angle * 0.4)

        if l_wheel > 3.14:
            l_wheel = -3.14
            r_wheel = -3.14

        else:
            l_wheel +=0.01
            r_wheel +=0.01

        hello_xycar.position = [steering,steering,r_wheel,l_wheel,r_wheel,l_wheel]
        pub.publish(hello_xycar)
        #hello_xycar.position=[msg.angle,msg.angle,0,0,0,0]
        #hello_xycar.velocity=[0,0,msg.speed,msg.speed,msg.speed,msg.speed]

    rospy.init_node('converter')
    pub=rospy.Publisher('joint_states',JointState)

    hello_xycar = JointState()
    hello_xycar.header=Header()
    hello_xycar.name = ['front_right_hinge_joint','front_left_hinge_joint','front_right_wheel_joint','front_left_wheel_joint','rear_right_wheel_joint','rear_left_wheel_joint']
    hello_xycar.velocity=[]
    hello_xycar.effort=[]

    l_wheel = -3.14
    r_wheel = -3.14


    # 토픽의 구독을 준비
    rospy.Subscriber("xycar_motor", xycar_motor, callback)


    # 무한루프에서 토픽이 도착하기를 기다림
    rospy.spin()
    
    ```