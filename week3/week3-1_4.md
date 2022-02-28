# ROS 프로그래밍 기초(2)


1. Launch 파일 사용
    
    > roslaunch
    * *.launch 파일 내용에 따라 여러 노드들을 한꺼번에 실행시킬 수 있음
    * 파라미터 값을 노드에 전달 가능

    * 사용법
        + $ roslaunch [패키지이름] [실행시킬 launch 파일 이름]
        + 이때 [실행시킬 launch 파일]은 반드시 패키지에 포함된 launch 파일이어야 함
    
    > .launch 파일
    * roslaunch 명령어를 이용하여 많은 노드를 동시에 실행시키기 위한 파일
    *실행시킬 노드들의 정보가 XML 형식으로 기록되어 있음

    > node Tag
    * 실행할 노드 정보를 입력할 때 사용되는 태그
    
    ```html
    <node pkg ="패키지명" type="노드가 포함된 소스파일 명" name="노드 이름" />
    ```
    * 속성
        + pkg : 실행시킬 노드의 패키지 이름을 입력하는 속성
        + type : 노드의 소스코드가 담긴 파이썬 파일의 이름을 입력하는 속성, 이때 파이썬 .py 파일은 반드시 실행권한이 있어야함
        + name : 노드의 이름을 입력하는 속성

    > include Tag
    * 다른 launch 파일을 불러오고 싶을 때 사용하는 태그
    ```html
    <include file = "같이 실행할 *.launch 파일 경로"/>
    ```
    * 속성
        + file : 함꼐 실행시킬 *.launch 파일의 경로를 입력하는 속성

    > Launch 파일 작성
        
    ```html
    <launch>
        <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node"/>
        <node pkg="my_pkg1" type="pub.py" name="pub_node"/>
        <node pkg="my_pkg1" type="sub.py" name="sub_node" output="screen"/>
    </launch>
    ```

    > Launch 파일 실행

    ```bash
    $roslaunch my_pkg1 pub-sub.launch
    ```

        * 별도로 $roscore 명령을 실행할 필요가 없다.
        * 내부적으로 roscore가 자동으로 실행된다

    > 요약

    ```bash
    $cd ~/xycar_ws/src/my_pkg1
    $mkdir launch
    $cd ~/xycar_ws/src/my_pkg1/launch
    $vi pub-sub.launch
    $cm
    $roslaunch my_pkg1 pub-sub.launch
    ```

2. Launch에서 Tag 활용
    > .launch 파일 사례
    * USB 카메라 구동과 파라미터 세팅을 위한 파일
        + 패키지명 : usb_cam, 파라미터는 5개

    ```html
    <launch>
        <node name="usb_cam" pkg="usb_cam" type="cam_node" output="screen">
            <param name="...." value ="..."/>
            <param name="...." value ="..."/>
            <param name="...." value ="..."/>
            <param name="...." value ="..."/>
            <param name="...." value ="..."/>
        </node>
    </launch>
    ```

    > param
    * ROS 파라미터 서버에 변수를 등록하고 그 변수에 값을 설정하기 위한 태그
    * e.g. <param name=" " type=" ' value" "/>
    * 속성
        + name : 등록할 변수의 이름
        + type : 등록할 변수의 타입. 사용할 수 있는 타입의 종류는 str, int, double, bool, yaml
        + value : 등록할 변수의 값
    
    * ROS 파라미터 서버에 등록된 변수는 노드 코드에서 불러와 사용할 수 있음
        + e.g. ) print(rospy.get_param('~age))
            - private parameter는 앞에 ~을 붙인다


    > 파라미터 전달 실습
    1) Launch 파일 새로 만들기
        ```html
        <launch>
            <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node"/>
            <node pkg="my_pkg1" type="pub_param.py" name="node_param">
                <param name="circle_size" value="2"/>
            </node>
            <node pkg="my_pkg1" type="sub.py" name="sub_node" output="screen"/>
        </launch>
        ```
    2) pub.py 파일을 복사_+수정해서 pub_param.py 파일을 새로 만들기
        ```bash
        $cp pub.py pub_param.py
        $vi pub_param.py
        ```

        ```python
        import rospy
        from geometry_msgs.msg import Twist
        def move():
            rospy.init_node('my_node', anonymous=True)  # 노드 생성
            pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publisher 객체 생성 (topic, message format, size)

            msg = Twist()
            #msg.linear.x=2.0
            linear_X=rospy.get_param('~circle_size')
            msg.linear.x = linear_X
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
    3) launch 파일 실행
    ```bash
    $roslaunch my_pkg1 pub-sub-param.launch
    ```
    