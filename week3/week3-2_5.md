# ROS 노드 통신 프로그래밍

1. 나만의 메세지 만들기

    > Custom Message 사용 방법

    ```bash
    $cd ~/xycar_ws/src
    $roscd Msg_send
    $mkdir msg
    $cd msg
    $vi my_msg.msg
    ```

    * my_msg.msg

    ```
    string first_name
    string last_name
    int32 age
    int32 score
    string phone_number
    int32 id_number
    ```

    > Custom Message 선언

    ```bash
    $vi package.xml
    ```

    * package.xml(파일 아래쪽에 아래 내용 추가)
    
    ```html
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ```

    * CMakeLists.txt 수정
    ```cpp
    find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        message_generation      //새로 추가
    )

    add_message_files(
        FILES
        my_msg.msg    //주석풀고 추가
    )

    generate_messages(
        DEPENDENCIES
        std_msgs       //코멘트 풀기
    )

    catkin_package(
        CATKIN_DEPENDS message_runtime // 한줄 추가
        # INCLUDE_DIRS include
        # LIBRARIES my_package
        # CATKIN_DEPENDS rospy std_msgs
        # DEPENDS system_lib
    )

    ```

2. Custom Message 설정과 확인
    ```bash
    $cm
    $rosmsg show my_msg
    ```
    ![](2022-03-01-16-55-51.png)

3. 내 코드 안에서 Custom Message 사용하기

    * 코드 안에 include or import 하는 법
        + from msg_send.msg import my_msg
    
    * 다른 패키지에서도 custom msg 사용 가능

    * 사용 예제

    
    ```bash
    $vi msg_sender.py
    ```

    > msg_sender.py

    ```python
    #!/usr/bin/env python

    import rospy
    from msg_send.msg import my_msg

    rospy.init_node('msg_sender', anonymous=True)
    pub = rospy.Publisher('msg_to_xycar', my_msg)

    msg = my_msg.msg()
    msg.first_name = "nayoung"
    msg.last_name = "Choi"
    msg.id_number = 20210815
    msg.phone_number = "010-9072-7247"

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        print("sending message")
        rate.sleep()

    ```

    ```bash
    $vi msg_receiver.py

    $chmod +x msg_sender.py msg_receiver.py
    ```

    > msg_receiver.py

    ```python
    #!/usr/bin/env python

    import rospy
    from msg_send.msg import my_msg

    def callback(msg):
        print ("1. Name: ", msg.last_name + msg.first_name)
        print ("2. ID: ", msg.id_number)
        print ("3. Phone Number: " , msg.phone_number)
        
    rospy.init_node('msg_receiver', anonymous=True)

    sub = rospy.Subscriber('msg_to_xycar', my_msg, callback)

    rospy.spin()

    ```
    
    * 실행 결과

    ```bash
    $cm
    $roscore
    $rosrun msg_send msg_receiver.py
    $rosrun msg_send msg_sender.py
    ```

    * launch를 활용한 실행

    ```bash
    $roslaunch msg_send m_send_sr.launch
    ```

    > m_send_sr.launch
    ```html
    <launch>
        <node pkg="msg_send" type="msg_sender.py" name="sender1"/>
        <node pkg="msg_send" type="msg_sender.py" name="sender2"/>
        <node pkg="msg_send" type="msg_receiver.py" name="receiver" output="screen"/>
    </launch>

    ```

    ![](2022-03-01-17-16-48.png)

2. 다양한 상황에서의 노드 통신
    * 누락 없이 잘 작동하는가?
        + 중간보다는 처음과 끝에 에러가 자주 일어날 수 있다
        + receiver를 먼저 실행하고 sender 실행
        + roslaunch는 노드를 순서대로 실행시킬 수 없다

    * 데이터 크기에 따른 전송속도
        + 정해진 크기의 데이터를 반복해서 보냄
        + 10분동안 체크해서 송신속도 계산
        + 만약 receiver가 없다면 ?

    * 도착하는 데이터를 미처 처리하지 못하면?
        + 구독자의 callback 함수에 시간이 많이 걸리는 코드를 넣어 구성

    * 주기적 발송에서 타임 슬롯을 오버한다면?
        + Rate(5) 세팅 후 수행시간을 늘려 구성

    * 협업해야 하는 노드를 순서대로 기동시킬 수 있나?
        + Launch 파일 활용, ROS 활용, 프로그래밍으로 구현(총 3가지 방법으로 각각 구현)

    ### 3.2(수) 직접 구현 예정