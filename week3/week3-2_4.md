# ROS 노드 통신 프로그래밍

1. Int32 타입의 메세지를 주고 받는 파이썬 코드
    > bash code
    ```bash
    $cp teacher.py teacher_int.py
    $cp student.py student_int.py
    $vi teacher_int.py
    $vi student_int.py
    ```

    > teacher_int.py

    ```python
    #!/usr/bin/env python                       

    import rospy                                
    from std_msgs.msg import Int32           

    rospy.init_node('teacher')                  

    pub = rospy.Publisher('my_topic', Int32)   

    rate = rospy.Rate(2)                        
    count = 1

    while not rospy.is_shutdown():              
        pub.publish(count)
        count+=1         
        rate.sleep()   
    ```           

    > student_int.py

    ```python
    #!/usr/bin/env python                                  

    import rospy                                            
    from std_msgs.msg import Int32                         

    def callback(msg):                                     
        print msg.data                                      
        
    rospy.init_node('student')                              

    sub = rospy.Subscriber('msg_to_student', Int32, callback)    

    rospy.spin()                                            
    ```

2. 노드를 여러 개 띄울 때
    * 하나의 코드로 여러 개의 노드를 연결하려면 각 노드의 이름이 달라야함
        + 노드의 init 함수에서 anonymous=True 값을 넣어주면 노드이름이 자동 설정

    * Launch 파일을 이용해서 roslaunch 명령으로 여러 노드를 띄울 수 있음
    ```html
    <launch>
        <node pkg="msg_send" type="teacher_int.py" name="teacher1"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher2"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher3"/>
        <node pkg="msg_send" type="student_int.py" name="student1" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student2" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student3" output="screen"/>
    </launch>
    ```

3. 1:N 통신

    > m_send_1n.launch

    ```html
    <launch>
        <node pkg="msg_send" type="teacher_int.py" name="teacher"/>
        <node pkg="msg_send" type="student_int.py" name="student1" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student2" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student3" output="screen"/>
    </launch>
    ```

    ![](2022-03-01-16-31-59.png)


4. N:1 통신
    > m_send_n1.launch

    ```html
    <launch>
        <node pkg="msg_send" type="teacher_int.py" name="teacher1"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher2"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher3"/>
        <node pkg="msg_send" type="student_int.py" name="student" output="screen"/>
    </launch>
    ```
    ![](2022-03-01-16-35-56.png)

5. N:N 통신
    > m_send_nn.launch

    ```html
    <launch>
        <node pkg="msg_send" type="teacher_int.py" name="teacher1"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher2"/>
        <node pkg="msg_send" type="teacher_int.py" name="teacher3"/>
        <node pkg="msg_send" type="student_int.py" name="student1" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student2" output="screen"/>
        <node pkg="msg_send" type="student_int.py" name="student3" output="screen"/>
    </launch>
    ```
    ![](2022-03-01-16-37-29.png)