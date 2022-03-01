# ROS 노드 통신 프로그래밍

1. 노드 통신을 위한 패키지 만들기

    1) 패키지만들기

    ```bash
    $cd ~/xycar_ws/src
    $catkin_create_pkg msg_send std_msgs rospy # msg_send 패키지 이름. 이 패키지가 의존하고 있는 다른 패키지들을 나열
    $cd msg_send
    $mkdir launch
    $cm
    ```

    2) 파이썬 프로그램 코딩


    > teacher.py

    ```python
    #!/usr/bin/env python                       

    import rospy                                
    from std_msgs.msg import String             

    rospy.init_node('teacher')                  

    pub = rospy.Publisher('my_topic', String)   

    rate = rospy.Rate(2)                        

    while not rospy.is_shutdown():              
        pub.publish('call me please')          
        rate.sleep()   
    ```                        

    > student.py

    ```python
    #!/usr/bin/env python                                  

    import rospy                                            
    from std_msgs.msg import String                         

    def callback(msg):                                     
        print msg.data                                      
        
    rospy.init_node('student')                              

    sub = rospy.Subscriber('msg_to_student', String, callback)    

    rospy.spin()                                            
    ```

    3) Launch 파일 만들고 실행

    ```bash
    $vi m_send.launch
    $cm
    $chmod +x teacher.py student.py
    $roslaunch msg_send m_send.launch
    ```
    > m_send.launch
    ```html
    <launch>
        <node pkg="msg_send" type="teacher.py" name="teacher"/>
        <node pkg="msg_send" type="student.py" name="student" output="screen"/>
    </launch>
    ```

    ![](2022-03-01-16-02-34.png)


2. 요약

```bash
$cd ~/xycar_ws/src
$catkin_create_pkg msg_send std_msgs rospy
$mkdir launch
$cd ~/xycar_ws/src/msg_send/src
$vi student.py
$vi teacher.py
$chmod +x student.py teacher.py
$cd ~/xycar_ws/src/msg_send/launch
$vi m_send.launch
$cm
$roslaunch msg_send m_send.launch