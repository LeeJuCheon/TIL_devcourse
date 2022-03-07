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
    from sensor_msgs.msg import JoinsState
    from std_msgs.msg  import Header
    rospy.init_node('move_joint')
    pub=rospy.Publisher('joint_states', JointStatre, queue_size=10)
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