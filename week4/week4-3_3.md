# 자이카 IMU 센서 활용

1. IMU센서 토픽
    * Type : sensor_msgs/imu
    * 구성
        + std_msgs/Header header
            - uint32 seq
            - time stamp
            - string frame_id
        + geometry_msgs/Quaternion orientation
            - float64 x
            - float64 y
            - float64 z
            - float64 w

    * 패키지 만들기
    ```bash
    $catkin_create_pkg my_imu std_msgs rospy
    ```

    * roll_pitch_yaw.py 작성
    ```python
    #!/usr/bin/env python
    import rospy
    import time

    from sensor_msgs.msg import Imu
    from tf.transformations import euler_from_quaternion

    Imu_msg = None

    def imu_callback(data):
        global Imu_msg
        Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 

    rospy.init_node("Imu_Print")
    rospy.Subscriber("imu", Imu, imu_callback)

    while not rospy.is_shutdown():
        if Imu_msg == None:
            continue

        (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)
        
        print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (roll, pitch, yaw))
        
        time.sleep(1.0)
    ```

    * roll_pitch_yaw.launch 작성
    ```html
    <launch>
        <node pkg="xycar_imu" type="9dof_imu_node.py" name="xycar_imu" output="screen">
        <param name="rviz_mode" type="string" value="false" />
        </node>
        <node pkg="my_imu" type="roll_pitch_yaw.py" name="Imu_Print" output="screen" />
    </launch>
    ```

    * 실행
    ```bash
    $roslaunch my_imu roll_pitch_yaw.launch
    ```
    