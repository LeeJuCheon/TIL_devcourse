# 라이다 데이터 시각화

1. RVIZ 라이다 센싱 데이터 시각화
    * 패키지 생성
    ``` bash
    $catkin_create_pkg rviz_lidar rospy tf geometry_msgs urdf rviz xacro
    ```
    
    * lidar_3d.launch 작성
    ```html
    <launch>
        <!-- rviz display -->
        <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
            args="-d $(find rviz_lidar)/rviz/lidar_3d.rviz"/>

        <node name="xycar_lidar" pkg="xycar_lidar" type="xycar_lidar" output="screen">
        <param name="serial_port"         type="string" value="/dev/ttyRPL"/>
        <param name="serial_baudrate"     type="int"    value="115200"/>
        <param name="frame_id"            type="string" value="laser"/>
        <param name="inverted"            type="bool"   value="false"/>
        <param name="angle_compensate"    type="bool"   value="true"/>
        </node>
    </launch>
    ```

    * 라이다 장치가 없는경우
        + 실제 라이다 장치를 대신하여 /scan 토픽을 발행하는 프로그램을 이용
        + <b>ROS에서 제공하는 rosbag 이용</b>

    * lidar_3d_rosbag.launch 작성
    ```html
    <launch>
        <!-- rviz display -->
        <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
            args="-d $(find rviz_lidar)/rviz/lidar_3d.rviz"/>

        <node name="rosbag_play" pkg="rosbag" type="play" output="screen" required="true" args="$(find rviz_lidar)/src/lidar_topic.bag"/>
    </launch>
    ```

    * 실행(라이다 센서 있을시)
    ```bash
    $roslaunch rviz_lidar lidar_3d.launch
    ```
    
    * 실행(라이다 센서 없을시)
    ```bash
    $roslaunch rviz_lidar lidar_3d_rosbag.launch
    ```

    * 라이다 viewer
        + 플러그인(LaserScan) 추가
        + Topic 설정 : RVIZ DisplaysTab - Topic
        + RVIZ Displays Tab - Fixed Frame
        + size 설정 : RVIZ Displays Tab - Size(m)
    