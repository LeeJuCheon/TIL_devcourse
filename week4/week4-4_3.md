# RVIZ 기반 라이다 뷰어 제작

1. ROSBAG
    * 토픽을 구독하여 파일을 저장하거나, 파일에서 토픽을 꺼내 발행하는 기능

    * 사용법(Terminal)
    ```bash
    $rosbag record -O lidar_topic scan
    $rosbag play lidar_topic.bag
    ```
    * 사용법(Launch)
    ```html
    <launch>
        <node name ="rosbag_play" pkg ="rosbag" type="play" output ="screnn" required="true" args = "$(find rviz_lidar)/src/lidar_topic.bag"/>
    </launch>
    ```

    * rosbag 저장 파일
        + /scan 토픽이 저장된 .bag 파일(lidar_topic.bag)

    * Range 데이터 발행
        1) Range 타입의 데이터를 담은 /scan1, /scan2, /scan3, /scan4의 4개 토픽을 발행
        2) RVIZ에서는 원뿔 그림으로 range 거리정보를 시각화하여 표시

2. RVIZ 기반 라이다
    * 파일 구성 : 기존 rviz_lidar 패키지에서 작업
    * 노드 연결관계
        + 노드 이름 : lidar_range
        + 토픽 이름 : /scan1, /scan2, /scan3, /scan4
        + 메시지 타입 : Range(from sensor_msgs.msg import Range)

    * 토픽을 Publish 하는 노드 만들기
        + /src 폴더 아래에 lidar_range.py 작성
        + 4개의 publisher를 선언하고 모두 각각 다른 토픽 이름으로 발행하도록 작성

    * 발행하는 토픽의 데이터를 range 타입으로 설정
    > 메세지 구조 확인
    ```bash
    $rosmsg show sensor_msgs/Range
    ```

    * lidar_range.py 작성
    ``` python
    #!/usr/bin/env python

    import serial, time, rospy
    from sensor_msgs.msg import Range
    from std_msgs.msg import Header

    rospy.init_node('lidar_range')

    pub1 = rospy.Publisher('scan1',Range,queue_size=1)
    pub2 = rospy.Publisher('scan2',Range,queue_size=1)
    pub3 = rospy.Publisher('scan3',Range,queue_size=1)
    pub4 = rospy.Publisher('scan4',Range,queue_size=1)

    msg = Range()
    h = Header()
    h.frame_id = "sensorXY"
    msg.header=h
    msg.radiation_type = Range().ULTRASOUND
    msg.min_range=0.02
    msg.max_range=2.0
    msg.field_of_view=(30.0/180.0)*3.14

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()

        msg.range=0.4
        pub1.publish(msg)
        msg.range=0.8
        pub2.publish(msg)
        msg.range=1.2
        pub3.publish(msg)
        msg.range=1.6
        pub4.publish(msg)

        time.sleep(0.2)
    ```

    * lidar_range.launch 작성
    ```html
    <launch>
        <!-- rviz display -->
        <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
          args="-d $(find rviz_lidar)/rviz/lidar_range.rviz"/>

        <node name="lidar_range" pkg="rviz_lidar" type="lidar_range.py"/>
    </launch>
    ```

    * 토픽 발행 확인
    ```bash
    $rostopic list
    $rostopic echo scan1
    ```

    * Range 뷰어 설정
        + Fixed Frame : Publish 노드에서 "sensorXY"값으로 세팅했으므로 넣어줌
        + 플러그인 추가 : By topic tap -> /scan# -> Range -> OK
    

3. [과제] 라이다 데이터를 Range 데이터로 바꿔서 RVIZ로 출력
    
* lidar_urdf.py
```python
#!/usr/bin/env python

import serial, time, rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

lidar_points = None

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges
rospy.init_node('lidar')

rospy.Subscriber("/scan",LaserScan,lidar_callback,queue_size=1)

pub1 = rospy.Publisher('scan1',Range,queue_size=1)
pub2 = rospy.Publisher('scan2',Range,queue_size=1)
pub3 = rospy.Publisher('scan3',Range,queue_size=1)
pub4 = rospy.Publisher('scan4',Range,queue_size=1)

msg = Range()
h = Header()

#fill in the range message
#h.frame_id = "sensorXY"

# msg.header=h
# msg.radiation_type = Range().ULTRASOUND
# msg.max_range=lidar_points.max_range
# msg.field_of_view=lidar_points.field_of_view

msg.radiation_type = Range().ULTRASOUND
msg.min_range=0.02
msg.max_range=2.0
msg.field_of_view=(30.0/180.0)*3.14



while not rospy.is_shutdown():
    if lidar_points == None:
        continue
    
    # msg.range=lidar_points.range
    h.frame_id = "front"
    msg.header=h
    msg.range=lidar_points[90]
    pub1.publish(msg)

    h.frame_id = "right"
    msg.header=h
    msg.range=lidar_points[180]
    pub2.publish(msg)

    h.frame_id = "back"
    msg.header=h
    msg.range=lidar_points[270]
    pub3.publish(msg)

    h.frame_id = "left"
    msg.header=h
    msg.range=lidar_points[0]
    pub4.publish(msg)

    time.sleep(0.5)
```

* lidar_urdf.launch


```html
<launch>
    <param name="robot_description" textfile="$(find rviz_lidar)/urdf/lidar_urdf.urdf"/>
    <param name="use_gui" value="true"/>

    <node name="rviz_visualizer" pkg="rviz" type="rviz" required="true" 
          args="-d $(find rviz_lidar)/rviz/lidar_range.rviz"/>
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
    <node name="rosbag_play" pkg="rosbag" type="play" required="true" args="$(find rviz_lidar)/src/lidar_topic.bag"/>

    <node name="lidar" pkg="rviz_lidar" type="lidar_urdf.py" output="screen"/>
</launch>

```

* lidar_urdf.urdf
```
<?xml version="1.0" ?>
<robot name="xycar" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <link name="base_link"/>

    <link name = "baseplate">
        <visual>
            <material name="red"/>
            <origin rpy="0 0 0" xyz= "0 0 0"/>
            <geometry>
                <box size="0.2 0.2 0.07"/>
            </geometry>
        </visual>
    </link>
    
    <joint name="base_link_to_baseplate" type="fixed">
        <parent link = "base_link"/>
        <child link="baseplate"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>

    <link name="front"/>

    <joint name="baseplate_to_front" type="fixed">
        <parent link = "baseplate"/>
        <child link="front"/>
        <origin rpy="0 0 0" xyz="0.1 0 0"/>
    </joint>

    <link name="back"/>

    <joint name="baseplate_to_back" type="fixed">
        <parent link = "baseplate"/>
        <child link="back"/>
        <origin rpy="0 0 3.14" xyz="-0.1 0 0"/>
    </joint>

    <link name="left"/>

    <joint name="baseplate_to_left" type="fixed">
        <parent link = "baseplate"/>
        <child link="left"/>
        <origin rpy="0 0 1.57" xyz="0 0.1 0"/>
    </joint>

    <link name="right"/>

    <joint name="baseplate_to_right" type="fixed">
        <parent link = "baseplate"/>
        <child link="right"/>
        <origin rpy="0 0 4.71" xyz="0 -0.1 0"/>
    </joint>

    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
        <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
    </material>
        <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
    </material>
        <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
    </material>
        <material name="orange">
        <color rgba="1.0 0.4235 0.039215 1.0"/>
    </material>
    
</robot>
```