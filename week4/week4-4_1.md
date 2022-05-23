# 라이다 센서 활용

1. 라이다 토픽 (/scan 토픽)

    * 타입 : sensor_msgs/LaserScan
    * 구성 :
        std_msgs/Header header // 헤더( 시퀸스 번호와 시간, 아이디 정보 등)
            uint32 seq
            time stamp
            string frame_id
        float32     angle_min
        float32     angle_max
        ...
        float32[]   ranges  //장애물까지의 거리정보

    
    * 패키지 생성
    ``` bash
    $catkin_create_pkg my_lidar std_msgs rospy
    ```

    * lidar_scan.py 작성
    ```python
    #!/usr/bin/env python

    import rospy
    import tim
    from sensor_msgs.msg import LaserScan

    lidar_points = None

    def lidar_callback(data):
        global lidar_points
        lidar_points = data.ranges

    rospy.init_node("my_lidar",anonymous=True)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    while not rospy.is_shutdown():
        if lidar_points == None:
            continue

        rtn = ""
        
        for i in range(12):
            rtn+=str(format(lidar_points[i*30],'.2f')) + ", "
        
        print(rtn[:-2])
        time.sleep(1.0)
    ```

    * lidar_scan.launch 작성
    ```html
    <launch>
        <include file="$(find xycar_lidar)/launch/lidar_noviewer.launch" />
        <node pkg="my_lidar" type="lidar_scan.py" name="my_lidar" output="screen" />
    </launch>
    ```

    * 실행
    ```bash
    $roslaunch my_lidar lidar_scan.launch
    ```
    