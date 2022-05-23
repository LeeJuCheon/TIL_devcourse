# OpenCV 자이카 카메라 활용

1. 카메라 관련 ROS 패키지
* /xycar_ws/src/usb_cam
* 토픽
    + /usb_cam/image_raw    -> 원본 연상
    + /usb_cam/image_raw/compressed -> 압축된 영상

    + type : sensor_msgs/Image
    + 구성
        ```
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        uint32 height
        uint32 width
        ...
        uint8[] data
        ```

* 카메라 기능 사용
    + Lauch 파일에서 "usb_cam"노드 실행
    ```html
    <launch>
        <node name = "usb_cam" pkg = "usb_cam" type="usb_cam_node" output= "screen">
            <param name = "video_device" value = "/dev/v41/by=id/..."/>
            <param name = "autoexposure" value ="false"/>
            <param name = "exposure" value ="150"/>
            <param name = "image_width" value ="640"/>
            <param name = "image_height" value ="480"/>
            <param name = "pixel_format" value ="yuyv"/>
            <param name = "camera_frame_id" value ="usb_cam"/>
            <param name = "id_method" value ="mmap"/>
    ```

* 카메라 설정값 조정 (파라미터 세팅)
    + 해상도 Resolution
        - <param name = "image_width" value ="640"/>
        - <param name = "image_height" value ="480"/>
    + 노출도 Exposure
        - <param name = "autoexposure" value ="false"/>
        - <param name = "exposure" value ="150"/>
        - 주변 광원의 밝기에 따라 알맞은 노출을 설정함으로써 오브젝트 인식의 정확도를 높일 수 있음

* 패키지 생성
```bash
$catkin_create_pkg my_cam std_msgs  rospy
```
* launch 파일(edge_cam.launch)
```html
...
```

* 카메라 영상 출력 프로그램(edge_cam.py)
```cpp
#!/usr/bin/env python
...

```

2. 동영상 파일 제작
* 작업 폴더 : xycar_ws/src/ex_codes/rosbag_ex
* ROS 토픽의 저장
```bash
$rosbag record -a(날아다니는 모득 토픽 저장, 멈추려면 ctrl+c)
$rosbag record rosout xycar_imu(특정 토픽만 저장)
$rosbag record -O subset xycar_ultrasonic(토픽을 subset.bag 파일로 저장)
$rosbag info subset.bag(저장된 파일의 각종정보 보여줌)
```
* ROS 토픽의 재생
```bash
$rosbag play subset.bag(저장했던 토픽 재생)
$rosbag play -r 2 subset.bag(2배속 재생)
```

* 저장된 ROS bag 파일에서 카메라 토픽만 꺼내기
```bash
$rosbag play full_topic.bag
$rosbag record -O cam_topic /usb_cam/image_raw/
$rosbag info cam_topic.bag
```

* 레코드 - 재생 - 토픽 확인 순으로 실행

* 카메라 토픽을 모아 동영상 파일 만들기
```bash
$ rosrun image_view video_recorder image:='/usb_cam/image_raw'
_filename:='track2.avi' _fps:=30
$rosbag play cam_topic.bag
```

