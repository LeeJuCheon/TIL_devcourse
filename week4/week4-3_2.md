# 센서 ROS 패키지

1. 카메라를 위한 ROS Package
    * UVC 1.1 스펙을 지원하는 카메라 ROS 패키지 사용
        + 웹캠과 같은 범용 USB 카메라 제어용 ROS Package를 사용함
        + 카메라 영상을 촬영하고 압축하고 전송하는 일을 처리함
        + xycar_ws/src/xycar_device/usb_cam

    * 카메라 관련 노드와 토픽
        + /usb_cam/image_raw
            - /auto_drive
            - /darknet_ros

        + /usb_cam/image_raw/compressed -> 압축관련
            - /android

    ```bash
    $roscd usb_cam
    $ls -l
    ```

    * 카메라 노드의 실행
        + /launch/usb_cam-test.launch

    ```bash
    $roslaunch usb_cam usb_cam-test.launch
    ```

    * 카메라 노드와 토픽의 관계
        + 메세지를 발행하는 노드는 /usb_cam
        + 메세지를 구독하는 노드는 /image_raw

    * 카메라 관련 토픽의 정보
    ```bash
    $rostopic info /usb_cam/image_raw
    $rosmsg show sensor_msgs/Image
    $rostopic echo /usb_cam/image_raw | head -n 11
    ```

2. IMU 센서를 위한 ROS Package

    * 9축 IMU 센서를 위한 범용 ROS 패키지 사용
        + ROS 커뮤니티에서 많이 사용하는 IMU 센서 제어용 ROS Package를 사용함
        + 가속도계/자이로/지자계의 정보를 수집하고 전송하는 일을 처리함
        + /xycar_ws/src/xycar_device/xycar_imu

    * IMU센서 관련 노드와 토픽
        + xycar_imu/imu/auto_drive
    ```bash
    $roscd xycar_imu
    ```

    * IMU 노드의 실행
        + /launch/xycar_imu_9dof.launch
    
    ```bash
    $roslaunch xycar_imu xycar_imu_9dof.launch
    ```

    * IMU 관련 토픽의 정보
    ```bash
    $rostopic list
    $rostopic info /imu
    $rosmsg show sensor_msgs/imu
    $rostopic echo /imu
    ```

3. 라이다를 위한 ROS Package
    
    * 라이다 제조사에서 제공하는 ROS 패키지 사용
        + 라이다 제조사가 만들어 배포하는 라이다 제어용 ROS Package를 사용함
        + 라이다로부터 장애물까지의 거리 값을 수집하고 전송하는 일을 처리함
        + xycar_ws/src/xycar_device/xycar_lidar
    
    * 라이다 관련 노드와 토픽
        + xycar_lidar/scan/auto_drive
    ```bash
    $roscd xycar_lidar
    ```

    * 라이다 노드의 실행
        + /launch/lidar.launch
    
    ```bash
    $roslaunch xycar_lidar lidar.launch
    ```

    * 라이다 관련 토픽의 정보
    ```bash
    $rostopic list
    $rostopic info /scan
    $rosmsg show sensor_msgs/LaserScan
    $rostopic echo /scan
    ```

4. 초음파 센서를 위한 ROS Package
    
    * 아두이노와의 통신을 위한 ROS 패키지를 제작하여 사용
        + 아두이노가 각각의 초음파센서 제어하여 거리 정보를 수집함
        + 전체 초음파센서의 거리정보를 통합하여 관리함
        + xycar_ws/src/xycar_device/xycar_ultrasonic
    
    * 초음파 관련 노드와 토픽
        + xycar_ultrasonic/xycar_ultrasonic/auto_drive
    
    ```bash
    $roscd xycar_ultrasonic
    ```

    * 초음파 노드의 실행
        + /launch/xycar_ultrasonic.launch
    
    ```bash
    $roslaunch xycar_ultrasonic xycar_ultrasonic.launch
    ```

    * 초음파 관련 토픽의 정보
    ```bash
    $rostopic list
    $rostopic info /xycar_ultrasonic
    $rostopic echo /xycar_ultrasonic
    ```

5. Depth 센서를 위한 ROS Package
    
    * Depth 카메라 제조사가 제공하는 ROS 패키지를 사용
        + 모두 3개의 카메라를 제어하여 영상정보와 거리정보를 수집
        + 다양한 토픽에 다양한 정보를 담아 제공
        + xycar_ws/src/xycar_device/realsense2_camera
    
    * Depth 카메라  관련 노드와 토픽
        + camera/realsense2_camera/camera/color/image_raw
        + camera/realsense2_camera/camera/depth/image_rect_raw
    
    ```bash
    $roscd realsense2_camera
    ```

    * Depth 카메라 노드의 실행
        + /launch/demo_pointcloud.launch
    
    ```bash
    $roslaunch realsense2_camera demo_pointcloud.launch
    ```

    * Depth 카메라 관련 토픽의 정보
    ```bash
    $rostopic list
    $rostopic info /camera/color/image_raw
    $rostopic info /camera/depth/image_rect_raw
    ```