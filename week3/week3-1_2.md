# ROS 설치와 구동 실습

1. ROS 설치
     >준비물

        리눅스 Ubuntu 18.04가 설치된 노트북 또는 PC
        파일 다운로드를 위한 인터넷 환경
     > 설치과정

    1) ROS를 제공하는 Sofrware Repository 등록
        
        ```bash
        $sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        $cat /etc/apt/sources.list.d/ros-latest.list
        ``` 

        ![](2022-02-28-16-52-45.png)
    2) apt key를 셋업

        ```bash
        $sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        ```
        ![](2022-02-28-17-14-13.png)
        #### * 키 입력시 오타가 나지 않게 주의!!!

    3) 패키지 설치
        
        ```bash
        $sudo apt-get update
        ```

        ![](2022-02-28-17-17-40.png)
        

        ```bash
        $sudo apt install ros-melodic-desktop-full
        ```

        ![](2022-02-28-17-18-10.png)

        #### * 해당 메세지에서 Y 입력

    4) rosdep 초기화

        ```bash
        $sudo apt install python-rosdep
        $sudo rosdep init
        $rosdep update
        ```
    5) shell 환경 설정

        ```bash
        $echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
        $source ~/.bashrc
        ```
    6) 추가로 필요한 도구 등 설치

        ```bash
        $sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
        ```
        ![](2022-02-28-17-49-26.png)
        #### * 해당 메세지에서 Y 입력
    7) 설치 확인
        
        * 다음 명령을 실핼했을 때 아래 이미지와 같은 화면이 나오면 OK
        ```bash
        $roscore
        ```
        ![](2022-02-28-17-51-03.png)
        #### * 나중에 여기에서 Ctrl + C 입력해야 roscore가 종료된다

2. ROS 프로그래밍을 위한 환경 설정

    > ROS Workspace
    * ROS에서 코딩을 하기 위한 공간
        ```bash
        $cd                        #Home 폴더로 이동
        $mkdir -p ~/xycar_ws/src   #서브 폴더 생성
        $cd xycar_ws               #xycar_ws 폴더 아래로 이동
        $catkin_make               #ROS 코딩 환경 셋업과 정리 (빌드)
        ```
        + 해당 작업을 끝내면 Home 디렉터리 아래 xycar_ws가 생성되고 해당 디렉터리 내에 src가 생성됨. 이후 작성하는 소스코드는 /src에 넣는다
    * 빌드 명령 : catkin_make
        + ROS의 Workspace에서 새로운 소스코드 파일이나 패키지가 만들어지면 catkin_make 명령을 통해 빌드(bulid) 작업을 진행
        + ROS 프로그래밍 작업과 관련있는 모든 것들을 깔끔하게 정리해서 최신 상태로 만드는 작업

    > 환경변수 설정
    * ROS 작업에 필요한 환경변수 설정
        ```bash
        $cd                        # home 디렉터리로 이동
        $sudo gedit ~/.bashrc      # bashrc 내용 수정
        $source .bashrc            # 수정한 내용을 시스템에 반영
        ```
        + bashrc 추가 내용

        ```bash
        alias cm='cd ~/xycar_ws && catkin_make'   # cm을 오른쪽 식으로 대체
        source /opt/ros/melodic/setup.bash         # source 시스템에 알림
        source ~/xycar_ws/devel/setup.bash
        export ROS_MASTER_URL=http://localhost:11311 # master의 주소
        export ROS_HOSTNAME=localhost               # 본인의 주소
        ```

    * ROS 작업에 필요한 환경변수 설정 확인

        ```bash
        $printenv |grep ROS
        ```

        ![](2022-02-28-18-20-46.png)

3. ROS 예제 프로그램 구동 실습
    > ROS core 실행

    1) 마스터(roscore)의 실행 [terminal 1]

        ```bash
        $roscore
        ```

        ![](2022-02-28-18-51-37.png)


    2) ROS node의 확인 [terminal 4]

        ```bash
        $rosnode list
        ```
        
        ![](2022-02-28-18-52-42.png)

    3) ROS node 실행(1) [terminal 2]
        
        ```bash
        $rosrun turtlesim turtlesim_node
        ```

        ![](2022-02-28-18-54-12.png)

    4) ROS node 실행 확인 [terminal 4]

        ```bash
        $rosnode list
        ```
        ![](2022-02-28-18-55-16.png)

    5) ROS node 실행 - 사용자 입력에 맞춰 토픽 발행 [terminal 3]

        ```bash
        $rosrun turtlesim turtle_teleop_key     # 키보드에 따라 
        ```

        ![](2022-02-28-19-01-34.png)

    6) 노드 사이에서 토픽 주고 받기 [terminal 4]

        ```bash
        $rosnode list
        ```
        ![](2022-02-28-19-03-41.png)

        ![](2022-02-28-19-05-26.png)
    
    7) 토픽 조사 [terminal 4]

        ```bash
        $rqt_graph          # 노드를 그래프 형식으로 보여줌
        $rostopic list      # 어떤 토픽이 날아다니는지 보여줌
        $rostopic echo /turtle1/cmd_vel # 토픽이 담긴 메세지의 내용 출력
        $rostopic list -v               # 토픽 자세하게 출력
        $rostopic type /turtle1/cmd_vel # 이 토픽에 발행되는 메세지 타입 출력
        $rosmsg show geometry_msgs/Twist # 타입의 메세지 구성 출력
        ```
        ![](2022-02-28-19-08-52.png)

    + 토픽 직접 발행 
        
        ```bash
        $rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'        # -1 : 발행 한번만 / 토픽 / 메세지 타입 / 메세지 내용
        ```
    + 주기적으로 반복 발행되는 메세지
        ```bash
        $rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'        # 토픽 / 메세지 타입 / 메세지 내용 / -r 1 = 발행주기 1Hz = 1초에 한번씩
        ```        

    > ROS 토픽 메세지 레퍼런스
     * https://wiki.ros.org/ 