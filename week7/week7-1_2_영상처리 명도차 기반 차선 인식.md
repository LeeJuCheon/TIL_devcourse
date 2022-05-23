# 영상처리 명도차 기반 차선 인식

1. 차선 인식 프로그램 순서 
    1) 카메라 입력으로 취득한 영상에서 적절한 영역을 잘라냄
    2) 차선의 색깔을 탐색, 점 주위에 사각형 생성
    3) 사각형 내 색깔 점의 개수를 파악

2. OpenCV 기반 영상 처리
* Image Read : 카메라 영상 신호를 이미지로 읽기
* GrayScale : 흑백 이미지 변환
* Gaussian Blur : 노이즈 제거
* HSV - Binary : HSV 기반으로 이진화
* ROI : 관심영역 잘라내기

* gray.py
```python
import cv2
img = cv2.imread('sample.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

cv2.imshow('gray',gray)
cv2.waitKey(10000)
```

* blur.py
    ```python
    import cv2
    img = cv2.imread('sample.png')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)

    cv2.imshow('gray',gray)
    cv2.waitKey(10000)
    ```
    + 각 필셀에 5x5 윈도우를 올려 놓고, 그 영역 안에 포함되는 값을 모두 더한 뒤 이것을 25로 나누어 인접한 점들의 밝기의 산술평균을 구해 노이즈 제거

* line.py
    ```python
    import cv2
    import numpy as np

    img = cv2.imread('sample.png')
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_white=np(array([0,0,70]))
    upper_white=np(array([131,255,255]))


    mask = cv2.inRange(hsv,lower_white,upper_white)

    cv2.imshow('line',mask)
    cv2.waitKey(10000)
    ```

* canny.py
    ```python
    import cv2

    img = cv2.imread('sample.png')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    edge = cv2.Canny(blur,20,190)

    cv2.imshow('edge',edge)
    cv2.waitKey(10000)
    ```

* nonzero.py
    ```python
    import cv2
    import numpy as np

    img = cv2.imread('sample.png')
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_white=np(array([0,0,70]))
    upper_white=np(array([131,255,255]))
    img=cv2.inRange(hsv,lower_white,upper_white)

    xx=20
    while True:
        area = img[430:450 , xx:xx+15]
        if cv2.countNonZero(area) >200:
            image = cv2.rectangle(image,(xx,430),(xx+15,450),(0,255,0),3)
        else:
            image = cv2.rectangle(image,(xx,430),(xx+15,450),(255,0,0),3)
        xx = xx+20

        if xx>640:
            break
    
    cv2.imshow('countNonZero',image)
    cv2.waitKey(10000)
    ```

2. 카메라 활용 차선 검출
* 트랙 영상에서 특정 영역을 ROI로 설정하여 차선위치 검출
    + BGR -> HSV -> 이진화

* 목표 : 검출된 차선을 녹색 사각형으로 표시
* 범위 : 세로(430 ~ 450), 가로(0~200,440~640)


3. 패키지 생성
```bash
$catkin_create_pkg line_drive rospy tf geometry_msgs rviz xacro
```

* line_find.py
```python
#!/usr/bin/env python
# 동영상 videoCapture
# 명도 하한 threshole
# 차선 검출을 위하여 검사할 영역의 범위

while True:
    ret, frame = cap.read()
    #예외처리
    #roi 설정
    #설정된 ROI의 둘레에 파란색 사각형 그림
    #hsv 변환
    #lower_bound
    #upper_bound

    bin = cv2.inRange(hsv,lower_bound,upper_bound)
    #view

    #for문을 활용하여 바깥쪽에서 안쪽으로 범위 탐색

    #차선이 검출되면 잘라낸 ROI 이미지에 녹색 사각형을 그림

    # 영상 출력

```

4. [HW] 실제 트랙에서 차선을 벗어나지 않고 주행
* 프로세스 순서
    + 카메라 입력 데이터에서 프레임 취득
    + 얻어낸 영상 데이터를 처리하여 차선 위치를 결정
        - 색변환 : BGR->HSV 변환
        - 이진화
        - ROI 잘라내기
    + 차선 검출 : 흰색 점들이 모여 있는 곳의 좌표를 계산
        - 검출한 차선위치에 사각형을 그려 차선 검출 결과의 확인에 이용
    + 차선위치를 기준으로 조향각 결정
        - 차선의 중앙을 차량이 달리도록
    + 결정한 조향각에 따라 조향 모터를 제어
        - 모터제어 메시지 전송