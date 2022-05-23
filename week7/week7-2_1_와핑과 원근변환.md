# 와핑기법과 원근 변환

1. Warping 정의
* 이미지의 기하학적 변형
* 영상시스템에서 영상을 이동, 회전, 크기변환 등을 이용해 이미지를 찌그러뜨리거나 반대로 찌그러진 이미지를 복원하기 위한 처리 기법
* 170도 영상으로 왜곡된 카메라 영상을 복원할 수 있음

2. Warping 기법 종류
* 변환(Transformations)
    + 좌표 x를 새로운 좌표 x'로 변환하는 함수
    + 사이즈 변경(Scaling), 위치변경(Translation), 회전(Rotation) 등


    (1) 강체변환(Rigid-Body) : 크기 및 각도가 보존되는 변환(ex Translation, Rotation)

    (2) 유사변환(Similarity) : 크기는 변하고 각도는 보존되는 변환(ex Scaling)

    (3) 선형변환(Linear) : Vector 공간에서의 이동

    (4) Affine : 선형변환과 이동변환까지 포함. 선의 수평성은 유지(e.g. 사각형 -> 평행사변형)

    (5) Perspective : Affine변환에 수평성도 유지되지 않음. 원근변환

3. 패키지 생성

```bash
$catkin_create_pkg sliding_drive std_msgs rospy
```

* /xycar_ws/src/sliding_drive/src

4. Translation 변환
* 평행이동
    + x_new = x_old + d1 = 1*x_old + 0*y_old + d1
    + y_new = y_old + d2 = 0*x_old + 1*y_old + d2

* 변환행렬을 사용하는 OpenCV함수
    + dst= cv2.wrapAffine(src, matrix,dsize,dst,flags,borderMode,borderValue)

    + e.g. dst= cv2.warpAffine(img, M,(cols,rows))

    + flags : 보간법 알고리즘 플래그
        - cv.INTER_LINEAR : default 값, 인접한 4개 픽셀 값에 거리 가중치 사용
        - cv.INTER_NEAREST : 가장 가까운 픽셀 값 사용
        - cv.INTER_AREA : 픽셀 영역 관계를 이용한 재샘플링
        - cv.INTER_CUBIC : 인접한 16개 픽셀 값에 거리 가중치 사용

    + borderMode : 외각영역 보정 플래그
        - cv.BORDER_CONSTANT : 고정 색상 값
        - cv.BORDER_REPLICATE : 가장자리 복제
        - cv.BORDER_WRAP : 반복
        - cv.BORDER_ReFLECT : 반사

        