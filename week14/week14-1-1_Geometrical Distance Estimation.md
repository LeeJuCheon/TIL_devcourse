# Week14 Day 1, Geometrical Distance Estimation

1. 목표
* Computer Vision 에서 활용하는 기하학적 방법으로 객체의 거리를 추정하는 방법 학습

2. Geometrical Method
* 카메라의 설치 높이와 대상 객체가 3차원 공간에 존재하는 특수 조건을 활용한 방법
* Extrinsic Calibration 정보 활용

> I : Image
> 
> P : Pinhole
> 
> f : 초점거리
> 
> H : Height
> 
> Z1, Z2 : Pinhole과 타겟 차량들과의 거리

* 조건 1 : 노면이 평면을 이뤄야 한다
* 조건 2 : 노면과 지면, 광학 축이 평행을 이뤄야 한다 

* 적용된 수식(2차원 기준)
    * P<sub>img</sub>(y) : focal length = height : distance
    * $distance = \frac{focal height * height} {P~img~(y)}$

> FOV(Field of View) : 카메라가 바라보는 이미지에 해당하는 범위
> 
> FOV<sub>V</sub> : Vertical 각도
> 
> FOV<sub>H</sub> : horizontal 각도

* 지면에 붙어있지 않는 대상, 횡방향에 대한 거리 추정에 대한 수식 내용
* 카메라의 고유한 특성(Intrinsic, Sensor&Lens의 속성)과 대상 객체의 실제 크기 정보를 활용



3. Perspective Projection Method 
* 객체가 평면과 관련이 있어야 한다
* Homography를 활용하는 방법
* 이미지에 맺힌 특정 대상은 3차원 공간에 유일하고 유한한 위치에 존재한다
* 하지만 이것을 2차원 공간에 투영한다면 무한개의 점으로 표현이 가능하다
* $P(X,Y,Z) -> P(u,v)$ : Projection
* $P(u,v) -> P(X,Y,Z)$ : Inverse-projection

> Pixel : 2차원 좌표계에서 데이터를 부르는 단위
>
> Voxel : 3차원 좌표계에서 데이터를 부르는 단위

4. Geometrical Distance Estimation Code
* Code Tutorial
  * OpenCV의 Basic concepts of the homography explained with code 튜토리얼
  * https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html

* Code Function
  * getPerspectiveTransform()
    * 4개의 입력 이미지 포인트와 4개의 출력 포인트를 입력값으로 받음
  * findHomography()
    * 4개 이상의 입력 이미지 포인트와 출력 포인트를 입력값으로 받음
    * 오차가 되는 outlier를 제거하는 방법(e.g.RANSAC)이 존재
  * findChessboardCorners()
    * chessboard 이미지를 입력받아 ret과 corners 포인트를 반환
  * findHomography(src,dst,cv.RANSAC)
    * src와 dst에 findChessboard에서 얻은 corner를 인자값으로 사용

  * calibrateCamera()
    * ret : RMS(Root Mean Square) 오차
    * mtx : Intrinsic Parameter
    * dist : Distortion Coefficients
    * rvecs : 각 이미지에 대한 Camera Coordinate에서의 Rotation
    * tvecs : 각 이미지에 대한 Camera Coordinate에서의 Translation
  * drawFrameAxes()
    * 카메라의 POSE를 그리기 위해 도움을 주는 함수

* 원근 변환(Perspective Transform), 투영변환(Projective Transform) -> 투영변환이 좀 더 상위 개념


> 결론 : 객체의 3차원 정보를 복원하는 3가지 방법
> 
> 1. 카메라의 Extrinsic을 이용하는방법
>
> 2. 카메라의 Intrinsic과 대상 객체의 사전 정보를 이용하는 방법
> 
> 3. 카메라의 Image Plane과 대상 Plane의 변환 Matrix를 사용하는 방법

