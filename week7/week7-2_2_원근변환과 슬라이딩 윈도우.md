# 원근 변환과 슬라이딩 윈도우

1. 단계
    1) camera Calibration
    
    2) Bird's eye View

    3) 이미지 임계값 및 이진 이미지

    4) 슬라이딩 윈도우로 차선 위치 파악

    5) 파악된 차선 위치 원본이미지에 표시


2. Camera Calibration(카메라 보정)
* 곡면렌즈로 인식한 영상을 실제 우리 눈에 보이는 것 처럼 보정하는 기법
* 왜곡된 지점을 왜곡되지 않은 지점으로 Mapping

* findChessboradCorners() : 체스판의 코너를 찾음
* drawChessboardCorners() : 찾은 체스 판의 코너들을 그림
* cv2.calibrateCamera() : camera matrix, 왜곡계수, 회전/변환 벡터들을 리턴
* cv2.undistort() : 이미지를 펴서 왜곡이 없어지게 함

3. Bird's eye View 
* cv2.getPerspectiveTransform(src,dst) : 원근변환 행렬을 구하는 함수로, 4개의 점의 이동 전과 이동 후 좌표를 입력하면 이동 전 좌표를 이동 후 좌표로 투시 변환함
* cv2.getPerspectiveTransform(dst, src)를 활용하여 역 원근변환
* cv2.warpPerspective(img,M,img_size,flags = cv2.INTER_LINEAR) : 원근변환 행렬을 활용 이미지를 뒤틀어 원하는 구도로 변환

4. 이미지 임계값, 이진 이미지
* 차선이 명확하게 보이는 이미지를 생성하기 위해 색상 임계값 조절
* 이미지를 흰색과 노란색으로 마스킹
* Gray scale로 변환
* 이진 이미지 생성
* HSV / LAB / HLS를 활용하여 색상 검출

5. 슬라이딩 윈도우를 활용한 차선 식별
* 슬라이딩 윈도우
    + 선 중심 주변에 배치된 슬라이딩 윈도우를 사용하여 프레임 상단까지 선을 찾아 따라감
    + 한 윈도우 안에서 감지되는 선의 중심을 기준으로 계속 윈도우가 쌓임
    + 아래쪽 처음 블록은 앞선 히스토그램으로 정의되고, 이미지 아래쪽에서 위쪽으로 검색하며 올라감
