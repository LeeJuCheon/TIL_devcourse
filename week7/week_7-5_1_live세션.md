# Live 세션

1. 크기 불변 특징
* Harris, GFTT, FAST 코너의 문제점
    + 이동, 회전 변환에 강인
    + 크기 변환에 취약

* 특징점(feature point) = 키포인트(keypoint) = 관심점(interest point)
* 기술자(descriptor) = 특징 벡터(feature vector)

* 크기 불변 특징점 검출 방법
    + SIFT, KAZE, AKAZE, ORB 등 다양한 특징점 검출 방법에서 스케일 스페이스,이미지 피라미드를 구성하여 크기 불변 특징점을 검출

* SIFT 계산 단계
    + Scale-space
    + keypoint localization

* 기타 특징점 기법
    + SURF(Speed-up Robust Features)
        - SIFT를 기반으로 속도를 향상시킨 크기 불변 특징점 검출 방법
        - DOG함수를 단순한 이진 패턴으로 근사화
        - 적분 영상을 이용하여 속도 향상

    + KAZE
        - 비선형 스케일 스페이스에서 공기의 흐름
        - 가우시안 함수 대신 비선형 확산 필터를 이용하여 특징점을 검출
        - SURF 보다는 느리지만 SIFT보다 빠르고 동등한 성능
    + BRIEF
        - 이진 기술자를 이용한 빠른 키포인트 기술 방법
        - 키포인트 주변 픽셀 쌍을 미리 정하고, 픽셀 값의 크기를 비교하여 0 또는 1로 특징을 기술
    + ORB
        - FAST 방법으로 키포인트를 찾고 Harris 코너 방식으로순위를 매김
    + AKAZE

2. OpenCV 특징점 클래스