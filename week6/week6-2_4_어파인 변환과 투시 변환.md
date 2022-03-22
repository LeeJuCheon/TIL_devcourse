# 어파인 변환과 투시 변환

1. 어파인 변환 행렬 구하기
    ```cpp
    Mat getAffineTransform(const Point2f src[], const Point2f dst[]);
    Mat getAffineTransform(InputArray src,InputArray dst);
    ```
    + src : 3개의 원본좌표점
    + dst : 3개의 결과 좌표점
    + 반환값 : 2x3 크기의 변환행렬

2. 투시변환 행렬 구하기
    ```cpp
    Mat getPerspectiveTransform(const Point2f src[], const Point2f dst[], int solveMethod = DECOMP_LU);
    Mat getPerspectiveTransform(InputArray src, InputArray dst, int solveMethod =DECOMP_LU);
    ```
    + src : 4개의 원본 좌표점
    + dst : 4개의 결과 좌표점
    + 반환값 : 3x3 크기의 변환행렬

3. 영상의 어파인 변환
    ```cpp
    void warpAffine(InputArray src, OutputArray dst, InputArray M, Size dsize, inf flags = INTER_LINEAR, int borderMode = BORDER_CONSTANT, const Scalar& border Value = Scalar());
    ```
    + M : 2x3 어파인 변환 행렬
    + dsize: 결과 영상의 크기
    + flags : 보간법 선택
    + borderMode : 가장자리 픽셀 처리 방식
    + borderValue : BORDER_CONSTANT 모드 사용 시 사용할 픽셀 값

4. 영상의 투시 변환
    ```cpp
    void warpPerspective(InputArray src, OutputArray dst, InputArray M, Size dsize, inf flags = INTER_LINEAR, int borderMode = BORDER_CONSTANT, const Scalar& border Value = Scalar());
    ```
    + M : 3x3 투시 변환 행렬
    + dsize: 결과 영상의 크기
    + flags : 보간법 선택
    + borderMode : 가장자리 픽셀 처리 방식
    + borderValue : BORDER_CONSTANT 모드 사용 시 사용할 픽셀 값