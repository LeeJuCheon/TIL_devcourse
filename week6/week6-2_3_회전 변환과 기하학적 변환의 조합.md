# 회전 변환과 기하학적 변환의 조합

1. 영상의 회전 변환
* 영상을 특정 각도만큼 회전시키는 변환
* openCV는 반시계 방향을 기본으로 사용
    ```cpp
    Mat getRotationMatrix2D(Point2f center, double angle, double scale);
    void warpAffine(InputArray src, OutputArray dst, InputArray M, Size dsize, inf flags = INTER_LINEAR, int borderMode = BORDER_CONSTANT, const Scalar& border Value = Scalar());
    ```
    + center : 회전 중심 좌표
    + angle : 회전 각도. 음수는 시계방향
    + scale : 회전 후 확대 비율
    + 반환값 : 2x3 double 행렬
    + M : 2x3 어파인 변황 행렬
    + dsize: 결과 영상의 크기
    + flags : 보간법 선택
    + borderMode : 가장자리 픽셀 처리 방식
    + borderValue : BORDER_CONSTANT 모드 사용 시 사용할 픽셀 값

* 코드예제
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void on_rotate(int pos, void* data);

int main()
{
	Mat src = imread("lenna.bmp");

	if (src.empty()) {
		cerr << "Image load failed!" << endl;
		return -1;
	}

	imshow("src", src);

	namedWindow("dst");
	createTrackbar("angle", "dst", 0, 360, on_rotate, (void*)&src);
	on_rotate(0, (void*)&src);

	waitKey();
}

void on_rotate(int pos, void* data)
{
	Mat src = *(Mat*)data;

	float degree = (float)pos;
	Point2f pt(src.cols / 2.f, src.rows / 2.f);
	Mat rot = getRotationMatrix2D(pt, degree, 1.0);

	Mat dst;
	warpAffine(src, dst, rot, Size());

	imshow("dst", dst);
}
```

2. 동차좌표계(homogenous coordinates)
* 차원의 좌표를 1차원 증가시켜  표현하는 방법

3. 대칭 변환
    ```cpp
    void flip(InputArray src, OutputArray dst, int flipCode);
    ```
    + flipCode : 대칭 방향 지정(양수 : 좌우대칭, 0 : 상하대칭, 음수 : 좌우& 상하 대칭)
