# 이동변환과 전단변환

1. 영상의 기하학적 변환

* 영상을 구성하는 픽셀의 배치 구조를 변경함으로써 전체 영상의 모양을 바꾸는 작업
* 전처리 작업, 영상정합, 왜곡 제거 등

2. 영상의 이동변환
* 가로 또는 세로 방향으로 영상을 특정 크기만큼 이동시키는 변환
* x축과 y축 방향으로의 이동 변위를 지정
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
	Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

	if (src.empty()) {
		cerr << "Image laod failed!" << endl;
		return -1;
	}

	Mat dst = Mat::zeros(src.rows, src.cols, CV_8UC1);
	for (int y = 0; y < src.rows; y++) {
		for (int x = 0; x < src.cols; x++) {
			int x_ = x + 200;
			int y_ = y + 100;
			if (x_ < 0 || x_ >= dst.cols) continue;
			if (y_ < 0 || y_ >= dst.rows) continue;
			dst.at<uchar>(y_, x_) = src.at<uchar>(y, x);
		}
	}

	imshow("src", src);
	imshow("dst", dst);
	waitKey();
}
```

3. 영상의 전단변환
* 직사각형 형태의 영상을 한쪽방향으로 밀어서 평행사변형 모양으로 변형되는 변환. 층밀림 변환
* 가로 방향 또는 세로 방향으로 따로 정의됨
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
	Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

	if (src.empty()) {
		cerr << "Image load failed!" << endl;
		return -1;
	}

#if 1
	Mat dst(src.rows * 3 / 2, src.cols, src.type(), Scalar(0));

	double m = 0.5;
	for (int y = 0; y < src.rows; y++) {
		for (int x = 0; x < src.cols; x++) {
			int nx = x;
			int ny = int(y + m*x);
			dst.at<uchar>(ny, nx) = src.at<uchar>(y, x);
		}
	}
#else
	Mat aff = (Mat_<float>(2, 3) << 1, 0.5, 0, 0, 1, 0);

	Mat dst;
	warpAffine(src, dst, aff, Size(src.cols * 3 / 2, src.rows));
#endif

	imshow("src", src);
	imshow("dst", dst);

	waitKey();
}
```