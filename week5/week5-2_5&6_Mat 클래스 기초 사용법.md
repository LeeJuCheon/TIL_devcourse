# Mat 클래스 기초 사용법 (1)

> 전체 소스코드
```cpp

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void MatOp1();
void MatOp2();
void MatOp3();
void MatOp4();
void MatOp5();

int main()
{
	MatOp1();
	//MatOp2();
	//MatOp3();
	//MatOp4();
	//MatOp5();
}

void MatOp1()
{
	Mat img1; 	// empty matrix

	Mat img2(480, 640, CV_8UC1);		// unsigned char, 1-channel
	Mat img3(480, 640, CV_8UC3);		// unsigned char, 3-channels
	Mat img4(Size(640, 480), CV_8UC3);	// Size(width, height)

	Mat img5(480, 640, CV_8UC1, Scalar(128));		// initial values, 128
	Mat img6(480, 640, CV_8UC3, Scalar(0, 0, 255));	// initial values, red

	Mat mat1 = Mat::zeros(3, 3, CV_32SC1);	// 0's matrix
	Mat mat2 = Mat::ones(3, 3, CV_32FC1);	// 1's matrix
	Mat mat3 = Mat::eye(3, 3, CV_32FC1);	// identity matrix

	float data[] = {1, 2, 3, 4, 5, 6};
	Mat mat4(2, 3, CV_32FC1, data);

	Mat mat5 = (Mat_<float>(2, 3) << 1, 2, 3, 4, 5, 6);
	Mat mat6 = Mat_<uchar>({2, 3}, {1, 2, 3, 4, 5, 6});

	mat4.create(256, 256, CV_8UC3);	// uchar, 3-channels
	mat5.create(4, 4, CV_32FC1);	// float, 1-channel

	mat4 = Scalar(255, 0, 0);
	mat5.setTo(1.f);
}

void MatOp2()
{
	Mat img1 = imread("dog.bmp");

	Mat img2 = img1;
	Mat img3;
	img3 = img1;

	Mat img4 = img1.clone();
	Mat img5;
	img1.copyTo(img5);

	img1.setTo(Scalar(0, 255, 255));	// yellow

	imshow("img1", img1);
	imshow("img2", img2);
	imshow("img3", img3);
	imshow("img4", img4);
	imshow("img5", img5);

	waitKey();
	destroyAllWindows();
}

void MatOp3()
{
	Mat img1 = imread("cat.bmp");

	if (img1.empty()) {
		cerr << "Image load failed!" << endl;
		return;
	}

	Mat img2 = img1(Rect(220, 120, 340, 240));
	Mat img3 = img1(Rect(220, 120, 340, 240)).clone();

	img2 = ~img2;

	imshow("img1", img1);
	imshow("img2", img2);
	imshow("img3", img3);

	waitKey();
	destroyAllWindows();
}

void MatOp4()
{
	Mat mat1 = Mat::zeros(3, 4, CV_8UC1);

	for (int y = 0; y < mat1.rows; y++) {
		for (int x = 0; x < mat1.cols; x++) {
			mat1.at<uchar>(y, x)++;
		}
	}

	for (int y = 0; y < mat1.rows; y++) {
		uchar* p = mat1.ptr<uchar>(y);

		for (int x = 0; x < mat1.cols; x++) {
			p[x]++;
		}
	}

	for (MatIterator_<uchar> it = mat1.begin<uchar>(); it != mat1.end<uchar>(); ++it) {
		(*it)++;
	}

	cout << "mat1:\n" << mat1 << endl;
}

void MatOp5()
{
	float data[] = {1, 1, 2, 3};
	Mat mat1(2, 2, CV_32FC1, data);
	cout << "mat1:\n" << mat1 << endl;

	Mat mat2 = mat1.inv();
	cout << "mat2:\n" << mat2 << endl;

	cout << "mat1.t():\n" << mat1.t() << endl;
	cout << "mat1 + 3:\n" << mat1 + 3 << endl;
	cout << "mat1 + mat2:\n" << mat1 + mat2 << endl;
	cout << "mat1 * mat2:\n" << mat1 * mat2 << endl;
}
```

1. 영상의 생성과 초기화
* 소스코드
```cpp
void MatOp1()
{
	Mat img1; 	// empty matrix

	Mat img2(480, 640, CV_8UC1);		// unsigned char, 1-channel
	Mat img3(480, 640, CV_8UC3);		// unsigned char, 3-channels
	Mat img4(Size(640, 480), CV_8UC3);	// Size(width, height)

	Mat img5(480, 640, CV_8UC1, Scalar(128));		// initial values, 128
	Mat img6(480, 640, CV_8UC3, Scalar(0, 0, 255));	// initial values, red

	Mat mat1 = Mat::zeros(3, 3, CV_32SC1);	// 0's matrix
	Mat mat2 = Mat::ones(3, 3, CV_32FC1);	// 1's matrix
	Mat mat3 = Mat::eye(3, 3, CV_32FC1);	// identity matrix

	float data[] = {1, 2, 3, 4, 5, 6};
	Mat mat4(2, 3, CV_32FC1, data);

	Mat mat5 = (Mat_<float>(2, 3) << 1, 2, 3, 4, 5, 6);
	Mat mat6 = Mat_<uchar>({2, 3}, {1, 2, 3, 4, 5, 6});

	mat4.create(256, 256, CV_8UC3);	// uchar, 3-channels
	mat5.create(4, 4, CV_32FC1);	// float, 1-channel

	mat4 = Scalar(255, 0, 0);
	mat5.setTo(1.f);
}

```

2. 영상의 참조와 복사
* Mat 클래스 객체의 참조와 복사
    + Mat 객체에 대해 = 연산자는 참조(얕은 복사)를 수행
    + Mat::clone() 또는 Mat::copyTo() 함수를 이용하여 깊은 복사를 수행

* 소스코드
```cpp
void MatOp2()
{
	Mat img1 = imread("dog.bmp");

	Mat img2 = img1;    // 원본과의 동기화, 얕은복사
	Mat img3;
	img3 = img1;

	Mat img4 = img1.clone();    //독립적인 복사, 깊은복사
	Mat img5;
	img1.copyTo(img5);

	img1.setTo(Scalar(0, 255, 255));	// yellow

	imshow("img1", img1);
	imshow("img2", img2);
	imshow("img3", img3);
	imshow("img4", img4);
	imshow("img5", img5);

	waitKey();
	destroyAllWindows();
}

```

3. 부분영상 추출
* Mat 클래스 객체에서 부분 행렬 추출
	+ Mat 객체에 대해 () 연산자를 이용하여 부분 영상 추출 가능
	+ () 연산자 안에는 Rect 객체를 지정하여 부분 영상의 위치와 크기를 지정
	+ 참조를 활용하여 ROI(Region of Interest) 연산 수행 가능

* 소스코드
```cpp
void MatOp3()
{
	Mat img1 = imread("cat.bmp");

	if (img1.empty()) {
		cerr << "Image load failed!" << endl;
		return;
	}

	Mat img2 = img1(Rect(220, 120, 340, 240));
	Mat img3 = img1(Rect(220, 120, 340, 240)).clone();

	img2 = ~img2;

	imshow("img1", img1);
	imshow("img2", img2);
	imshow("img3", img3);

	waitKey();
	destroyAllWindows();
}

```

4. 영상의 픽셀값 참조
* Mat 영상의 픽셀 값 접근하기
	+ OpenCV에서 제공하는 기능이 아니라 자신만의 새로운 기능을 추가해야 하는 경우에 유용
	+ 기본적으로 Mat::data 멤버 변수가 픽셀 데이터 메모리 공간을 가리키지만, Mat 클래스 멤버 함수를 사용하는 방법을 권장
		- Mat::data 사용방법 : 메모리 연산이 잘못될 경우 프로그램이 비정상 종료할 수 있음
		- Mat::at() 함수 사용 방법 : 좌표 지정이 직관적, 임의 좌표에 접근할 수 있음.
		- Mat::ptr() 함수 사용 방법 : Mat::at()보다 빠르게 동작, 행 단위 연산을 수행할 때 유리
		- Matiterator_반복자 사용 방법 : 좌표를 지정하지 않아서 안전, 성능은 느린편
* Mat::at() 사용방법
	```cpp
	template<typename _Tp> _Tp& Mat::at(int y,int 	x)
	```
	+ y : 참조할 행 번호
	+ x : 참조할 열 번호
	+ 반환값 : (_Tp&타입으로 캐스팅된) y행 x열 원소 값(참조)

* Mat::ptr() 사용방법
	```cpp
	template<typename _Tp> _Tp& Mat::ptr(int y)
	```
	+ y : 참조할 행 번호
	+ 반환값 : (_Tp&타입으로 캐스팅된) y번 행의 시작 주소

* MatIterator <T> 반복자 사용방법
	+ OpenCV는 Mat 클래스와 함께 사용할 수 있는 반복자 클래스 템플릿 MatIterator_를 제공
	+ MatIterator_는 클래스 템플릿이므로 사용할 때에는 Mat 행렬 타입에 맞는 자료형을 명시해야함
	+ Mat::begin() 함수는 행렬의 첫 번째 원소 위치를 반환
	+ Mat::end() 함수는 행렬의 마지막 원소 바로 다음 위치를 반환

* 사용예제 코드
```cpp.
void MatOp4()
{
	Mat mat1 = Mat::zeros(3, 4, CV_8UC1);

	for (int y = 0; y < mat1.rows; y++) {
		for (int x = 0; x < mat1.cols; x++) {
			mat1.at<uchar>(y, x)++;
		}
	}

	for (int y = 0; y < mat1.rows; y++) {
		uchar* p = mat1.ptr<uchar>(y);

		for (int x = 0; x < mat1.cols; x++) {
			p[x]++;
		}
	}

	for (MatIterator_<uchar> it = mat1.begin<uchar>(); it != mat1.end<uchar>(); ++it) {
		(*it)++;
	}

	cout << "mat1:\n" << mat1 << endl;
}
```

5. 기초 행렬 연산
* 소스코드
```cpp
void MatOp5()
{
	float data[] = {1, 1, 2, 3};
	Mat mat1(2, 2, CV_32FC1, data);
	cout << "mat1:\n" << mat1 << endl;

	Mat mat2 = mat1.inv();
	cout << "mat2:\n" << mat2 << endl;

	cout << "mat1.t():\n" << mat1.t() << endl;
	cout << "mat1 + 3:\n" << mat1 + 3 << endl;
	cout << "mat1 + mat2:\n" << mat1 + mat2 << endl;
	cout << "mat1 * mat2:\n" << mat1 * mat2 << endl;
}
```