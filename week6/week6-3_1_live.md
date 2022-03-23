# week6-3 라이브 강의

1. 컬러 영상의 픽셀 값 참조
* 컬러 영상의 반전 직접 구현
    ```cpp
    void ColorOp1()
    {
        Mat src = imread("mandrill.bmp");
        
        Mat dst(src.rows, src.cols, CV_8UC3);

        for(int y=0; y<src.rows;y++){
            for(int x=0; x<src.cols;x++){
                dt.at<Vec3b>(y,x) = Vec3b(255,255,255) - src.at<Vec3b>(y,x);
            }
        }
        ...
    }
    ```

2. 컬러 영상을 그레이스케일로 변환
* Y = 0.299*R + 0.587*G + 0.114*B (약 3:6:1)
* 장점 : 데이터 저장 용량 감소, 연산처리속도 향상
* 단점 : 색상 정보 손실

```cpp
#define RGB2GRAY(r,g,b) (4899*r+ 9617*g+1868*b >>14)
//실수 연산을 없애고 나눗셈 대신 빠른 쉬프트 연산자 이용
```

2. 다양한 색 공간
* 색 공간 변환
    + 영상 처리에서는 특정한 목적을 위해 RGB 색 공간을 HSV,YCrCb, Lab 등의 다른 색 공간으로 변환하여 처리

* RGB 색공간
    + 빛의 삼원색인 빨간색, 녹생 파란색을 혼합하여 색상 표현
    + TV& 모니터 , 카메라 센서 Bayer 필터, 비트맵

* HSV 색공간
    + Hue : 색상, 색의 종류
    + Saturation : 채도, 색의 탁하고 선명한 정도
    + Value : 명도, 빛의 밝기
    + HSV 값 범위 : CV_8U 영상의 경우 (0<= H <=179, 0<= S <=255, 0<= V <= 255)

* YCrCb 색공간
    + PAL,NTSC,SECAM 등의 컬러 비디오 표준에 사용되는 색 공간
    + 영상의 밝기 정보와 색상 정보를 따로 분리하여 부호화(흑백 TV 호환)
    + Y : 밝기 정보(luma)
    + Cr, Cb : 색차(chroma)
    + YCrCb 값 범위 : CV_8U 영상의 경우 0<= Y,Cr,Cb <= 255

3. 히스토그램 평활화
* 직관적 방법 : R,G,B 각 색 평면에 대해 히스토그램 평활화
* 밝기 성분에 대해서만 히스토그램 평활화(YCrCb 의 Y값만)
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#define RGB2GRAY(r,g,b) ((4899*(r)+ 9617*(g)+1868*(b)) >>14)
using namespace std;
using namespace cv;

int main()
{
	//Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

	//if (src.empty()) {
	//	cerr << "Image laod failed!" << endl;
	//	return -1;
	//}

	//Mat dst;
	//cvtColor(src, dst, COLOR_GRAY2BGR);

	//circle(src, Point(200, 200), 100, Scalar(255, 0, 0), 2);
	//circle(dst, Point(200, 200), 100, Scalar(255, 0, 0), 2);


	//imshow("src", src);
	//imshow("dst", dst);
	//waitKey();


	Mat src = imread("lenna.bmp");

	Mat src_ycrcb;
	cvtColor(src, src_ycrcb,COLOR_BGR2YCrCb);

	vector<Mat> planes;

	split(src_ycrcb, planes);

	//bgr[0] = bgr[0] + 50;
	//bgr[1] = bgr[1] + 50;
	//bgr[2] = bgr[2] + 50;
	
	/*equalizeHist(bgr[0], bgr[0]);
	equalizeHist(bgr[1], bgr[1]);
	equalizeHist(bgr[2], bgr[2]);*/
	equalizeHist(planes[0],planes[0]);
	
	Mat dst_ycrcb;
	merge(planes, dst_ycrcb);

	Mat dst;
	cvtColor(dst_ycrcb, dst, COLOR_YCrCb2BGR);


	//Mat dst = src + Scalar(50,50,50);


#if 0
	Mat dst = Scalar(255, 255, 255) - src;
	Mat dst1;
	cvtColor(src, dst, COLOR_BGR2GRAY);
#else
	Mat dst1(src.rows, src.cols, CV_8UC1);
	Mat dst2(src.rows, src.cols, CV_8UC1);
	for (int y = 0; y<dst1.rows; y++) {
		for (int x = 0; x < dst1.cols; x++) {
			Vec3b v = src.at<Vec3b>(y, x);
			uchar b = v[0];
			uchar g = v[1];
			uchar r = v[2];

			/*Vec3b& p1 = src.at<Vec3b>(y, x);
			Vec3b& p2 = dst.at<Vec3b>(y, x);*/
			//p2[0] = 255 - p1[0];
			//p2[1] = 255 - p1[1];
			//p2[2] = 255 - p1[2];
			/*dst.at<Vec3b>(y, x) = Vec3b(255, 255, 255) - src.at<Vec3b>(y, x);*/
			//dst1.at<uchar>(y, x) = uchar((299*r + 587*g + 114*b)/1000);
			dst1.at<uchar>(y, x) = RGB2GRAY(r, g, b);

			dst2.at<uchar>(y, x) = (r+g+b)/3;


		}
	}

#endif
	
	imshow("src", src);
	//imshow("dst1", dst1);
	//imshow("dst2", dst2);
	imshow("dst", dst);
	waitKey();
}
```

