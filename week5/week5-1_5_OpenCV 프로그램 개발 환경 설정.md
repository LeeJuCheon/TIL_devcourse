# OpenCV 프로그램 개발 환경 설정

1. HelloCV 프로젝트 만들기
* 새 프로젝트 만들기 -> 빈 프로젝트 -> 솔루션 및 프로젝트를 같은 디렉토리에 배치 체크

```cpp
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
int main()
{
	cout << "Hello,OpenCV" << CV_VERSION << endl;
}
```
* OpenCV 헤더 파일 위치 지정
    * [프로젝트] -> [HelloCV 속성]
    * [구성 속성] -> [C/C++] -> [일반]
    * 추가 포함 디렉터리에 $(OPENCV_DIR)\include 입력

* OpenCV LIB 파일 위치 지정
    * [프로젝트] -> [HelloCV 속성]
    * [구성 속성] -> [링커] -> [일반]
    * 추가 라이브러리 디렉터리에 $(OPENCV_DIR)\x64\vc15\lib 입력

* OpenCV LIB 파일 이름 지정
    * [프로젝트] -> [HelloCV 속성]
    * [구성 속성] -> [링커] -> [입력]
    * 추가 종속성 항목에 opencv_world455d.lib 추가(release 일시 opencv_world455.lib)

-> 해당 내용은 절대경로로도 가능