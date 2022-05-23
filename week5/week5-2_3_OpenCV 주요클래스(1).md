# OpenCV 주요 클래스(1)

1. Point_ 클래스
* 2차원 점의 좌표 표현을 위한 템플릿 클래스
* 멤버 변수 : x,y
* 멤버 함수 : dot(),ddot(), cross(), inside() 등
* 다양한 사칙 연산에 대한 연산자 오버로딩과 std::cout 출력을 위한 << 연산자 오버로딩을 지원
* 코드 정의
```cpp
template<typename _Tp> class Point_
{
    public:
    ...
    TP x,y; //< the point coordinates
};

typedef Point_<int> Point2i;
typedef Point_<int64> Point2l;
typedef Point_<float> Point2f;
typedef Point_<double> Point2d;
typedef Point2i Point;
```

2. Size_ 클래스
* 영상 또는 사각형의 크기 표현을 위한 템플릿 클래스
* 멤버 변수 : width, height
* 멤버 함수 : area()
* 다양한 사칙 연산에 대한 연산자 오버로딩과 std::cout 출력을 위한 << 연산자 오버로딩을 지원
* 코드 정의
```cpp
template<typename _Tp> class Size_{
    public:
        ...
        _Tp width, height;  //the width and the height
};

typedef Size_<int> Size2i;
typedef Size_<int64> Size2l;
typedef Size_<float> Size2f;
typedef Size_<double> Size2d;
typedef Size2i Size;
```

3. Rect_ 클래스
* 2차원 사각형 표현을 위한 템플릿 클래스
* 멤버 변수 : x, y, width, height
* 멤버 함수 : tl(),br(),size(), area(), contains()
* 다양한 사칙 연산에 대한 연산자 오버로딩과 std::cout 출력을 위한 << 연산자 오버로딩을 지원
* 코드 정의
```cpp
template<typename _Tp> class Rect_{
    public:
        ...
        _Tp x, y, width, height; 
};

typedef Rect_<int> Rect2i;
typedef Rect_<int64> Rect2l;
typedef Rect_<float> Rect2f;
typedef Rect_<double> Rect2d;
typedef Rect2i Size;
```

3. Rect_ 클래스
* 2차원 사각형 표현을 위한 템플릿 클래스
* 멤버 변수 : x, y, width, height
* 멤버 함수 : tl(),br(),size(), area(), contains()
* 다양한 사칙 연산에 대한 연산자 오버로딩과 std::cout 출력을 위한 << 연산자 오버로딩을 지원
* 코드 정의
```cpp
template<typename _Tp> class Rect_{
    public:
        ...
        _Tp x, y, width, height; 
};

typedef Rect_<int> Rect2i;
typedef Rect_<int64> Rect2l;
typedef Rect_<float> Rect2f;
typedef Rect_<double> Rect2d;
typedef Rect2i Size;
```

4. Range 클래스
* 정수 값의 범위를 나타내기 위한 클래스
* 멤버 변수 : start, end
* 멤버 함수 : size(),empty(),all()
* start는 범위에 포함되고, end는 범위에 포함되지 않음 : [start, end)

* 코드 정의
```cpp
class Range{
    public:
        Range();
        Range(int _start, int _end);
        int size() const;
        bool empty() const;
        static Range all();
        
        int start,end;
}
```

5. String 클래스
* 원래 OpenCV에서 자체적으로 정의하여 사용하던 문자열 클래스였으나 OpenCV 4.x 버전 부터 std::string 클래스로 대체됨
```cpp
typedef std::string cv::String;
```
* cv::format()함수를 이용하여 형식 있는 문자열 생성 가능 -> C언어의 printf()함수와 인자 전달방식이 유사함

6. Vec 클래스
* 벡터는 같은 자료형 원소 여러개로 구성된 데이터 형식
* Vec클래스는 벡터를 표현하는 템플릿 클래스
* std::cout 출력을 위한 << 연산자 오버로딩을 지원

* 코드 정의
```cpp
template<typename _Tp, int m, int n> class Matx{
    public:
        ...
        _Tp val[m*n];   //< matrix elements
};
template<typename _Tp,int cn> class Vec : public Matx<_Tp, cn, 1>{
    public:
        const _Tp& operator [](int i) const;
        _Tp& operator[](int i);
};
```

* Vec 클래스 이름 재정의
    + 자주 사용되는 자료형과 개수에 대한 Vec 클래스 템플릿의 이름 재정의
    + 형식 : Vec<num-of-data>{b|s|w|i|f|d}
    //b : unsigned char, s : short, w : unsigned short, i : int, f : float, d : double 

7. Scalar 클래스
* 크기가 4인 double 배열(double val[4])을 멤버 변수로 가지고 있는 클래스 
* 4채널 이하의 영상에서 픽셀 값을 표현하는 용도로 자주 사용
* [] 연산자를 통해 원소에 접근 가능

* 코드 정의
```cpp
template<typename _Tp> class Scalar_ : public Vec<_Tp, 4>{
    public:
        Scalar_();
        Scalar_(_Tp v0, _Tp v1, _Tp v2 =0, _Tp v3=0);
        Scalar_(_Tp v0);

        static Scalar_<_Tp> all(_Tp v0);
        ...
        
};

typedef Scalar_<double> Scalar;
```