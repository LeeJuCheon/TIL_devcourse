# 영상 파일 형식과 특징

> 비트맵
* 비트들의 집합 == 픽셀의 집합
* 영상의 전체 크기에 해당하는 픽셀 정보를 그대로 저장
    + 장점: 표현이 직관적이고 분석이 용이
    + 단점 : 메모리 용량을 많이 차지, 영상의 확대/축소시 화질 손상이 심함
* 사진, 포토샵
* 장치 의존 비트맵(DDB)
    + 출력장치(화면,프린터 등)의 설정에 따라 다르게 표현됨
* 장치 독립 비트맵(DIB)
    + 출력 장치가 달라지더라도 항상 동일하게 출력됨
    + BMP 파일은 Windows 환경에서 비트맵을 DIB 형태로 저장한 파일 포맷
<br>

> 벡터 그래픽스
* 점과 점을 연결해 수학적 원리로 그림을 그려 표현하는 방식
* 이미지의 크기를 확대 또는 축소해도 화질이 손상되지 않음
* 폰트, 일러스트레이터
<br>

1. 비트맵 개요

* BMP 파일의 구조
    + 비트맵 파일 헤더
        - 비트맵 파일에 대한 정보
        ```cpp
        typedef struct tagBITMAPFILEHEADER{
            WORD bfType;    //'B','M',0x42,0x4D
            DWORD bfSize;   //BMP 파일 크기
            WORD bfReserved1;
            WORD bfReserved2;
            DWORD bfOffBits;    // 비트맵 비트까지의 오프셋
        }BITMAPFILEHEADER;
        ```
        - WORD : 2Bytes, DWORD : 4Bytes, 총 14Bytes

    + 비트맵 정보 헤더
        - 비트맵 영상에 대한 정보
        ```cpp
        typedef struct tagBITMAPINFOHEADER{
            DWORD biSize;       //BITMAPINFOHEADER 크기
            LONG biWidth;       // 비트맵 가로 크기
            LONG biHeight;      // 비트맵 세로 크기
            WORD biPlanes;      //1
            WORD biBitCount;    //픽셀 당 비트 수
            DWORD biCompression;//BI_RGB
            DWORD biSizeImage;
            LONG biXpelsPerMeter;
            LONG biYpelsPerMeter;
            DWORD biClrUsed;
            DWORD biClrImportant;
        }BITMAPINFOHEADER;
        ```
        - 총 40Bytes
        
    + 색상 테이블/팔레트
        - 비트맵에서 사용되는 색상 정보
        ```cpp
        typedef struct tagRGBQUAD{
            BYTE rgbBlue;
            BYTE rgbGreen;
            BYTE rgbRed;
            BYTE rgbReserved;
        }RGBQUAD;
        ```
        - 총 4Bytes

    + 픽셀 데이터
        - 그레이스케일 비트맵 : RGBQUAD 배열의 인덱스 저장
        - 트루컬러 비트맵 : (BGR) 순서로 픽셀값 저장
        - 일반적으로 상하가 뒤집힌 상태로 저장됨
        - 효율적 데이터 관리를 위해 영상의 가로크기를 4의 배수로 저장

2. BMP 파일 출력 프로그램 만들기
* BmpShow 프로그램
    + 마우스를 클릭하면 해당 위치에 cat.bmp파일에 들어있는 비트맵을 출력하는 프로그램
    + Windows 프로그램의 기본 동작 방식을 이해
    + BMP 파이을 디코딩하여 DIB를 출력하는 코드를 이해

* 프로젝트 만드는 방법
    + windows 데스크톱 애플리케이션으로 실행
    + 솔루션 및 프로젝트를 같은 디렉터리에 배치

* WM_LBUTTONDOWN case 추가
```cpp
case WM_LBUTTONDOWN:
    {
        FILE* fp = NULL;
        fopen_s(&fp, "cat.bmp", "rb");  //cat.bmp open

        if (!fp)
            break;

        BITMAPFILEHEADER bmfh;          //fileheader
        BITMAPINFOHEADER bmih;          //infoheader

        fread(&bmfh, sizeof(BITMAPFILEHEADER), 1, fp);      //read
        fread(&bmih, sizeof(BITMAPINFOHEADER), 1, fp);      //read

        LONG nWidth = bmih.biWidth;
        LONG nHeight = bmih.biHeight;
        WORD nBitCount = bmih.biBitCount;

        DWORD dwWidthStep = (DWORD)((nWidth * nBitCount / 8 + 3) & ~3); //nWidth*nBitCount/8+3 보다 같거나 큰 4의배수값을 계산하는 코드
        DWORD dwSizeImage = nHeight * dwWidthStep;

        DWORD dwDibSize;
        if (nBitCount == 24)
            dwDibSize = sizeof(BITMAPINFOHEADER) + dwSizeImage;         //트루컬러일때
        else
            dwDibSize = sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * (1 << nBitCount) + dwSizeImage;    //그레이스케일이면 팔레트색상의 크기도 추가

        BYTE* pDib = new BYTE[dwDibSize];   //동적할당

        fseek(fp, sizeof(BITMAPFILEHEADER), SEEK_SET);     //픽셀데이터 위치까지 이동 
        fread(pDib, sizeof(BYTE), dwDibSize, fp);

        LPVOID lpvBits;             // 실제 픽셀데이터가 나타나는 위치의 주소값
        if (nBitCount == 24)
            lpvBits = pDib + sizeof(BITMAPINFOHEADER);
        else
            lpvBits = pDib + sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * (1 << nBitCount);

        HDC hdc = GetDC(hWnd);
        int x = LOWORD(lParam);             //실제 마우스 커서 위치
        int y = HIWORD(lParam);
        ::SetDIBitsToDevice(hdc, x, y, nWidth, nHeight, 0, 0, 0, nHeight, lpvBits,
            (BITMAPINFO*)pDib, DIB_RGB_COLORS);
        ReleaseDC(hWnd, hdc);

        delete[] pDib;
        fclose(fp);
    }
```

3. 영상 파일 형식과 특징

    1) BMP
    * 픽셀 데이터를 압축하지 않고 그대로 저장하여 용량이 큼
    * 파일 구조가 단순해서 별도의 라이브러리 도움 없이 파일 입출력 프로그래밍 가능
    
    2) JPG
    * 주로 사진과 같은 컬러 영상을 저장
    * 손실 압축(lossy compression)
    * 압출륙이 좋아서 파일용량이 크게 감소 -> 디지털 카메라 사진 포맷으로 주로 사용

    3) GIF
    * 256 색상 이하의 영상을 저장 ->일반 사진을 저장 시 화질 열화가 심함
    * 무손실 압축(lossless compression)
    * 움직이는 GIF 지원

    4) PNG
    * Portable Network Graphics
    * 무손실 압축(컬러영상도 무손실 압축)
    * 알파 채널(투명도)을 지원

* 저주파 성질이 강한 영상 : 주변의 색상의 차이가 적은 영상, 압축이 잘됨
* 고주파 성질이 강한 영상 : 주변의 색상의 차이가 빈번한 영상, 압축이 잘 되지 않음 