# OpenCV 편하게 사용하기

1. Imagewatch 확장 프로그램
* OpenCV Mat 데이터를 이미지 형태로 보여주는 Visual Studio 확장 프로그램
* OpenCV 프로그램 디버깅 시 유용함

* 설치방법
    + Visual Studio 2022 메뉴에서 [확장] -> [확장 관리] 선택
    + 우측 상단 검색창에 "opencv" 입력
    + Image Watch for Visual Studio 항목에서 [다운로드] 클릭
    + 재시작
    + 이후 디버깅을 하면 Imagewatch창이 출력

2. OpenCV 프로젝트 템플릿 만들기
* 프로젝트 템플릿
    + 프로젝트 속성, 기본 소스 코드 등이 미리 설정된 프로젝트를 자동으로 생성하는 기능
    + Visual Studio의 템플릿 내보내기 마법사를 통하여 ZIP 파일로 패키징된 자신만의 템플릿 파일을 생성할 수 있음

* OpenCV 프로젝트 템플릿
    + OpenCV 개발을 위한 추가 포함 디렉토리, 추가 라이브러리 디렉토리, 추가 종속성 등이 미리 설정되어 있는 콘솔 응용 프로그램 프로젝트를 생성
    + OpenCV 기본 소스 코드(main.cpp), 테스트 영상 파일(lenna.bmp)파일도 함께 생성됨

* OpenCV 템플릿 생성 순서
    1) OpenCVTemplate 이름의 프로젝트 생성
    + main.cpp 파일 추가 & 코드 작성
    + lenna.bmp 파일을 프로젝트에 추가
    + 프로젝트 속성에서 OpenCV 설정 추가-> 빌드 및 프로그램 동작 확인

    2) [프로젝트] -> [템프릿 내보내기...] 메뉴 선택
    + 기본 템프릿 형식 및 옵션 선택 후 [마침]

    3) C:\Users\dogu\Documents\Visual Studio 2022\Templates\ProjectTemplates\ 폴더에 있는 OpenCVTemplate.zip 파일을 수정
    + main.cpp& lenna.bmp 파일 추가
    + MyTemplate.vstemplate 파일 편집

    4) Visual Studio에서 새 프로젝트를 만들 때 해당 프로젝트 템플릿을 선택하여 사용