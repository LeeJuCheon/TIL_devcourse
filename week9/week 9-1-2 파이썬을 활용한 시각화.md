# Python에서의 시각화

1. Matplotlib 개요
* 데이터 시각화 라이브러리
* %matplotlib inline 명령어를 통해 활성화 해야함
  
2. 자주 사용되는 plotting의 Option
* figure = plotting을 할 도면을 선언
* plot = 실제 plot을 하는 함수
* title = 그래프 제목
* xlabel = x축 label명
* ylabel = y축 label명
* axis = x,y 축의 범위 설정
* xticks = x 눈금값 설정
* yticks = y 눈금값 설정
* legend = 범례 표시, label 값이 주어져야함
* show = plt를 확인


3. Matplotlib Case Study
* plot : 꺾은선 그래프, 시계열 데이터에서 효과적
* scatter : 산점도, 상관관계 혹은 분포 경향을 파악할 때 효과적
* boxplot : boxplot, 수치형 데이터에 대한 정보를 파악할 때 효과적(Q1, Q2, Q3, min, max) 
* bar : 막대형 그래프, 범주형 데이터에 대한 정보를 파악할 때 효과적
* hist : 히스토그램, 계급으로 나타냄, 범주형
* Pie : 원형그래프, 비율 확인에 용이하다


4. Seaborn Case Study
* kdeplot : 커널밀도그림, shade 값이 있음
* countplot : 범주형 columns의 빈도수 시각화 / groupby 후의 도수를 하는 것과 같다
* catplot : 숫자형 변수와 하나 이상의 범주형 변수의 관계를 보여주는 함수, 그래프의 형태가 아닌 묶는 방법이기 때문에 kind를 활용하여 strip, violin 등 다양한 형태로 바꿔줄 수 있다.
* stripplot : 스트립플롯, scatter plot과 유사하게 데이터의 수치를 표현하는 그래프
* heatmap : 히트맵, 상관계수에 유용하게 사용, 행렬을 색상으로 표현


> 새로 알게된 내용
> > cf) 라이브러리 : 개발자가 만든 것, 내부에 있는 코드들을 조합해서 결과를 내야함(ex. numpy, pandas,matplotlib 등)
> >      프레임워크 : 이미 틀이 짜여있는 곳에 내용물을 채워 결과물을 완성함(django, flask 등)


