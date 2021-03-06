# 인공지능과 기계학습

* 인공지능의 정의 : 인간의 학습, 추론, 지각, 자연언어 이해 등의 지능적 능력을 기기로 실현한 기술

* 인공지능은 과거의 규칙적인 것(Rule-based)에 한정되었다면, 최근에는 학습(Learning)에 중점을 두었다.

> 1. 1940 - 1960 : 사람을 최대한 모방하자 / 인공지능의 시작을 알린 촉발 시대
> 2. 1980 - 2000 : SVM, 다중 레이어 퍼셉트론의 개념 정립, 딥러닝 기술의 개념도 등장했지만 따라오지 못하는 하드웨어 스펙 등으로 실용화되지 못함
> 3. 2010 ~ : 하드웨어의 기능이 발전함에 따라 과거의 기술, 특히 딥러닝 기술의 대중화를 이끌어 심층학습의 혁신을 이룸


* 지식기반 -> 기계학습 -> 심층학습(표현학습)
    + 지식기반에서 학습에 중점을 둔 방식으로 변화
    + Rule-based -> Classic machine Learning -> Representation learning

* 주요 프레임워크 : pytorch, tensorflow

1. 기계학습의 개념

* 가설 정의 : 데이터 양상을 파악하고 임의로 모델을 선택

* 훈련(Train) : 주어진 문제인 예측을 가장 정확하게 할 수 있는 최적의 매개변수(parameter)를 찾는 작업

* 훈련집합(Training set)
    + x = 특징, y = 목표치

* 새로운 특징에 대응되는 목표치의 예측에 사용

* 최종 목표 : Test set에 대한 오류의 최소화 

* 데이터셋의 차원은 다양하기 때문에 벡터를 활용하여 연산

* 차원의 저주 : 차원의 높아짐에 때라 발생하는 문제

* 선형 분리 불가능한 공간을 공간변환을 통해 풀이할 수 있음(ex. manifold, t-SNE)


* 딥러닝
    + 표현학습의 하나로 다수의 은닉층을 가진 신경망을 이용하여 최적의 계층적인 특징을 학습

* 약인공지능 -> 강인공지능 -> 초인공지능
    + 현재는 약인공지능
    + Weak AI : 인간이 지시한 명령의 룰 안에서만 말하기 떄문에 예측과 관리가 용이
    + Strong AI : 인간이 할 수 있는 어떠한 지적인 업무도 성공적으로 해낼 수 있는 기계의 지능
    + Super AI : 인공지능의 발전이 가속화되어 모든 인류의 지성을 합친 것보다 더 뛰어난 인공지능

* Machine Learning은 데이터를 설명할 수 있는 학습 모델을 찾아내는 과정
    + Raw Data 수집 -> Clean Data -> Build Model -> Predict

* 희소데이터에 효과적인 기법

* 시각화가 불가능한 초평면을 시각화하는 기법

2. 기초 머신러닝 example
* 선형회귀 : 직선모델을 활용한 두개의 parameter 조정
* 목적함수(비용함수) : 선형회귀의 오차, 해당값을 최소화하는 방향으로 예측값을 향상시킴(MSE, MAE, RMSE 등)

* 차원을 축소하기 위해 Feature Selection 기법을 사용하기도 함

* 선형 모델은 underfitting의 확률이 높음
* 하지만 너무 높은 차수의 모델은 overfitting을 유발할 수 있음 -> train set은 잘 예측하지만 test set에 대해서 잘 예측하지 못함
  
* 일반적으로 용량이 작은 모델은 bias가 크고 variance가 작음

* 일반적으로 용량이 큰 모델은 bias가 작고 variance가 큼

* bias와 variance는 trade-off 관계
  
* 최종목표는 bias와 variance 모두 낮게 만드는 것이 목표이기 때문에 편향을 최소로 유지하며 분산도 최대로 낮추는 전략이 필요

* 하이퍼파라미터를 조정하기 위한 validation set을 따로 설정


* Cross-Validation : validation set을 fold로 나누어 교차적으로 검증

* bootstrap : 복원 추출 샘플링, 데이터 분포가 불균형일 때 유용(ex.anomaly detection)


* <b>현대 기계 학습의 전략은 용량이 충분히 큰 모델을 선택한 후, 선택한 모델이 정상을 벗어나지 않도록 규제 적용</b>

* 모델 능력 향상 방법
    + 가장 좋은 방법 : 데이터의 크기를 충분히 늘린다
        - But 막대한 비용
    + 따라서 Data augmentation을 활용하여 데이터를 변형(rotation, warping)함

    + 개선된 목적함수를 사용하여 가중치를 작게 조절하는 규제(L1 규제, L2 규제)

* 지도학습
    + Target 값이 있는 경우
    + 회귀 / 분류 문제
* 비지도 학습
    + Target이 없는 경우
    + Clustering
    + 밀도 추정, 특징 공간 변환(ex.PCA)

* 강화학습
    + 목표치가 주어지는데, 지도 학습과 다른 형태인 보상을 부여
  
* 준지도 학습
    + Target 값이 일부만 있는 형상
    + 데이터 Annotation이 수작업으로 많이 이루어지는 현재 매우 중요한 상황


> 궁금한 내용
> >정보이론?