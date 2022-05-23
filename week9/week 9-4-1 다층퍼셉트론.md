# 다층 퍼셉트론

1. 신경망의 다양한 모델
* forwaard 신경망
* recurrent 신경망
* shallow 신경망
* deep 신경망
* deterministic 신경망
* stochastic 신경망

2. 퍼셉트론
* node, weight, layer 개념의 도입
* 학습 알고리즘을 제안
* 구조
  * 입력값 + 편향 b로 이루어짐
  * 입력과 출력 사이에 연산하는 구조(가중치 w)
  * 출력은 한개의 노드로 인해 -1, 1 출력  
* 퍼셉트론은 선형분류기로써 선형분리가 불가능한 상황에서 일정한 양의 오류가 검출됨

3. 다층 퍼셉트론
* hidden layer를 추가하여 새로운 특징 공간으로 변환함(representation learning)
* 시그모이드 활성함수를 도입함
  * step function의 딱딱한 결정에서 부드러운 의사결정으로 변경(영역 -> 점 에서 영역->영역으로 변경)
  * 시그모이드에서 알파 값이 무한대로 갈수록 계단함수에 가까워짐.
  * 일반적으로 hidden layer에서는 logistic sigmoid 활성함수를 사용
  * S자 모양의 넓은 포화곡선은 gradient 기반 학습(back propagation)을 어렵게 함(gradient banish 현상)
    * 심층학습에서는 ReLU를 사용

* 오류 역전파 알고리즘을 사용
* 용량(capacity) : p개의 퍼셉트론을 결합하면 p차원 공간으로 변환

* 구조 : 입력 - 은닉층(여러 층 가능) - 출력의 구조
  * d+1개의 입력 노드(d는 특징)
  * p개의 은닉노드(hyperparameter) 
    * p가 너무 크면 오버피팅, 너무 적으면 언더피팅
  * 2층 구조로 가정했을 때, 입력-은닉층 사이의 행렬, 은닉층 - 출력 사이의 행렬 두개가 존재(U행렬 혹은 w 가중치로 표기)
* 동작 : 특징 벡터 x를 출력 벡터 o로 사상(mapping) 하는 함수로 간주할 수 있음
  * 은닉층은 특징 벡터를 분류에 더 유리한 새로운 특징 공간으로 변환

* 범용적 근사이론
  * 하나의 은닉층은 함수의 근사를 표현

* 얕은 은닉층의 구조
  * 지수적으로 더 넓은 폭(Width)이 필요할 수 있음
  * 하지만 오버피팅이 되기 쉬움
  * 일반적으로 Depth가 높은 hidden layer가 좋은 성능을 가짐 

3. 뉴럴 네트워크 경험적 개발에서 중요 쟁점
* 아키텍처 : 은닉층, 은닉 노드의 개수 / 적절한 규제 기법을 적용해야 함
* 초기값 : 보통의 경우 랜덤값을 부여하는데 값의 범위와 분포를 설정해주어야 함
* 학습률(learning rate) : 학습률이 낮으면 최적값을 찾는 속도가 더디고, 학습률이 크면 최적해에 도달하지 못하는 경우가 생긴다. 고정값을 사용하는 기법, 큰 값에서 작은 값으로 점점 줄이는 적응적 방식 등 다양한 설정 방법이 있음
* 활성함수 : 초창기 다층 퍼셉트론에는 tanh, 시그모이드 함수사용, 최근 딥러닝에서는 ReLU 함수를 주로 사용

4. 오류 역전파 알고리즘(Back propagation Algorithm)
* 목적함수
  * MSE(평균제곱오차)
    * 딥러닝에서는 온라인모드와 배치모드로 나뉨
    * 
* Chain rule 구현
  * 반복되는 부분식을 DP를 활용하여 저장하거나 재연산을 최소화

* 오류 역전파의 유도
  * 출력의 오차를 역방향으로 전파하여 gradient를 계산하는 알고리즘
  * 반복되는 부분식 gradient의 지수적 폭발 혹은 vanishing을 피해야 함
* 역전파 분해(backprop with scalars)
* 역전파 주요 예
  * 덧셈 : gradient distributor
    * 구해진 gradient 같은 수치로 분배
  * 곱셉 : swap multiplier
    * 구해진 gradient에서 반대편과 곱함
  * copy : gradient adder
    * 각각의 gradient를 더함
  * max : gradient router
    * max값에 gradient 부여, 아닌쪽은 0
  * sigmoid
    * output gradient * sigmoid의 미분값
* 도함수(derivative)의 종류
  * scalar to scalar : derivative
  * vector to scalar : gradient
  * vector to vector : jacobian

> 다층 퍼셉트론 시각화 링크
> https://playground.tensorflow.org
> Q1 : 왜 GPU는 행렬 계산에 특화되어 있을까?
