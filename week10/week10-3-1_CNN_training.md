# CNN에 적용할 수 있는 다양한 기법들

1. Optimizer
* SGD(Stochastic Gradient Descent)
* MGSD : SGD를 조금 더 Robust하게 추정(배치 단위로 경사하강법 진행)
* Momentum : 이전의 방향을 그대로 가져가는 기법
* AdaGrad : 기본개념은 learning rate를 유동적으로 변경하는것, 하지만 많이 움직인 element는 learning rate가 무한히 작아지는 문제가 발생함
* RMS-prop : 지수이동평균 개념을 도입 AdaGrad에서 learning rate가 무한히 작아지는 문제를 어느정도 해결
* Adam : RMS-prop + Momentum

2. Regularization
* overfitting을 방지하기 위한 규제
* L1 Regularization(강력한 규제)
  * 절대값 norm 사용, 절편은 규제하지 않음
  * Lasso
* L2 Regularization(부드러운 규제)
  * norm 제곱값 사용
  * Ridge

3. Drop out
* 랜덤한 weight를 지워서 overfitting을 피하는 기법
* 학습할때에만 사용

4. Batch normalization
* 활성함수에 안정적인 값을 넣고자 하는 아이디어에서 출발
* 배치 데이터의 mean, variance값을 사용