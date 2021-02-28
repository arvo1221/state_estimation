# state_estimation
state estimate. observer, kalman filter, etc

## DC-Motor Full State Observer

-DC Motor + Controller + Full State Observer

Full state feedback(Pole Placement)를 적용한 system에 Full state observer를 추가
observer의 pole은 controller의 6배 크기로 설정하였음
process noise, measurement noise 는 gaussian noise
observer는 noise에 취약한 경향을 보여줌. pole을 수정하면 noise의 영향을 덜 받겠으나, 움직임이 느려질 것
일정 주기마다 이동 거리(x1) 데이터를 감소시키면 관측기의 거동이 커지며, 실제 움직임에서 크게 벗어나는 모습을 볼 수 있음

--noise를 입력하지 않음

![noise x](https://user-images.githubusercontent.com/54099930/109395502-1d4fd000-7970-11eb-9760-8408f8072539.jpg)

--process noise + measurement noise

![gaussian noise](https://user-images.githubusercontent.com/54099930/109395507-20e35700-7970-11eb-9e79-91d511b4b7fa.jpg)

--추가로 impulse noise 입력

![impulse](https://user-images.githubusercontent.com/54099930/109395510-2476de00-7970-11eb-98bb-7851dff371d3.jpg)


##DC-Motor Kalman Filter

-DC Motor + Controller + Kalman Filter

Full state feedback(Pole Placement)를 적용한 system에 Kalman filter 적용.
모터의 스펙과 제어기의 pole은  observer_motor.m과 동일하게 구현.
필터 구현을 위한 시스템 모델링에서 DC Motor의 electrical circuit modeling, mechanical system modeling은 theta가 아닌 omega를 기준으로 진행하였음 차분 식을 활용하여 matrix 유도. theta는 등속도 운동으로 가정하였음. 필터의 성능을 그래프로 확인하기 위해 process noise와 measurement noise가 없다고 가정한 시스템의 움직임도 계산함.
process noise, measurement noise 는 gaussian noise.
Kalman Filter는 gaussian noise에 강인한 모습을 보이나, x1에 impulse noise 를 입력하면 발산하는 모습을 보임.

--process noise와 measurement noise를 입력했을 때의 결과

![pic1](https://user-images.githubusercontent.com/54099930/109395209-7dde0d80-796e-11eb-9953-f566dbfcbf79.jpg) ![pic2](https://user-images.githubusercontent.com/54099930/109395212-7fa7d100-796e-11eb-9dde-05b79c14357d.jpg)
![pic3](https://user-images.githubusercontent.com/54099930/109395218-859db200-796e-11eb-91a1-10dac20ecc36.jpg)

--추가로 impulse noise를 입력했을 때의 결과

![pic4](https://user-images.githubusercontent.com/54099930/109395223-89313900-796e-11eb-9b7d-ef57ab257662.jpg)

