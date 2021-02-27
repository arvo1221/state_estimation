clear all
clc

dt = 0.0001;

%DC Motor spec
Jm = 95.2 * 10^-7;
b = 2.539 * 10^-3;
Ra = 2.16;
La = 7.76 * 10^-4;
Ke = 74.1 * 10^-3;
Kt = Ke;

%State Space
%{
Equations of motion
Jm*w_dot + b*w = Kt*ia
La*ia_dot + Ra*ia + Ke*w = va
%}
%State variable form
%x:=[theta theta_dot ia]^T, u:=va, y=theta
F = [0 1 0; 0 -b/Jm Kt/Jm; 0 -Ke/La -Ra/La];
G = [0 ;0 ;1/La];
H = [1 0 0];
J = [0];

%{
desired spec
rising time 0.5sec
settling time 1.8sec
overshoot 4.22%
-> wn = 3.6 zeta = 0.7099
pole s = -zeta*wn +- j*wn*sqrt(1-zeta^2)
%}
wn = 3.6; zeta = 0.7099;
s1 = -zeta*wn + i*wn*sqrt(1-zeta^2)
s2 = -zeta*wn - i*wn*sqrt(1-zeta^2)
s3 = 10*real(s1)
c_s = [s1 s2 s3];

%gain K using Ackerman's formula
C = [G F*G F^2*G];
syms f;
a_c = sym2poly((f-c_s(1))*(f-c_s(2))*(f-c_s(3)))
a_cF = F^3 + a_c(1)*F^2 + a_c(2)*F + a_c(3)*eye(3,3)
K = [0 0 1]*inv(C)*a_cF
%use function acker
K = acker(F,G,c_s)
%eig(F-G*K)

%N_bar
N = inv([F G ; H J])*[0;0;0;1]
Nx = [N(1);N(2);N(3)];
Nu = [N(4)];
N_bar = K*Nx + Nu;

r = 1;
syms x1 x2 x3;
x1 = 0; x2 = 0; x3 = 0;
x = [x1; x2; x3]
%{
%plot the resulting unit step response
for j = 0:30000
    u = r*N_bar-K*x;
    x_dot = F*x + G*u;
    x = x + x_dot*0.0001;
    y = H*x;
    Y(:,j+1) = y;
end
t= 0:dt:3;
plot(t,Y);
grid on;
%}

%observable?
O = [H ; H*F ; H*F^2];
det(O)

%gain L using Ackerman's formula
e_s = 6*c_s;
C = [G F*G F^2*G];
a_e = sym2poly((f-e_s(1))*(f-e_s(2))*(f-e_s(3)))
a_eF = F^3 + a_e(1)*F^2 + a_e(2)*F + a_e(3)*eye(3,3)
L = a_eF*inv(O)*[0;0;1]
%use function acker 
L = (acker(F',H',e_s))'

syms x1_hat x2_hat x3_hat;
x1_hat = 0; x2_hat = 0; x3_hat = 0;
x_hat = [x1_hat; x2_hat; x3_hat]

%{
%plot the resulting unit step response contain observer
for j = 0:30000
    u = r*N_bar-K*x;
    x_dot = F*x + G*u;
    x = x + x_dot*dt;
    y = H*x;
    Y(:,j+1) = y;
    y_tilde = y - H*x_hat;
    x_hat_dot = F*x_hat + G*u + L*y_tilde;
    x_hat = x_hat + x_hat_dot * dt;
    y_hat = H*x_hat;
    Y_hat(:,j+1) = y_hat;
end
t= 0:dt:3;
plot(t,Y);
grid on;
hold on;
plot(t,Y_hat);
%}

%plot the resulting unit step response + process noise + measurement noise
gaussian_noise_w = 0.005*randn(1,30001)*sqrt(4);
gaussian_noise_m = 0.001*randn(1,30001);
for j = 0:30000
    u = r*N_bar-K*x;
    if(mod(j,10000) < 1)  
        x_dot = F*x + G*u + gaussian_noise_w(j+1);
        x(1) = x(1) - 0.001;
    else
        x_dot = F*x + G*u + gaussian_noise_w(j+1);
    end
    x = x + x_dot*dt;
    y = H*x + gaussian_noise_m(j+1);
    Y(:,j+1) = y;
    x_hat_dot = F*x_hat + G*u + L*(y - H*x_hat);
    x_hat = x_hat + x_hat_dot * dt;
    y_hat = H*x_hat;
    Y_hat(:,j+1) = y_hat;
    Y2(:,j+1) = [0 1 0]*x;
    Y3(:,j+1) = [0 0 1]*x;
    Y2_hat(:,j+1) = [0 1 0]*x_hat;
    Y3_hat(:,j+1) = [0 0 1]*x_hat;
    U(:,j+1) = u;
end
t= 0:dt:3;
%plot(t,U)

subplot(3,1,1);
plot(t,Y);
grid on;
hold on;
plot(t,Y_hat);
legend('origin','observer')
subplot(3,1,2);
plot(t,Y2);
grid on;
hold on;
plot(t,Y2_hat);
subplot(3,1,3);
plot(t,Y3);
grid on;
hold on;
plot(t,Y3_hat);