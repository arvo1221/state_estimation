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
A = [1 dt 0 ; 0 0.9689 0.5899 ; 0 -0.0073 0.7779];
G = [0 ;0 ;1/La];
B = [0 ;0.0760 ;0.1002];
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
s1 = -zeta*wn + i*wn*sqrt(1-zeta^2);
s2 = -zeta*wn - i*wn*sqrt(1-zeta^2);
s3 = 10*real(s1);
c_s = [s1 s2 s3];

%gain K using Ackerman's formula
C = [G F*G F^2*G];
syms f;
a_c = sym2poly((f-c_s(1))*(f-c_s(2))*(f-c_s(3)));
a_cF = F^3 + a_c(1)*F^2 + a_c(2)*F + a_c(3)*eye(3,3);
K = [0 0 1]*inv(C)*a_cF;
%use function acker
K = acker(F,G,c_s)
%eig(F-G*K)

%N_bar
N = inv([F G ; H J])*[0;0;0;1];
Nx = [N(1);N(2);N(3)];
Nu = [N(4)];
N_bar = K*Nx + Nu;

r = 1;
syms x1 x2 x3;
x1 = 0; x2 = 0; x3 = 0;
x = [x1; x2; x3];

%use kalman filter
%Q = [0.009 0 0 ; 0 0.001 0 ; 0 0 0.008];
Q = 10*[(10^(-24))/16 (10^(-20))/8 (10^(-16))/4;(10^(-20))/8 (10^(-16))/4 (10^(-12))/2 ; (10^(-16))/4 (10^(-12))/2 (10^(-8))];
R = [0.1];
P = [1 0 0; 0 1 0; 0 0 1];
P_bar = P;
x_k = x;
x_hat_k = x;
x_noise = x;

%plot the resulting unit step response + process noise + measurement noise
gaussian_noise_w = 0.005*randn(1,120001)*sqrt(9);
gaussian_noise_m = 0.001*randn(1,120001);
for j = 0:120000
    
    u_noise = r*N_bar-K*x_noise;
    u = r*N_bar-K*x;
    if(mod(j,10000) < 1)
        x_dot_noise = F*x_noise + G*u_noise + gaussian_noise_w(j+1);
        x_noise(1) = x_noise(1) - 0.005;
    else
        x_dot_noise = F*x_noise + G*u_noise + gaussian_noise_w(j+1);
    end
    x_noise = x_noise + x_dot_noise*dt;
    y_noise = H*x_noise + gaussian_noise_m(j+1);
    Y_noise(:,j+1) = y_noise;
    
    x_dot = F*x + G*u;
    x = x + x_dot*dt;
    y = H*x;
    Y(:,j+1) = y;
    
    x_hat_k = A*x_k + B*u_noise;
    P_bar = A*P*A' + Q;
    Kk = P_bar*H'*inv(H*P_bar*H' + R);
    x_k = x_hat_k + Kk*(y_noise-H*x_hat_k);
    P = P_bar - Kk*H*P_bar;
    yk_hat(:,j+1) = H*x_k;
    
end

t= 0:dt:12;
plot(t,Y)
hold on;
plot(t,Y_noise)
plot(t,yk_hat)
grid on;
hold off;
xlabel('time[s]');
ylabel('rad');
legend('origin','noise','filter')
title('radian')
