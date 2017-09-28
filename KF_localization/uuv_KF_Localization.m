close all
clear all
clc

%Kalman Filter Implementation for 2 Dimensional UUV
load hw1_soln_data.mat
m = 100; %kg
b = 20;  %N-s/m

%state space eom xdot = Fx + Gu and y = Hx + Ju
F = [-b/m 0; 1 0];
G = [1/m; 0];
H = [0 1];
J = [0];

Ts = .05; %s

sys = ss(F, G, H, J);
sysd = c2d(sys,Ts);

[A,B,C,D] = ssdata(sysd);

sigma = Sig0;%diag([1,1]);

% R = diag([1e-2,1e-4]); %noise covariance for velocity and position
% Q = .001; %noise covariance for measurement
% 
% %initialize time
% tfinal = 50; %s
% t = 0:Ts:tfinal;
N = length(t);
% 
% %initialize Force (u)
% u = zeros(1,N);
% u (1:100) = 50;
% u (501:600) = -50;
% 
% %initialize x and z
% x = zeros(2,N);
% x(:,1) = [0,0];
% z = zeros(1,N);
% z(1)=0;
% 
% %compute truth
% for i=2:N
%     x(:,i) = A*x(:,i-1)+B*u(i);%+[sqrt(R(1,1))*randn;sqrt(R(2,2))*randn];
%     z(i) = x(2,i)+(sqrt(Q)*randn); %add noise to measurement
% end

%initialize my
mu = zeros(2,N);
mubar = zeros(2,N);
mu(:,1) = mu0;%[0,0];
mubar(:,1)= [0,0];
K = zeros(2,N);

%Kalman Filter 
for i=2:N
    %prediction
    mubar(:,i) = A*mu(:,i-1)+B*u(i);
    sigmabar = A*sigma(:,:,i-1)*A'+R;
    
    %correction
    K(:,i)=(sigmabar*C')/(C*sigmabar*C'+Q);
    mu(:,i) = mubar(:,i)+K(:,i)*(z(i)-(C*mubar(:,i)));
    sigma(:,:,i) = (eye(2) - K(:,i)*C)*sigmabar;
    errbound1(i)=3*sqrt(sigma(1,1,i));
    errbound2(i)=3*sqrt((sigma(2,2,i)));
    errcov1(i) = sigma(1,1,i);
    errcov2(i) = sigma(2,2,i);
end

%compute error between truth and kalman
kf_error = zeros(2,N);
for i=1:N
%     kf_error(:,i) = mu(:,i)-x(:,i);
    kf_error(:,i) = mu(:,i)-[vtr(:,i);xtr(:,i)];
end

%Plot results
figure(1), clf
subplot(2,1,1)
hold on
% plot (t,x)
plot (t,vtr)
plot (t,mu(1,:))
plot (t,u)
ylim([-5,5])
title ('AUV Position and Velocity')
ylabel('velocity')
legend ('truth','estimate','input')

subplot(2,1,2)
hold on
plot (t,xtr)
plot (t,mu(2,:))
plot (t,u)

xlabel ('time')
ylabel ('position')
legend ('truth','estimate','input')
ylim ([-20,15])

figure(2), clf
subplot (2,1,1)
plot (t,K)
title ('Kalman Gains  and Error Covariance Vs. Time')
ylabel('Kalman Gains')
ylim([0,10])

subplot (2,1,2)
hold on
plot (t,errcov1)
plot (t,errcov2)
xlabel('t')
ylabel('variance')
ylim([0,.05])

figure(3), clf
hold on
subplot(2,1,1)
hold on
title ('Position and Velocity Error Vs. Time')
plot(t,errbound1)
plot(t,-errbound1)
plot (t,kf_error(1,:))
ylabel('velocity')
subplot(2,1,2)
hold on
plot (t,kf_error(2,:))
plot (t,errbound2)
plot (t,-errbound2)
ylabel('position')
xlabel('time')
ylim([-.5,.5])



