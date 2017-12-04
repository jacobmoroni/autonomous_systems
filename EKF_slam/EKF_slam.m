%Turtlebot EKF SLAM
clear
clc
showbot = true;
MM = 21; %Number of landmarks
%% VELOCITY MOTION MODEL
%initialize time
Ts = .1; %s
tfinal = 20; %s
t = 0:Ts:tfinal;
N = length(t);

%initialize states
x_true = zeros(1,N);
y_true = zeros(1,N);
theta_true = zeros(1,N);

%initial positions
x_true(1) = 0;%m
y_true(1) = 0;%m
theta_true(1) = 0;%deg

%linear and angular velocity equations
v_c = 1+ 0.5*cos(2*pi*.2*t);
w_c = -.4 + 2*cos(2*pi*0.6*t);

%noise characteristics of velocities experienced by the robot
alpha1 = .1;
alpha2 = .01;
alpha3 = .01;
alpha4 = .1;
alpha5 = 0;
alpha6 = 0;
alphas = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6];
%landmarks visible by robot (x;y) 
landmark = (rand(2,MM)-.5)*20;

%measurement noise
sig_r = .1; %range measurement
sig_phi = .05; %bearing measurement


 %initialize variables used in computing measurement
z = zeros(2,3,N);
dx = zeros(3,N);
dy = zeros(3,N);
range = zeros(3,N);
bearing = zeros(3,N);
state_true = [0,0,0];
for i=1:N
    input = [v_c(i),w_c(i)];
    
    % Implement velocity motion model
    state_prime = vel_motion_model(input, state_true, alphas, Ts);
%     state_prime = vel_motion_model(input, state_true, [0,0,0,0,0,0], Ts);
    x_true(i) = state_prime(1);
    y_true(i) = state_prime(2);
    theta_true(i) = state_prime(3);
    theta_true(i) = wrap(theta_true(i));
    %Get Range and bearing measurements
    for j = 1:MM
        z(:,j,i) = measurement(j,landmark,sig_r,sig_phi,x_true(i),y_true(i),theta_true(i));
%         z(:,j,i) = measurement(j,landmark,0,0,x_true(i),y_true(i),theta_true(i));
    end
    state_true = state_prime;
%     if showbot == true
%         %show robot movement
%         u=[x_true(i),y_true(i),theta_true(i),t(i)];
%         drawBot(u,landmark);
%     end
    X = [x_true;y_true;theta_true];
end
%% EKF SLAM IMPLEMENTATION
 %initialize variables used in ekf
 u = [v_c;w_c];
 %initial state for estimates
%  mu = [x_true(1);y_true(1);theta_true(1);zeros(2*MM,1)];
 mu = zeros(2*MM+3,1);
 Sigma = [zeros(2*MM+3,3), 10^10*rot90(eye(2*MM+3,2*MM),2)];
 mu_est = mu;
 zhat_est(:,:,1) = z(:,:,1);
% % Implement EKF slam algorithm

% figure(1);clf
% hold on
% xlim([-10 10]);
% ylim([-10 10]);
% for j = 1:MM
%     plot (landmark(1,j),landmark(2,j),'or')
% end

for i=1:N
    [mu, Sigma,zhat] = EKF_SLAM_known_correspondences(mu,Sigma,u(:,i),z(:,:,i),MM,Ts);
    mu_est(:,:,i) = mu;
    zhat_est(:,:,i) = zhat;
    errcov(:,i) = [Sigma(1,1);Sigma(2,2);Sigma(3,3)];
    if showbot == true
        drawBot_est(mu,t(i),landmark,Sigma,X(:,i));
    end
end

%% PLOTS
if showbot == false
    %plot true and estimated paths
    figure(1);
    hold on
    plot(x_true,y_true)
    plot(mu_est(1,:),mu_est(2,:))
    title ('movement of turtlebot')
    xlabel ('x')
    ylabel ('y')
    for j = 1:MM
        plot (landmark(1,j),landmark(2,j),'or')
        landmark_x= mu(3+2*j-1);
        landmark_y= mu(3+2*j);
        c= Sigma(3+2*j-1:3+2*j,3+2*j-1:3+2*j);
        c = 2*chol(c);
        [xx,yy,zz] = getellipse(c,landmark_x,landmark_y);
        plot(xx,yy,'g')
    end
end

%plot states
figure(2); clf
subplot (3,1,1)
hold on
plot (t, x_true(:))
plot (t, mu_est(1,:))
ylabel ('X posistion')
title ('State Truth Vs. Estimates')
legend ('truth','estimate')
subplot (3,1,2)
hold on
plot (t, y_true(:))
plot (t, mu_est(2,:))
ylabel ('Y posistion')
subplot (3,1,3)
hold on
plot (t, theta_true(:))
plot (t, mu_est(3,:))
ylabel ('theta')
xlabel ('time (t)')

if mod(MM,4) == 0
    figure(4);clf
    title ('range and range estimates')
    hold on
    for i = 1:MM
        subplot(4,MM/4,i)
        hold on
        z1(:,:) = z(:,i,:);
        zhat_est1(:,:) = zhat_est(:,i,:);
        plot (t,z1(2,:))
        plot (t,zhat_est1(2,:))
        ylim = [-pi/6 pi/6];
    end
end
    %compute errors
    error_x = x_true - mu_est(1,:);
    error_y = y_true - mu_est(2,:);
    error_theta = theta_true - mu_est(3,:);
    for i = 1:N
        error_theta(i) = wrap(error_theta(i));
    end
    
    %plot errors
    figure(3); clf
    subplot (3,1,1)
    hold on
    plot (t, error_x)
    plot (t, 2*sqrt(errcov(1,:)),'r')
    plot (t, -2*sqrt(errcov(1,:)),'r')
    plot (t, 3*sqrt(errcov(1,:)),'y')
    plot (t, -3*sqrt(errcov(1,:)),'y')
    ylabel ('X error')
    title ('State Estimate Error')
    subplot (3,1,2)
    hold on
    plot (t, error_y)
    plot (t, 2*sqrt(errcov(2,:)),'r')
    plot (t, -2*sqrt(errcov(2,:)),'r')
    plot (t, 3*sqrt(errcov(2,:)),'y')
    plot (t, -3*sqrt(errcov(2,:)),'y')
    ylabel ('Y error')
    subplot (3,1,3)
    hold on
    plot (t, error_theta)
    plot (t, 2*sqrt(errcov(3,:)),'r')
    plot (t, -2*sqrt(errcov(3,:)),'r')
    plot (t, 3*sqrt(errcov(3,:)),'y')
    plot (t, -3*sqrt(errcov(3,:)),'y')
    ylabel ('theta error')
    xlabel ('time (t)')

    figure(5)
    h = heatmap(Sigma);
%     map = [1 0 0;
%            1 1 1;
%            0 0 0];
%     h.Colormap = summer;
% %     cmap = colormap
    title ('Sigma Matrix')
    grid off

   