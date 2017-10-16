clear
clc


%Turtlebot Particle Filter Localization

 %tuning parameters
 MM = 3; % No. of landmarks
 M = 1000; %number of points in particle filter
 showbot = true;
 
%% VELOCITY MOTION MODEL
%initialize time
Ts = .1 %s
tfinal = 20; %s
t = 0:Ts:tfinal;
N = length(t);

%initialize states
x_true = zeros(1,N);
y_true = zeros(1,N);
theta_true = zeros(1,N);

%initial positions
x_true(1) = -5;%m
y_true(1) = -3;%m
theta_true(1) = 90*pi/180;%deg

%linear and angular velocity equations
v_c = 1+ 0.5*cos(2*pi*.2*t);
w_c = -.2 + 2*cos(2*pi*0.6*t);

% v_c = 10+ 0.5*cos(2*pi*.2*t);
% w_c = -.2 + 2*cos(2*pi*0.6*t);

%noise characteristics of velocities experienced by the robot
alpha1 = .1;
alpha2 = .01;
alpha3 = .01;
alpha4 = .1;
alpha5 = 0;
alpha6 = 0;
alphas = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6];

%landmarks visible by robot (x;y) 
l1 = [6;4];
l2 = [-7;8];
l3 = [6;-4];
if MM == 3
    landmark = [l1,l2,l3];
else
    landmark = [l1];
end

%measurement noise
sig_r = .1; %range measurement
sig_phi = .05; %bearing measurement
sig = [sig_r;sig_phi];

%initialize variables used in computing measurement
z = zeros(2,MM,N);
dx = zeros(3,N);
dy = zeros(3,N);
range = zeros(3,N);
bearing = zeros(3,N);

for i=2:N
    input = [v_c(i),w_c(i)];
    state_true = [x_true(i-1), y_true(i-1), theta_true(i-1)];
    % Implement velocity motion model
    state_prime = vel_motion_model(input, state_true, alphas, Ts);
%     state_prime = vel_motion_model(input, state_true, [0,0,0,0,0,0], Ts); %remove noise from motion model
    x_true(i) = state_prime(1);
    y_true(i) = state_prime(2);
    theta_true(i) = state_prime(3);
    
    %Get Range and bearing measurements
    for j = 1:MM
        z(:,j,i) = measurement(j,landmark,sig_r,sig_phi,x_true(i),y_true(i),theta_true(i));
%     z(:,i) = measurement(k,landmark,0,0,x_true(i),y_true(i),theta_true(i));  %remove noise from measurement
    end

    if showbot == true
        %show robot movement
        u=[x_true(i),y_true(i),theta_true(i),t(i),Ts];
        drawBot(u);
    end
    
end
%% PARTICLE FILTER IMPLEMENTATION
%initialize variables used in particle filter
z_t = z; %measurement
u_t = [v_c;w_c]; %input
mu_x = zeros(1,length(t));
mu_y = zeros(1,length(t));
mu_th = zeros(1,length(t));
stdev_x = zeros(1,length(t));
stdev_y = zeros(1,length(t));
stdev_th = zeros(1,length(t));
mu_x(1) = 0;%x_true(1);
mu_y(1) = 0;%y_true(1);
mu_th(1) = 0;%theta_true(1);

%initialize chi (random x and y accross span and random theta +/- pi/6 from
%90 degrees
% chi_t_1 = diag([20 20 pi/6])*(rand(3,M)-.5)+[zeros(1,M);zeros(1,M);pi/2*ones(1,M)]; 

% initialize chi random across full span of map and theta
chi_t_1 = diag([20 20 pi/2])*(rand(3,M)-.5);

% % Implement Monte Carlo algorithm
for i=2:N
    chi_t = MonteCarlo(chi_t_1, u_t(:,i),z_t(:,:,i),landmark,alphas,Ts,sig);
    chi_t_1 = chi_t; %reset chi for next step
    mu_x(i) = mean(chi_t(1,:));
    stdev_x(i) = std(chi_t(1,:));
    mu_y(i) = mean(chi_t(2,:));
    stdev_y(i) = std(chi_t(2,:));
    mu_th(i) = mean(chi_t(3,:));
    stdev_th(i) = std(chi_t(3,:));
end
 
% PLOTS
    %plot true and estimated paths\
    frame_width = 12;
    figure(1);
    if showbot ~= true
        clf
    end
    hold on
    
    axis([-frame_width, frame_width, -frame_width, frame_width]);
    if MM ==3
        plot(landmark(1,1),landmark(2,1),'ob','LineWidth', 2); % plot landmark
        plot(landmark(1,2),landmark(2,2),'or','LineWidth', 2); % plot landmark
        plot(landmark(1,3),landmark(2,3),'oy','LineWidth', 2); % plot landmark
    end
    plot(x_true,y_true)
    plot(mu_x,mu_y,'y')
    title ('movement of turtlebot')
    xlabel ('x')
    ylabel ('y')
    
    
    %plot states
    figure(2); clf
    subplot (3,1,1)
    hold on
    plot (t, x_true(:))
    plot (t, mu_x)
    ylabel ('X posistion')
    title ('State Truth Vs. Estimates')
    legend ('truth','estimate')
    subplot (3,1,2)
    hold on
    plot (t, y_true(:))
    plot (t, mu_y)
    ylabel ('Y posistion')
    subplot (3,1,3)
    hold on
    plot (t, theta_true(:))
    plot (t, mu_th)
    ylabel ('theta')
    xlabel ('time (t)')
    
    %compute errors
    error_x = x_true - mu_x;
    error_y = y_true - mu_y;
    error_theta = theta_true - mu_th;
    errcov_x(:) = stdev_x;
    errcov_y(:) = stdev_y;
    errcov_theta(:) = stdev_th;
%     
    %plot errors
    figure(3); clf
    subplot (3,1,1)
    hold on
    plot (t, error_x)
    plot (t, 2*sqrt(errcov_x),'r')
    plot (t, -2*sqrt(errcov_x),'r')
    plot (t, 3*sqrt(errcov_x),'y')
    plot (t, -3*sqrt(errcov_x),'y')
    ylabel ('X error')
    title ('State Estimate Error')
    ylim ([-.5,.5])
    subplot (3,1,2)
    hold on
    plot (t, error_y)
    plot (t, 2*sqrt(errcov_y),'r')
    plot (t, -2*sqrt(errcov_y),'r')
    plot (t, 3*sqrt(errcov_y),'y')
    plot (t, -3*sqrt(errcov_y),'y')
    ylabel ('Y error')
    ylim ([-.5,.5])
    subplot (3,1,3)
    hold on
    plot (t, error_theta)
    plot (t, 2*sqrt(errcov_theta),'r')
    plot (t, -2*sqrt(errcov_theta),'r')
    plot (t, 3*sqrt(errcov_theta),'y')
    plot (t, -3*sqrt(errcov_theta),'y')
    ylabel ('theta error')
    xlabel ('time (t)')
    ylim ([-.5,.5])

%     extract kalman gains
%     K1_1(:) = K(1,1,:);
%     K1_2(:) = K(1,2,:);
%     K2_1(:) = K(2,1,:);
%     K2_2(:) = K(2,2,:);
%     K3_1(:) = K(3,1,:);
%     K3_2(:) = K(3,2,:);
%    
%     plot kalman gains
%     figure(4); clf
%     subplot (3,2,1)
%     hold on
%     plot (t, K1_1)
%     ylabel ('Kalman gain 1')
%     title('kalman Gains vs time')
%     subplot (3,2,2)
%     hold on
%     plot (t, K1_2)
%     ylabel ('Kalman gain 2')
%     legend ('landmark1','landmark2','landmark3')
%     subplot (3,2,3)
%     hold on
%     plot (t, K2_1)
%     ylabel ('Kalman gain 3')
%     subplot (3,2,4)
%     hold on
%     plot (t, K2_2)
%     ylabel ('Kalman gain 4')
%     subplot (3,2,5)
%     hold on
%     plot (t, K3_1)
%     ylabel ('Kalman gain 5')
%     xlabel ('time (t)')
%     subplot (3,2,6)
%     hold on
%     plot (t, K3_2)
%     ylabel ('Kalman gain 6')
%     xlabel ('time (t)')
%    
% %     %measurements
% %     Zbar_range(:,:) = Zbar(1,:,:);
% %     Zbar_bearing(:,:) = Zbar(2,:,:);
% %     figure(5);clf
% %     
% %     subplot(2,1,1)
% %     hold on
% %     plot (t,z(1,:))
% %     plot (t,Zbar_range(1,:))
% %     plot (t,zhat(1,:))
% %     legend('z','Zbar','zhat')
% %     subplot(2,1,2)
% %     hold on
% %     plot (t,z(2,:))
% %     plot (t,Zbar_bearing(1,:))
% %     plot (t,zhat(2,:))
% %     legend('z','Zbar','zhat')