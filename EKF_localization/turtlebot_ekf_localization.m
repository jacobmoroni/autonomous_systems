clear
clc

%Turtlebot EKF Localization
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
x_true(1) = -5;%m
y_true(1) = -3;%m
theta_true(1) = 90*pi/180;%deg

%linear and angular velocity equations
v_c = 1+ 0.5*cos(2*pi*.2*t);
w_c = -.2 + 2*cos(2*pi*0.6*t);

%noise characteristics of velocities experienced by the robot
alpha1 = .1;
alpha2 = .01;
alpha3 = .01;
alpha4 = .1;
alpha5 = 0;
alpha6 = 0;

%landmarks visible by robot (x;y) 
l1 = [6;4];
l2 = [-7;8];
l3 = [6;-4];
landmark = [l1,l2,l3];
% landmark = [6, -7, 6;
%             4, 8, -4];

%measurement noise
sig_r = .1; %range measurement
sig_phi = .05; %bearing measurement

% plot (t,v_c)
% hold on
% plot (t,w_c)

 %initialize variables used in computing measurement
z = zeros(2,3,N);
dx = zeros(3,N);
dy = zeros(3,N);
range = zeros(3,N);
bearing = zeros(3,N);

for i=2:N
    % Implement velocity motion model
    v_hat = v_c(i)+randn*sqrt(alpha1*v_c(i)^2+alpha2*w_c(i)^2); %add noise to linear velocity
    w_hat = w_c(i)+randn*sqrt(alpha3*v_c(i)^2+alpha4*w_c(i)^2); %add noise to angular velocity
    gamma_hat = randn*sqrt(alpha5*v_c(i)^2+alpha6*w_c(i)^2); %gamma velocity (not used, so set to 0)
    x_true(i) = x_true(i-1)-v_hat/w_hat*sin(theta_true(i-1))+v_hat/w_hat*sin(theta_true(i-1)+w_hat*Ts); %extract x from taylor series expansion?
    y_true(i) = y_true(i-1)+v_hat/w_hat*cos(theta_true(i-1))-v_hat/w_hat*cos(theta_true(i-1)+w_hat*Ts); %extract ? from taylor series expansion?
    theta_true(i) = theta_true(i-1)+w_hat*Ts+gamma_hat*Ts; %extract theta from taylor series expansion?
    
    %Get Range and bearing measurements
    for j=1:3 %iterate through each of the 3 landmarks
        dx(j,i) = landmark(1,j)-x_true(i); %x distance from bot to landmark
        dy(j,i) = landmark(2,j)-y_true(i); %y distance from bot to landmark
        range(j,i) = sqrt(dx(j,i)^2+dy(j,i)^2)+randn*sig_r; %l2 norm from bot to landmark (noise added)
        bearing(j,i) = atan2(dy(j,i),dx(j,i))-theta_true(i)+randn*sig_phi; %bearing angle from front of bot to landmark (noise added)
        z(:,j,i) = [range(j,i);bearing(j,i)]; %collapse into single measurement matrix
    end
    
    %show robot movement
    u=[x_true(i),y_true(i),theta_true(i),t(i)];
    drawBot(u);
    
end
%% EKF IMPLEMENTATION
 %initialize variables used in ekf
 v = v_c;
 w = w_c;
 theta = zeros(1,N);
 G = zeros(3,3,N);
 V = zeros(3,2,N);
 M = zeros(2,2,N);
 H = zeros(2,3,3,N);
 mubar = zeros(3,N);
 mu = zeros(3,N);
 sigmabar = zeros(3,3,N);
 sigma = zeros(3,3,N);
 zhat = zeros(2,3,N);
 dx_est = zeros(3,N);
 dy_est = zeros(3,N);
 range_est = zeros(3,N);
 bearing_est = zeros(3,N);
 S = zeros(2,2,3,N);
 K = zeros(3,2,3,N);
 
 %initial state for estimates
 mu(:,1) = [x_true(1),y_true(1),theta_true(1)];
 
 
% % Implement EKF algorithm
for i=2:N
    %prediction step
    theta(i)=mu(3,i-1); % State estimate from prior time
    
    % Jacobian of g(u(t), x(t-1)) with respect to state
    G(:,:,i) = [1 0 -v(i)/w(i)*cos(theta(i))+v(i)/w(i)*cos(theta(i)+w(i)*Ts);
                0 1 -v(i)/w(i)*sin(theta(i))+v(i)/w(i)*sin(theta(i)+w(i)*Ts);
                0 0 1];
            
    % Jacobian of g(u(t), x(t-1)) with respect to inputs    
    V(:,:,i) = [(-sin(theta(i))+sin(theta(i)+w(i)*Ts))/w(i) (v(i)*(sin(theta(i))-sin(theta(i)+w(i)*Ts)))/(w(i)^2)+(v(i)*cos(theta(i)+w(i)*Ts)*Ts)/(w(i));
                (cos(theta(i))-cos(theta(i)+w(i)*Ts))/w(i) -(v(i)*(cos(theta(i))-cos(theta(i)+w(i)*Ts)))/(w(i)^2)+(v(i)*sin(theta(i)+w(i)*Ts)*Ts)/(w(i));
                 0 Ts];
    
    % Noise in control space [forward, 0; 0, Angular] 
    M(:,:,i) = [alpha1*(v(i)^2) + alpha2*(w(i)^2),                 0                ;
                              0                  , alpha3*(v(i)^2) + alpha4*(w(i)^2)];
    
    % state estimate mubar = g(u(t),mu(t-1))
    mubar(:,i) = mu(:,i-1)+[(-v(i)/w(i)*sin(theta(i)))+v(i)/w(i)*sin(theta(i)+w(i)*Ts);
                             (v(i)/w(i)*cos(theta(i)))-v(i)/w(i)*cos(theta(i)+w(i)*Ts);
                                                     w(i)*Ts                          ];
                                                 
    % Uncertainty in states at time t-1 + uncertainty due to motion noise                     
    sigmabar(:,:,i) = G(:,:,i)*sigma(:,:,i-1)*G(:,:,i)' + V(:,:,i)*M(:,:,i)*V(:,:,i)';
    
    % Uncertainty due to measurement noise 
    Q = [sig_r^2, 0 ; 0, sig_phi^2];
    
    %measurement update (correction)
    for j=1:3 %step through each of the 3 landmarks
        
        %estimated distance from landmark
        dx_est(j,i) = landmark(1,j)-mubar(1,i);
        dy_est(j,i) = landmark(2,j)-mubar(2,i);
        range_est(j,i) = sqrt(dx_est(j,i)^2+dy_est(j,i)^2);
        
        %estimated bearing angle from front of bot to landmark
        bearing_est(j,i) = atan2(dy_est(j,i),dx_est(j,i))-mubar(3,i);
        
        %combined estimated measurement matrix
        zhat(:,j,i) = [range_est(j,i);bearing_est(j,i)];
        
        
        H(:,:,j,i) = [-dx(j,i)/range_est(j,i), -dy(j,i)/range_est(j,i), 0;
                      dy(j,i)/(range_est(j,i)^2), -dx(j,i)/(range_est(j,i)^2), -1];
                  
        % Uncertainty in zhat (Measurement uncertainty in robot state + measurement uncertainty          
        S(:,:,j,i) = H(:,:,j,i)*sigmabar(:,:,i)*H(:,:,j,i)' + Q;
        
        % Kalman Gains
        K(:,:,j,i) = sigmabar(:,:,i)*H(:,:,j,i)'/S(:,:,j,i);
        
        % Kalman Gain maps innovation in measurement space into state space
        mubar(:,i) = mubar(:,i) + K(:,:,j,i)*(z(:,j,i)-zhat(:,j,i));
        
        % Updatad state uncertainty based on measurement
        sigmabar(:,:,i) = (eye(3)-K(:,:,j,i)*H(:,:,j,i))*sigmabar(:,:,i);
       
    end
    
    %assign mubar and sigmabar values to mu and sigma
    mu(:,i)=mubar(:,i);
    sigma(:,:,i) = sigmabar(:,:,i); 
    errcov_x(i) = sigma(1,1,i);
    errcov_y(i) = sigma(2,2,i);
    errcov_theta(i) = sigma(3,3,i);
end
 
%% PLOTS
    %plot true and estimated paths
    figure(1)
    hold on
    plot(x_true,y_true)
    comet(mu(1,:),mu(2,:))
    title ('movement of turtlebot')
    xlabel ('x')
    ylabel ('y')
    
    %plot states
    figure(2); clf
    subplot (3,1,1)
    hold on
    plot (t, x_true(:))
    plot (t, mu(1,:))
    ylabel ('X posistion')
    title ('State Truth Vs. Estimates')
    legend ('truth','estimate')
    subplot (3,1,2)
    hold on
    plot (t, y_true(:))
    plot (t, mu(2,:))
    ylabel ('Y posistion')
    subplot (3,1,3)
    hold on
    plot (t, theta_true(:))
    plot (t, mu(3,:))
    ylabel ('theta')
    xlabel ('time (t)')
    
    %compute errors
    error_x = x_true - mu(1,:);
    error_y = y_true - mu(2,:);
    error_theta = theta_true - mu(3,:);
    
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
    subplot (3,1,2)
    hold on
    plot (t, error_y)
    plot (t, 2*sqrt(errcov_y),'r')
    plot (t, -2*sqrt(errcov_y),'r')
    plot (t, 3*sqrt(errcov_y),'y')
    plot (t, -3*sqrt(errcov_y),'y')
    ylabel ('Y error')
    subplot (3,1,3)
    hold on
    plot (t, error_theta)
    plot (t, 2*sqrt(errcov_theta),'r')
    plot (t, -2*sqrt(errcov_theta),'r')
    plot (t, 3*sqrt(errcov_theta),'y')
    plot (t, -3*sqrt(errcov_theta),'y')
    ylabel ('theta error')
    xlabel ('time (t)')
    
    %extract kalman gains
    K1_1(:) = K(1,1,1,:);
    K1_2(:) = K(1,1,2,:);
    K1_3(:) = K(1,1,3,:);
    K2_1(:) = K(1,2,1,:);
    K2_2(:) = K(1,2,2,:);
    K2_3(:) = K(1,2,3,:);
    K3_1(:) = K(2,1,1,:);
    K3_2(:) = K(2,1,2,:);
    K3_3(:) = K(2,1,3,:);
    K4_1(:) = K(2,2,1,:);
    K4_2(:) = K(2,2,2,:);
    K4_3(:) = K(2,2,3,:);
    K5_1(:) = K(3,1,1,:);
    K5_2(:) = K(3,1,2,:);
    K5_3(:) = K(3,1,3,:);
    K6_1(:) = K(3,2,1,:);
    K6_2(:) = K(3,2,2,:);
    K6_3(:) = K(3,2,3,:);
    
    %plot kalman gains
    figure(4); clf
    subplot (3,2,1)
    hold on
    plot (t, K1_1)
    plot (t, K1_2)
    plot (t, K1_3)
    ylabel ('Kalman gain 1')
    title('kalman Gains vs time')
    subplot (3,2,2)
    hold on
    plot (t, K2_1)
    plot (t, K2_2)
    plot (t, K2_3)
    ylabel ('Kalman gain 2')
    legend ('landmark1','landmark2','landmark3')
    subplot (3,2,3)
    hold on
    plot (t, K3_1)
    plot (t, K3_2)
    plot (t, K3_3)
    ylabel ('Kalman gain 3')
    subplot (3,2,4)
    hold on
    plot (t, K4_1)
    plot (t, K4_2)
    plot (t, K4_3)
    ylabel ('Kalman gain 4')
    subplot (3,2,5)
    hold on
    plot (t, K5_1)
    plot (t, K5_2)
    plot (t, K5_3)
    ylabel ('Kalman gain 5')
    xlabel ('time (t)')
    subplot (3,2,6)
    hold on
    plot (t, K6_1)
    plot (t, K6_2)
    plot (t, K6_3)
    ylabel ('Kalman gain 6')
    xlabel ('time (t)')
   