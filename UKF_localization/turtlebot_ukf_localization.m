clear
clc


%Turtlebot UKF Localization

 %tuning parameters
 L = 7; %Number of augmented states
 n = 7;
 kappa = 0;% 4;
 alpha = 1;%.3;
 beta = 2; 
 lambda = alpha^2*(n+kappa)-n;
 gamma = sqrt(n+lambda);
 MM = 3; % No. of landmarks
 showbot = 0;%true;
 
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
alphas = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6];

%landmarks visible by robot (x;y) 
l1 = [6;4];
l2 = [-7;8];
l3 = [6;-4];
landmark = [l1,l2,l3];
% landmark = [l1];


%measurement noise
sig_r = .1; %range measurement
sig_phi = .05; %bearing measurement

%initialize variables used in computing measurement
z = zeros(2,N);
dx = zeros(3,N);
dy = zeros(3,N);
range = zeros(3,N);
bearing = zeros(3,N);

for i=2:N
    input = [v_c(i),w_c(i)];
    state_true = [x_true(i-1), y_true(i-1), theta_true(i-1)];
    % Implement velocity motion model
    state_prime = vel_motion_model(input, state_true, alphas, Ts);
%     state_prime = vel_motion_model(input, state_true, [0,0,0,0,0,0], Ts);
    x_true(i) = state_prime(1);
    y_true(i) = state_prime(2);
    theta_true(i) = state_prime(3);
    
    %Get Range and bearing measurements
    k = 1+mod(i,MM);
    z(:,i) = measurement(k,landmark,sig_r,sig_phi,x_true(i),y_true(i),theta_true(i));
%     z(:,i) = measurement(k,landmark,0,0,x_true(i),y_true(i),theta_true(i));  

    if showbot == true
        %show robot movement
        u=[x_true(i),y_true(i),theta_true(i),t(i)];
        drawBot(u);
    end
    
end
%% UKF IMPLEMENTATION
 %initialize variables used in ukf
 v = v_c;
 w = w_c;
 weight_m = zeros(1,2*L+1);
 weight_c = zeros(1,2*L+1);
 M = zeros(2,2,N);
 mu_a = zeros(L,N);
 mu = zeros(3,N);
 mubar = zeros(3,N);
 sigma = zeros(3,3,N);
 sigma_a = zeros(L,L,N);
 chi_a = zeros(L,2*L+1,N);
 chi_x = zeros(3,2*L+1,N);
 chibar_x = zeros(3,2*L+1,N);
 chi_u = zeros(2,2*L+1,N);
 chi_z = zeros(2,2*L+1,N);
 sigbar = zeros(3,3,N);
 Zbar = zeros(2,2*L+1,N);
 zhat = zeros(2,N);
 S = zeros(2,2,N);
 sigma_xz = zeros(3,2,N);
 K = zeros(3,2,N);
 sigma(:,:,1) = eye(3);
%  mu_a(:,1) = 1;

 
 %initial state for estimates
 mu(:,1) = [x_true(1),y_true(1),theta_true(1)];
 
 %weights for distribution
 weight_m(1) = lambda/(n+lambda);
 weight_c(1) = lambda/(n+lambda)+(1-alpha^2 +beta);
 weight_m(2:2*L+1) = 1/(2*(n+lambda));
 weight_c(2:2*L+1) = weight_m(2:2*L+1);
 
 
% % Implement UKF algorithm
for i=2:N
    % Generate augmented mean and covariance
    
    % Noise in control space [forward, 0; 0, Angular] 
    M(:,:,i) = [alpha1*(v(i)^2) + alpha2*(w(i)^2),                 0                ;
                              0                  , alpha3*(v(i)^2) + alpha4*(w(i)^2)];
                          
    % Uncertainty due to measurement noise 
    Q = [sig_r^2, 0 ; 0, sig_phi^2];
    
    %augmented state
    mu_a(:,i-1) = [mu(:,i-1)',0,0,0,0]';
    
    sigma_a(:,:,i-1) = [sigma(:,:,i-1) , zeros(3,2) , zeros(3,2);
                        zeros(2,3)     ,   M(:,:,i) , zeros(2,2);
                        zeros(2,3)     , zeros(2,2) , Q         ];
    
    % Generate sigma points
    chi_a(:,:,i-1) = [mu_a(:,i-1), mu_a(:,i-1)+gamma*chol(sigma_a(:,:,i-1)), mu_a(:,i-1)-gamma*chol(sigma_a(:,:,i-1))];
    chi_x(:,:,i-1) = chi_a(1:3,:,i-1);
    chi_u(:,:,i) = chi_a(4:5,:,i-1);
    chi_z(:,:,i) = chi_a(6:7,:,i-1);
    % Pass sigma points through motion model and compute gausian statistics
    input = [v_c(i),w_c(i)];
    for j = 1:2*L+1
        chibar_x(:,j,i) = UKF_vel_motion_model(input, chi_a(:,j,i-1), Ts);
    end
    
    mubar(:,i) = chibar_x(:,:,i)* weight_m';
    sigbar(:,:,i) = weight_c.*(chibar_x(:,:,i)-mubar(:,i))*(chibar_x(:,:,i)-mubar(:,i))';
    
    %predict observations at sigma points and compute gaussian statistics
    for j = 1:2*L+1
        k = 1+mod(i,MM);
        Zbar(:,j,i) = UKF_measurement(k,landmark,chibar_x(:,j,i)); 
    end
    Zbar(:,:,i) = Zbar(:,:,i)+chi_z(:,:,i-1);
    zhat(:,i) = Zbar(:,:,i)*weight_m';
    S(:,:,i) = weight_c.*(Zbar(:,:,i)-zhat(:,i))*(Zbar(:,:,i)-zhat(:,i))';
    sigma_xz(:,:,i) = weight_c.*(chibar_x(:,:,i)-mubar(:,i))*(Zbar(:,:,i)-zhat(:,i))';
    
    % Update mean and covariance
    K(:,:,i) = sigma_xz(:,:,i)/S(:,:,i);
    mu(:,i) = mubar(:,i) + K(:,:,i)*(z(:,i)-zhat(:,i));
    sigma(:,:,i) = sigbar(:,:,i) - K(:,:,i)*S(:,:,i)*K(:,:,i)';
end
 
%% PLOTS
    %plot true and estimated paths\
    frame_width = 12;
    figure(1);
    if showbot ~= true
        clf
    end
    hold on
    
    axis([-frame_width, frame_width, -frame_width, frame_width]);
    plot(landmark(1,1),landmark(2,1),'ob','LineWidth', 2); % plot landmark
    plot(landmark(1,2),landmark(2,2),'or','LineWidth', 2); % plot landmark
    plot(landmark(1,3),landmark(2,3),'oy','LineWidth', 2); % plot landmark
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
    errcov_x(:) = sigma(1,1,:);
    errcov_y(:) = sigma(2,2,:);
    errcov_theta(:) = sigma(3,3,:);
    
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

    %extract kalman gains
    K1_1(:) = K(1,1,:);
    K1_2(:) = K(1,2,:);
    K2_1(:) = K(2,1,:);
    K2_2(:) = K(2,2,:);
    K3_1(:) = K(3,1,:);
    K3_2(:) = K(3,2,:);
   
    %plot kalman gains
    figure(4); clf
    subplot (3,2,1)
    hold on
    plot (t, K1_1)
    ylabel ('Kalman gain 1')
    title('kalman Gains vs time')
    subplot (3,2,2)
    hold on
    plot (t, K1_2)
    ylabel ('Kalman gain 2')
%     legend ('landmark1','landmark2','landmark3')
    subplot (3,2,3)
    hold on
    plot (t, K2_1)
    ylabel ('Kalman gain 3')
    subplot (3,2,4)
    hold on
    plot (t, K2_2)
    ylabel ('Kalman gain 4')
    subplot (3,2,5)
    hold on
    plot (t, K3_1)
    ylabel ('Kalman gain 5')
    xlabel ('time (t)')
    subplot (3,2,6)
    hold on
    plot (t, K3_2)
    ylabel ('Kalman gain 6')
    xlabel ('time (t)')
   
%     %measurements
%     Zbar_range(:,:) = Zbar(1,:,:);
%     Zbar_bearing(:,:) = Zbar(2,:,:);
%     figure(5);clf
%     
%     subplot(2,1,1)
%     hold on
%     plot (t,z(1,:))
%     plot (t,Zbar_range(1,:))
%     plot (t,zhat(1,:))
%     legend('z','Zbar','zhat')
%     subplot(2,1,2)
%     hold on
%     plot (t,z(2,:))
%     plot (t,Zbar_bearing(1,:))
%     plot (t,zhat(2,:))
%     legend('z','Zbar','zhat')