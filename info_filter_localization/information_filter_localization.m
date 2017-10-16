clear
clc
%information filter turtlebot localization
load ('processed_data.mat')
v = vel_odom(1,:);
w = vel_odom(2,:);
t = odom_t;


MM = length(landmarks);
N = length(t);
show_movement = 1;%decide whether or not to show robot movement (1 --> show)
%% Plot Initial Velocity Data
figure(2); clf
subplot (2,1,1)
plot (t,v)
title('Linear and angular velocity')
ylabel('Linear Velocity (v)')
subplot(2,1,2)
plot (t,w)
ylabel('Angular Velocity (w)')
xlabel('t')

%% Velocity Motion Model
%initialize x,y,theta
x = zeros(1,N);
y = zeros(1,N);
theta = zeros(1,N);
dt = .1*ones(1,N);
x(1) = 6.86;
y(1) = 14.25;
theta(1) = -2.618;
for i=2:N
    dt(i) = t(i) - t(i-1);
%     if w(i) < .05 % Use simple straight line velocity motion model if angular velocity is too small
        x(i) = x(i-1) + (v(i)*cos(theta(i-1)))*dt(i);
        y(i) = y(i-1) + (v(i)*sin(theta(i-1)))*dt(i);
        theta(i) = theta(i-1) +w(i)*dt(i);
%     else
%         x(i) = x(i-1)-v(i)/w(i)*sin(theta(i-1))+v(i)/w(i)*sin(theta(i-1)+w(i)*dt(i));
%         y(i) = y(i-1)+v(i)/w(i)*cos(theta(i-1))-v(i)/w(i)*cos(theta(i-1)+w(i)*dt(i));
%         theta(i) = theta(i-1)+w(i)*dt(i);
%     end
    
    if show_movement == 1
        %show robot movement
        figure(1)
        u=[x(i),y(i),theta(i),t(i)];
        drawBot(u);
    end
end

%plot position data 
figure(3); clf
hold on
plot(x,y)
plot(pos_odom_se2(1,:),pos_odom_se2(2,:))
for m = 1:MM
    plot (landmarks(m,1),landmarks(m,2),'ob','LineWidth',2)
end
xlabel('x position')
ylabel('y position')
title ('Robot position')
legend('Estimated odometry', 'measured odometry','landmarks')

