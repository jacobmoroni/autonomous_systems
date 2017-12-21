clc
clear
load 'slam2D_Gazebo.mat'

t = t(1:end);
P_array = P_array(2:end,:,:);
xhat_array = xhat_array(2:end,:);
truth_array = truth_array(2:end,:);
t = t-t(1);

t = t(1:10:end);
P_array = P_array(1:10:end,:,:);
xhat_array = xhat_array(1:10:end,:);
truth_array = truth_array(1:10:end,:);
rot = 90*pi/180;
% for i = 1:length(t)
%    xhat_array(i,1:2) = ([cos(rot), -sin(rot); sin(rot),-cos(rot)]*xhat_array(i,1:2)')';
% %    xhat_array(i,2) = xhat_array(i,2)*sin(-pi/2);
% end
xhat_array(:,3) = xhat_array(:,3)+rot;
[b,a] = butter(12,0.2,'low');           % IIR filter design
    xhat_array= filtfilt(b,a,xhat_array);


landmarks= [0.0, 14.5, -5.0;
        5.0, 14.5, -5.0;
        -5.0, 14.5, -5.0;
        0.0, -14.5, -5.0;
        5.0, -14.5, -5.0;
        -5.0, -14.5, -5.0;
        7.0, 0.0, -5.0;
        7.0, 7.5, -5.0;
        7.0, -7.5, -5.0;
        -7.0, 0.0, -5.0;
        -7.0, 7.5, -5.0;
        -7.0, -7.5, -5.0];

pn_e = xhat_array(:,1);
pe_e = xhat_array(:,2);
pd_e = xhat_array(:,3);
u_e  = xhat_array(:,4);
v_e  = xhat_array(:,5);
w_e  = xhat_array(:,6);
phi_e  = xhat_array(:,7);
theta_e  = xhat_array(:,8);
psi_e  = xhat_array(:,9);



for i = 1:length(t)
    drawDrone_gazebo(xhat_array(i,:),t(i),landmarks, P_array(i,:,:),truth_array(i,:));
end
%% Plot Estimates Vs Truth
for i = 1:length(t)
    truth_array(i,9) = wrap(truth_array(i,9));
    xhat_array(i,9) = wrap(xhat_array(i,9));
end

figure(2);clf
subplot(3,1,1)
hold on
plot(t,truth_array(:,1),'r')
plot(t,xhat_array(:,1),'b')
ylabel('North (m)')
xlabel('time (s)')
title('Position Estimates Vs. Truth')
subplot(3,1,2)
hold on
plot(t,truth_array(:,2),'r')
plot(t,xhat_array(:,2),'b')
ylabel('East (m)')
xlabel('time (s)')
subplot(3,1,3)
hold on
plot(t,truth_array(:,9),'r')
plot(t,xhat_array(:,3),'b')
ylabel('\psi (rad)')
xlabel('time (s)')

%% Plot Errors
% 
% truth_array(:,9) = unwrap(truth_array(:,9));
% xhat_array(:,9) = unwrap(xhat_array(:,9));

err = truth_array(:,1:9)-xhat_array(:,1:9);
for i = 1:9
    errcov(:,i) = sqrt(P_array(:,i,i));
end

figure(3);clf
subplot(2,3,1)
hold on
plot (t,err(:,1),'b')
plot (t,2*errcov(:,1),'r')
plot (t,3*errcov(:,1),'y')
plot (t,-2*errcov(:,1),'r')
plot (t,-3*errcov(:,1),'y')
ylabel('Pn (m)')
xlabel('t (s)')

subplot(2,3,2)
hold on
plot (t,err(:,2),'b')
plot (t,2*errcov(:,2),'r')
plot (t,3*errcov(:,2),'y')
plot (t,-2*errcov(:,2),'r')
plot (t,-3*errcov(:,2),'y')
ylabel('Pe (m)')
xlabel('t (s)')

subplot(2,3,3)
hold on
plot (t,err(:,3),'b')
plot (t,2*errcov(:,3),'r')
plot (t,3*errcov(:,3),'y')
plot (t,-2*errcov(:,3),'r')
plot (t,-3*errcov(:,3),'y')
ylabel('Pd (m)')
xlabel('t (s)')
legend('error','2 \sigma bound', '3 \sigma bound')

% subplot(3,3,2)
% hold on
% plot (t,err(:,4),'b')
% plot (t,2*errcov(:,4),'r')
% plot (t,3*errcov(:,4),'y')
% plot (t,-2*errcov(:,4),'r')
% plot (t,-3*errcov(:,4),'y')
% ylabel('u (rad/s)')
% 
% subplot(3,3,5)
% hold on
% plot (t,err(:,5),'b')
% plot (t,2*errcov(:,5),'r')
% plot (t,3*errcov(:,5),'y')
% plot (t,-2*errcov(:,5),'r')
% plot (t,-3*errcov(:,5),'y')
% ylabel('v (rad/s)')
% 
% subplot(3,3,8)
% hold on
% plot (t,err(:,6),'b')
% plot (t,2*errcov(:,6),'r')
% plot (t,3*errcov(:,6),'y')
% plot (t,-2*errcov(:,6),'r')
% plot (t,-3*errcov(:,6),'y')
% ylabel('w (rad/s)')
% xlabel('t (s)')

subplot(2,3,4)
hold on
plot (t,err(:,7),'b')
plot (t,2*errcov(:,7),'r')
plot (t,3*errcov(:,7),'y')
plot (t,-2*errcov(:,7),'r')
plot (t,-3*errcov(:,7),'y')
ylabel('\phi (rad)')
xlabel('t (s)')


subplot(2,3,5)
hold on
plot (t,err(:,8),'b')
plot (t,2*errcov(:,8),'r')
plot (t,3*errcov(:,8),'y')
plot (t,-2*errcov(:,8),'r')
plot (t,-3*errcov(:,8),'y')
ylabel('\theta (rad)')
xlabel('t (s)')

subplot(2,3,6)
hold on
plot (t,err(:,9),'b')
plot (t,2*errcov(:,9),'r')
plot (t,3*errcov(:,9),'y')
plot (t,-2*errcov(:,9),'r')
plot (t,-3*errcov(:,9),'y')
ylabel('\psi (rad)')
xlabel('t (s)')