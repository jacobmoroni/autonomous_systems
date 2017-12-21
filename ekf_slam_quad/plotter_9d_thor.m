clc
load 'slam3D_mat.mat'

t = t(2:end);
P_array = P_array(2:end,:,:);
xhat_array = xhat_array(2:end,:);
truth_array = truth_array(2:end,:);
t = t-t(1);

P_array1 = zeros(739,36,36);
P_array1(:,1:9,1:9) = P_array(:,1:9,1:9);
P_array1(:,10:36,10:36) = P_array(:,13:39,13:39);
P_array = P_array1;

xhat_array1 = zeros(739,36);
xhat_array1(:,1:9) = xhat_array(:,1:9);
xhat_array1(:,10:36) = xhat_array(:,13:39);
xhat_array = xhat_array1;

landmarks =[-0.51	 2.302	-1.31;
             1.455	 2.493	-1.486;
             3.92	 1.333	-1.498;
             3.964	-1.753	-1.566;
             2.916	-2.543	-1.537;
             1.181	-2.471	-1.581;
            -1.593	-2.488	-1.572;
            -3.528	-0.658	-1.461;
            -2.023	 2.462	-1.492];

% landmarks =[ -2.529	 1.496	-0.833;
%              -2.531	 2.464	-1.583;
%              -1.664	 2.469	-1.083;
%              -1.514	 2.469	-1.677;
%              -0.522	 2.484	-1.078;
%               0.62	 2.498	-0.932;
%               0.608	 2.501	-1.699;
%               1.485	 2.498	-1.488;
%               2.263	 2.515	-1.095;
%               3.917	 1.293	-1.488;
%               3.934	 0.644	-1.57;
%               3.948	-0.152	-1.358;
%               3.956	-0.833	-1.708;
%               3.971	-1.703	-1.597;
%               2.862	-2.495	-1.502;
%               1.798	-2.485	-1.291;
%               1.137	-2.471	-1.73;
%               0.769	-2.502	-1.09;
%              -0.008	-2.52	-1.58;
%              -0.668	-2.552	-1.04;
%              -1.226	-2.515	-1.534;
%              -1.61	-2.485	-1.8;
%              -1.628	-2.483	-1.134;
%              -2.7	-2.545	-1.522;
%              -3.513	-1.435	-1.347;
%              -3.41	-0.651	-0.48;
%              -3.442	-0.002	-0.672;
%              -3.528	-0.634	-1.449;
%              -3.508	 0.206	-1.36];

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
    drawDrone(xhat_array(i,:),t(i),landmarks, P_array(i,:,:),truth_array(i,:));
end
%% Plot Estimates Vs Truth
for i = 1:length(t)
    truth_array(i,9) = wrap(truth_array(i,9));
    xhat_array(i,9) = wrap(xhat_array(i,9));
end

figure(2);clf
subplot(2,3,1)
hold on
plot(t,truth_array(:,1),'r')
plot(t,xhat_array(:,1),'b')
ylabel('North (m)')
xlabel('time (s)')
title('Position Estimates Vs. Truth')
subplot(2,3,2)
hold on
plot(t,truth_array(:,2),'r')
plot(t,xhat_array(:,2),'b')
ylabel('East (m)')
xlabel('time (s)')
subplot(2,3,3)
hold on
plot(t,truth_array(:,3),'r')
plot(t,xhat_array(:,3),'b')
legend('Truth','Estimate')
ylabel('Down (m)')
xlabel('time (s)')

% subplot(3,3,2)
% hold on
% plot(t,truth_array(:,4),'r')
% plot(t,xhat_array(:,4),'b')
% ylabel('u (rad/s)')
% title('Velocity Estimates Vs. Truth')
% subplot(3,3,5)
% hold on
% plot(t,truth_array(:,5),'r')
% plot(t,xhat_array(:,5),'b')
% ylabel('v (rad/s)')
% subplot(3,3,8)
% hold on
% plot(t,truth_array(:,6),'r')
% plot(t,xhat_array(:,6),'b')
% ylabel('w (rad/s)')
% xlabel('time (s)')

subplot(2,3,4)
hold on
plot(t,truth_array(:,7),'r')
plot(t,xhat_array(:,7),'b')
ylabel('\phi (rad)')
xlabel('time (s)')
title('Attitude Estimates Vs. Truth')

subplot(2,3,5)
hold on
plot(t,truth_array(:,8),'r')
plot(t,xhat_array(:,8),'b')
ylabel('\theta (rad)')
xlabel('time (s)')
subplot(2,3,6)
hold on
plot(t,truth_array(:,9),'r')
plot(t,xhat_array(:,9),'b')
ylabel('\psi (rad)')
xlabel('time (s)')

%% Plot Errors

truth_array(:,9) = unwrap(truth_array(:,9));
xhat_array(:,9) = unwrap(xhat_array(:,9));

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