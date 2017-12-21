clc
clear
load 'thor_3D_Loc_bag1.mat'

t = t(500:end);
P_array = P_array(500:end,:,:);
xhat_array = xhat_array(500:end,:);
truth_array = truth_array(500:end,:);
t = t-t(1);

tt = t(1:5:end);
P = P_array(1:5:end,:,:);
xhat = xhat_array(1:5:end,:);
% truth_array = truth_array(1:3:end,:);

[b,a] = butter(12,0.2,'low');           % IIR filter design
    xhat= filtfilt(b,a,xhat);

% xhat = filtfilt(xhat);
truth = truth_array(1:5:end,:);

% P_array1 = zeros(739,36,36);
% P_array1(:,1:9,1:9) = P_array(:,1:9,1:9);
% P_array1(:,10:36,10:36) = P_array(:,13:39,13:39);
% P_array = P_array1;
% 
% xhat_array1 = zeros(739,36);
% xhat_array1(:,1:9) = xhat_array(:,1:9);
% xhat_array1(:,10:36) = xhat_array(:,13:39);
% xhat_array = xhat_array1;

% landmarks =[-0.51	 2.302	-1.31;
%              1.455	 2.493	-1.486;
%              3.92	 1.333	-1.498;
%              3.964	-1.753	-1.566;
%              2.916	-2.543	-1.537;
%              1.181	-2.471	-1.581;
%             -1.593	-2.488	-1.572;
%             -3.528	-0.658	-1.461;
%             -2.023	 2.462	-1.492];

landmarks =[ -2.529	 1.496	-0.833;
             -2.531	 2.464	-1.583;
             -1.664	 2.469	-1.083;
             -1.514	 2.469	-1.677;
             -0.522	 2.484	-1.078;
              0.62	 2.498	-0.932;
              0.608	 2.501	-1.699;
              1.485	 2.498	-1.488;
              2.263	 2.515	-1.095;
              3.917	 1.293	-1.488;
              3.934	 0.644	-1.57;
              3.948	-0.152	-1.358;
              3.956	-0.833	-1.708;
              3.971	-1.703	-1.597;
              2.862	-2.495	-1.502;
              1.798	-2.485	-1.291;
              1.137	-2.471	-1.73;
              0.769	-2.502	-1.09;
             -0.008	-2.52	-1.58;
             -0.668	-2.552	-1.04;
             -1.226	-2.515	-1.534;
             -1.61	-2.485	-1.8;
             -1.628	-2.483	-1.134;
             -2.7	-2.545	-1.522;
             -3.513	-1.435	-1.347;
             -3.41	-0.651	-0.48;
             -3.442	-0.002	-0.672;
             -3.528	-0.634	-1.449;
             -3.508	 0.206	-1.36];

pn_e = xhat(:,1);
pe_e = xhat(:,2);
pd_e = xhat(:,3);
phi_e  = xhat(:,4);
theta_e  = xhat(:,5);
psi_e  = xhat(:,6);


N = 100;
for i = 1:length(tt)
    if i<N+1
        past_xhat = xhat(1:i,1:3);
        past_truth = truth(1:i,1:3);
    else 
        past_xhat = xhat(i-N:i,1:3);
        past_truth = truth(i-N:i,1:3);
    end
    drawDrone_6d(xhat(i,:),tt(i),landmarks, P(i,:,:),truth(i,:),past_xhat,past_truth);
end
%% Plot Estimates Vs Truth
for i = 1:length(tt)
    truth(i,9) = wrap(truth(i,9));
    xhat(i,6) = wrap(xhat(i,6));
end

figure(2);clf
subplot(2,3,1)
hold on
plot(tt,truth(:,1),'r')
plot(tt,xhat(:,1),'b')
ylabel('North (m)')
xlabel('time (s)')
title('Position Estimates Vs. Truth')
subplot(2,3,2)
hold on
plot(tt,truth(:,2),'r')
plot(tt,xhat(:,2),'b')
ylabel('East (m)')
xlabel('time (s)')
subplot(2,3,3)
hold on
plot(tt,truth(:,3),'r')
plot(tt,xhat(:,3),'b')
legend('Truth','Estimate')
ylabel('Down (m)')
xlabel('time (s)')

% subplot(3,3,2)
% hold on
% plot(tt,truth_array(:,4),'r')
% plot(tt,xhat_array(:,4),'b')
% ylabel('u (rad/s)')
% title('Velocity Estimates Vs. Truth')
% subplot(3,3,5)
% hold on
% plot(tt,truth_array(:,5),'r')
% plot(tt,xhat_array(:,5),'b')
% ylabel('v (rad/s)')
% subplot(3,3,8)
% hold on
% plot(tt,truth_array(:,6),'r')
% plot(tt,xhat_array(:,6),'b')
% ylabel('w (rad/s)')
% xlabel('time (s)')

subplot(2,3,4)
hold on
plot(tt,truth(:,7),'r')
plot(tt,xhat(:,4),'b')
ylabel('\phi (rad)')
xlabel('time (s)')
title('Attitude Estimates Vs. Truth')

subplot(2,3,5)
hold on
plot(tt,truth(:,8),'r')
plot(tt,xhat(:,5),'b')
ylabel('\theta (rad)')
xlabel('time (s)')
subplot(2,3,6)
hold on
plot(tt,truth(:,9),'r')
plot(tt,xhat(:,6),'b')
ylabel('\psi (rad)')
xlabel('time (s)')

% %% Plot Errors
% 
% % truth(:,9) = unwrap(truth(:,9));
% % xhat(:,6) = unwrap(xhat(:,6));
% 
% err = truth(:,1:3)-xhat(:,1:3);
% err2 = truth(:,7:9)-xhat(:,4:6);
% err = [err, err2];
% for i = 1:6
%     errcov(:,i) = sqrt(P(:,i,i));
% end
% 
% figure(3);clf
% subplot(2,3,1)
% hold on
% plot (tt,err(:,1),'b')
% plot (tt,2*errcov(:,1),'r')
% plot (tt,3*errcov(:,1),'y')
% plot (tt,-2*errcov(:,1),'r')
% plot (tt,-3*errcov(:,1),'y')
% ylabel('Pn (m)')
% xlabel('t (s)')
% 
% subplot(2,3,2)
% hold on
% plot (tt,err(:,2),'b')
% plot (tt,2*errcov(:,2),'r')
% plot (tt,3*errcov(:,2),'y')
% plot (tt,-2*errcov(:,2),'r')
% plot (tt,-3*errcov(:,2),'y')
% ylabel('Pe (m)')
% xlabel('t (s)')
% 
% subplot(2,3,3)
% hold on
% plot (tt,err(:,3),'b')
% plot (tt,2*errcov(:,3),'r')
% plot (tt,3*errcov(:,3),'y')
% plot (tt,-2*errcov(:,3),'r')
% plot (tt,-3*errcov(:,3),'y')
% ylabel('Pd (m)')
% xlabel('t (s)')
% legend('error','2 \sigma bound', '3 \sigma bound')
% 
% % subplot(3,3,2)
% % hold on
% % plot (tt,err(:,4),'b')
% % plot (tt,2*errcov(:,4),'r')
% % plot (tt,3*errcov(:,4),'y')
% % plot (tt,-2*errcov(:,4),'r')
% % plot (tt,-3*errcov(:,4),'y')
% % ylabel('u (rad/s)')
% % 
% % subplot(3,3,5)
% % hold on
% % plot (tt,err(:,5),'b')
% % plot (tt,2*errcov(:,5),'r')
% % plot (tt,3*errcov(:,5),'y')
% % plot (tt,-2*errcov(:,5),'r')
% % plot (tt,-3*errcov(:,5),'y')
% % ylabel('v (rad/s)')
% % 
% % subplot(3,3,8)
% % hold on
% % plot (tt,err(:,6),'b')
% % plot (tt,2*errcov(:,6),'r')
% % plot (tt,3*errcov(:,6),'y')
% % plot (tt,-2*errcov(:,6),'r')
% % plot (tt,-3*errcov(:,6),'y')
% % ylabel('w (rad/s)')
% % xlabel('t (s)')
% 
% subplot(2,3,4)
% hold on
% plot (tt,err(:,4),'b')
% plot (tt,2*errcov(:,4),'r')
% plot (tt,3*errcov(:,4),'y')
% plot (tt,-2*errcov(:,4),'r')
% plot (tt,-3*errcov(:,4),'y')
% ylabel('\phi (rad)')
% xlabel('t (s)')
% 
% 
% subplot(2,3,5)
% hold on
% plot (tt,err(:,5),'b')
% plot (tt,2*errcov(:,5),'r')
% plot (tt,3*errcov(:,5),'y')
% plot (tt,-2*errcov(:,5),'r')
% plot (tt,-3*errcov(:,5),'y')
% ylabel('\theta (rad)')
% xlabel('t (s)')
% 
% subplot(2,3,6)
% hold on
% plot (tt,err(:,6),'b')
% plot (tt,2*errcov(:,6),'r')
% plot (tt,3*errcov(:,6),'y')
% plot (tt,-2*errcov(:,6),'r')
% plot (tt,-3*errcov(:,6),'y')
% ylabel('\psi (rad)')
% xlabel('t (s)')