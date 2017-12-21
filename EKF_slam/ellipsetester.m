% ellipsetester
C(:,:) = P_array(700,13:15,13:15);
% C = [1.2,.1,.5;
%     .1,1,.5;
%     .1,.5,1.5];
mu = xhat_array(700,13:15);

h = error_ellipse(C,mu,.5);

