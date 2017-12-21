% % ellipsetester
% C(:,:) = P_array(700,13:15,13:15);
% % C = [1.2,.1,.5;
% %     .1,1,.5;
% %     .1,.5,1.5];
% mu = xhat_array(700,13:15);
% 
% [xx,yy,zz] = error_ellipse(C,mu,.5);
% plot3(yy,xx,-zz,'r')


offset = 90;
scale = .25;
vector = plot_squares(center,offset,scale);
fill3(vector(1,:),vector(2,:),vector(3,:),[0 0 0])
