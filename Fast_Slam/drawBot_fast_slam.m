function drawBot_fast_slam(Y,t,landmark,X,view_angle)

    % inputs to function
    % x        : x pos
    % y        : y pos
    % theta    : angle of rod
    x = Y(1);
    y = Y(2);
    theta = Y(3);
    MM = length(landmark);
    x_true = X(1);
    y_true = X(2);
    theta_true = X(3);
    %drawing parameters
    R= .75;
    
    
    
    % define persistent variables 
    persistent bot_handle
    persistent heading_handle
    persistent estimate_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t<=.1
        figure(1), clf
        frame_width=12;
        hold on
        for i = 1:length(landmark)
             plot (landmark(1,i),landmark(2,i),'or')
        end
        bot_handle  = updateBot(x, y, R, []);
        heading_handle = updateHeading(x,y,theta,view_angle,R,[]);
        estimate_handle = updateEst(Y,MM,[]);
        axis([-frame_width, frame_width, -frame_width, frame_width]);
    
        
    % at every other time step, redraw ball and beam
    else 
        
        updateBot(x, y, R, bot_handle);
        updateHeading(x,y,theta,view_angle,R,heading_handle);
        updateEst(Y,MM,estimate_handle);
%         plot(x,y,'.r')
%         for i = 1:length(landmark)
%              plot (landmark(1,i),landmark(2,i),'or')
%         end
        plot(x,y,'.b')
        plot(x_true,y_true,'.r')
%         for ll = 1: MM
%            landmark_x= mu(3+2*ll-1);
%            landmark_y= mu(3+2*ll);
%            c= Sigma(3+2*ll-1:3+2*ll,3+2*ll-1:3+2*ll);
%            [xx,yy,zz] = getellipse(c,landmark_x,landmark_y);
%            plot(xx,yy)
%         end
    end
end

   
%
%=======================================================================
% drawheading
% draw the beam
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = updateHeading(x,y,theta,view_angle, R, handle)
  
  X = [x, x+R*cos(theta)];
  Y = [y, y+R*sin(theta)];
  X_l = [x, x+4*R*cos(theta+view_angle)];
  X_r = [x, x+4*R*cos(theta-view_angle)];
  Y_l = [y, y+4*R*sin(theta+view_angle)];
  Y_r = [y, y+4*R*sin(theta-view_angle)];

  if isempty(handle)
    handle = plot(X,Y,'m');%,'LineWidth',2);
  else
    handle.XData = [X,NaN,X_r,NaN,X_l];
    handle.YData = [Y,NaN,Y_r,NaN,Y_l];
    drawnow
  end
end
 
%
%=======================================================================
% drawBall
% draw the ball
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = updateBot(x, y, R,handle)
  
  ang = 0:.01:2*pi;
  botx=R*cos(ang);
  boty=R*sin(ang);
  
  
  X = x+botx;%z*cos(theta)-(R+gap)*sin(theta)+x;
  Y = y+boty;%z*sin(theta)+(R+gap)*cos(theta)+y;

  if isempty(handle)
    handle = fill(X, Y, 'g');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
  end
end

function handle = updateEst(Y,MM,handle)
xdata = [];
ydata = [];
    for ll = 1: MM
       Sigma(1,:)=Y(3+6*ll-3:3+6*ll-2); %Extract Sigma Matricies from Y
       Sigma(2,:)=Y(3+6*ll-1:3+6*ll);
       landmark_est= [Y(3+6*ll-5),Y(3+6*ll-4)];
       c = Sigma;
       if c ~= [0,0;0,0]
           c = 3*chol(c);
           [xx,yy,zz] = getellipse(c,landmark_est(1),landmark_est(2)); 
           xdata = [xdata;NaN;xx];
           ydata = [ydata;NaN;yy];
       end
    end
    if isempty(handle)
       handle = plot(xdata,ydata,'color',[0 0 0]);
    else
       handle.XData = xdata;
       handle.YData = ydata;
       drawnow
    end
end