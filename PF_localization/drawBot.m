function drawBot(u)

    % inputs to function
    % x        : x pos
    % y        : y pos
    % theta    : angle of rod
    x = u(1);
    y = u(2);
    theta = u(3);
    t = u(4);
    Ts = u(5);
    
    %drawing parameters
    R= .75; %radius of bot
   
    % define persistent variables 
    persistent bot_handle
    persistent heading_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t==Ts
        figure(1), clf
        frame_width=12;
        l1 = [6;4];
        l2 = [-7;8];
        l3 = [6;-4];
        landmark = [l1,l2,l3];
        hold on
        plot(landmark(1,1),landmark(2,1),'ob','LineWidth', 2); % plot landmark
        plot(landmark(1,2),landmark(2,2),'or','LineWidth', 2); % plot hinge
        plot(landmark(1,3),landmark(2,3),'oy','LineWidth', 2); % plot landmark
        bot_handle  = updateBot(x, y, R, []);
        heading_handle = updateHeading(x,y,theta,R,[]);
        axis([-frame_width, frame_width, -frame_width, frame_width]);
    
        
    % at every other time step, redraw ball and beam
    else 
        updateBot(x, y, R, bot_handle);
        updateHeading(x,y,theta,R,heading_handle);
    end
end

   
%
%=======================================================================
% drawheading
% draw the beam
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = updateHeading(x,y,theta, R, handle)
  
  X = [x, x+R*cos(theta)];
  Y = [y, y+R*sin(theta)];

  if isempty(handle)
    handle = plot(X,Y,'m','LineWidth',2);
  else
    handle.XData = X;
    handle.YData = Y;
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