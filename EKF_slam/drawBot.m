function drawBot(u,landmark)

    % inputs to function
    % x        : x pos
    % y        : y pos
    % theta    : angle of rod
    x = u(1);
    y = u(2);
    theta = u(3);
    t = u(4);
    
    
    %drawing parameters
    R= .75;
    
    
    
    % define persistent variables 
    persistent bot_handle
    persistent heading_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t<=.1
        figure(1), clf
        frame_width=12;
%         landmark = [6, -7, 6;
%                     4, 8, -4];
        hold on
        for i = 1:length(landmark)
             plot (landmark(1,i),landmark(2,i),'or')
        end
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