function drawBot(u)

    % inputs to function
    % x        : x pos
    % y        : y pos
    % theta    : heading angle
    x = u(1);
    y = u(2);
    theta = u(3);
    t = u(4);

    
    %drawing parameters
    R= .5; %radius of bot
   
    % define persistent variables 
    persistent bot_handle
    persistent heading_handle
    
    
    % first time function is called, initialize plot and persistent vars
    if t<.05
        load ('processed_data.mat', 'landmarks');
        figure(1), clf
        frame_width=9;
        hold on
        for m = 1:length(landmarks)
        plot(landmarks(m,1),landmarks(m,2),'ob','LineWidth', 2); % plot landmark
        end
        bot_handle  = updateBot(x, y, R, []);
        heading_handle = updateHeading(x,y,theta,R,[]);
        axis([-frame_width+2, frame_width+2, -frame_width+7, frame_width+7]);
        title('Movement of turtlebot')
    
        
    % at every other time step, redraw ball and beam
    else 
        updateBot(x, y, R, bot_handle);
        updateHeading(x,y,theta,R,heading_handle);
    end
end
%=======================================================================
% drawheading
% draw the beam
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
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
 
function handle = updateBot(x, y, R,handle)
  
  ang = 0:.3:2*pi;
  botx=R*cos(ang);
  boty=R*sin(ang);
  
  X = x+botx;
  Y = y+boty;

  if isempty(handle)
    handle = fill(X, Y, 'g');
  else
    handle.XData = X;
    handle.YData = Y;
    drawnow
    plot(x,y,'.r')
  end
end