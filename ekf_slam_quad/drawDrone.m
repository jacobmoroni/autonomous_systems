function drawDrone(mu,t,landmarks,P,truth,V,F,patchcolors)

    % process inputs to function
%     pn       = mu(1);       % inertial North position     
%     pe       = mu(2);       % inertial East position
%     pd       = mu(3);           
%     u        = mu(4);       
%     v        = mu(5);       
%     w        = mu(6); 
%     phi      = mu(7);       % roll angle         
%     theta    = mu(8);       % pitch angle     
%     psi      = mu(9);       % yaw angle     
%     p        = mu(10);       % roll rate
%     q        = mu(11);       % pitch rate     
%     r        = mu(12);       % yaw rate   

    pn       = truth(1);       % inertial North position     
    pe       = truth(2);       % inertial East position
    pd       = truth(3);           
    u        = truth(4);       
    v        = truth(5);       
    w        = truth(6); 
    phi      = truth(7);       % roll angle         
    theta    = truth(8);       % pitch angle     
    psi      = truth(9);       % yaw angle 
    
    t        = t;       % time
    MM = length(landmarks);

    % define persistent variables 
    persistent vehicle_handle;
    persistent estimate_handle
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        estimate_handle = updateEst(mu,P,MM,landmarks,[]);
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               []);
        
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
       
        plot_size = 5;
        axis([ -3, 3, -4, 4, -1, 2.5]);
        
        grid on
        hold on
        
        for i=1:length(landmarks)
            if abs(landmarks(i,1))>3.4
                offset = 90;
            else
                offset = 0;
            end
            scale = .1;
            vector = plot_squares(landmarks(i,:),offset,scale);
            fill3(vector(2,:),vector(1,:),-vector(3,:),[0 0 0])
%             plot3 (landmarks(i,2),landmarks(i,1),-landmarks(i,3),'or')
        end
        fill([-3,3,3,-3],[-4,-4,4,4],[.8 .8 .8]);
%         set(h,'facealpha',.5)
        
    % at every other time step, redraw vehicle
    else
        
        updateEst(mu,P,MM,landmarks,estimate_handle);
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
        
        
%         plot3(mu(2),mu(1),-mu(3),'.b')
        plot3(truth(2),truth(1),-truth(3),'.r')

    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle)
    handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
  else
    handle.Vertices = V';
    handle.Faces = F;
%     xlim(handle.Parent,[pe-7, pe+7]);
%     ylim(handle.Parent,[pn-7, pn+7]);
%     zlim(handle.Parent,[-(pd+7), -(pd-7)]);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle2 = updateEst(mu,P,MM,landmarks,handle2)
xdata = [];
ydata = [];
zdata = [];
    for ll = 1: MM
       landmark_est = landmarks(ll,:);
%        landmark_est= [mu(9+3*ll-2),mu(9+3*ll-1),mu(9+3*ll)];
       c(:,:) = P(1,9+3*ll-2:9+3*ll,9+3*ll-2:9+3*ll);
        
%        c = 3*chol(c);
       [xx,yy,zz] = error_ellipse(c,landmark_est,.15); 
%        xx = real(sqrt(xx));
%        yy = real(sqrt(yy));
%        zz = real(sqrt(zz));
       xdata = [xdata;NaN;xx];
       ydata = [ydata;NaN;yy];
       zdata = [zdata;NaN;zz];
    end
    if isempty(handle2)
%        ax = axes;
%        f = ax.Parent;
%        p = uipanel('Parent',f);
       handle2 = plot3(ydata,xdata,zdata,'color',[0 0 0]);
    else
       handle2.XData = ydata;
       handle2.YData = xdata;
       handle2.ZData = -zdata;
       drawnow
    end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody
scale = .2;
base_l = 1 * scale;
base_h = .25 *scale;
arm_l = 1*scale;
arm_t = .05*scale;
leg_d = base_l/2*.8;
leg_h = .5*scale;
leg_t = arm_t/2;
motor_h = .25*scale;

%prop motor
[X_motor,Y_motor,Z_motor] = cylinder(.1,15);
X_motor = X_motor*scale;
Y_motor = Y_motor*scale;
X_prop = X_motor*5;
Y_prop = Y_motor*5;


% Define the vertices (physical location of vertices
V = [...
    %base
    -base_l/2, base_l/2,0;...   % pt 1
     base_l/2, base_l/2,0;...   % pt 2
     base_l/2,-base_l/2,0;...   % pt 3
    -base_l/2,-base_l/2,0;...   % pt 4
     0       ,  0      ,-base_h;...  % pt 5
     0       ,  0      ,base_h;... % pt 6
     
     %arms top coordinates
     0       , arm_t*2    ,-arm_t;...  % pt 7
    arm_l - arm_t, arm_l + arm_t, -arm_t;...  % pt 8
    arm_l + arm_t, arm_l - arm_t, -arm_t;...  % pt 9
    arm_t*2    , 0   ,-arm_t;...  % pt 10
    arm_l + arm_t, -arm_l + arm_t, -arm_t;...  % pt 11
    arm_l- arm_t, -arm_l - arm_t, -arm_t;...  % pt 12
    0       , -arm_t*2    ,-arm_t;...  % pt 13
    -arm_l + arm_t, -arm_l - arm_t, -arm_t;...  % pt 14
    -arm_l- arm_t, -arm_l + arm_t, -arm_t;...  % pt 15
    -arm_t*2, 0    ,-arm_t;...  % pt 16
    -arm_l - arm_t, arm_l - arm_t, -arm_t;...  % pt 17
    -arm_l + arm_t, arm_l + arm_t, -arm_t;...  % pt 18
    
    %arms bottom coordinates
         0       , arm_t*2    ,arm_t;...  % pt 19
    arm_l - arm_t, arm_l + arm_t, arm_t;...  % pt 20
    arm_l + arm_t, arm_l - arm_t, arm_t;...  % pt 21
    arm_t*2    , 0   ,arm_t;...  % pt 22
    arm_l + arm_t, -arm_l + arm_t, arm_t;...  % pt 23
    arm_l- arm_t, -arm_l - arm_t, arm_t;...  % pt 24
    0       , -arm_t*2    ,arm_t;...  % pt 25
    -arm_l + arm_t, -arm_l - arm_t, arm_t;...  % pt 26
    -arm_l- arm_t, -arm_l + arm_t, arm_t;...  % pt 27
    -arm_t*2, 0    ,arm_t;...  % pt 28
    -arm_l - arm_t, arm_l - arm_t, arm_t;...  % pt 29
    -arm_l + arm_t, arm_l + arm_t, arm_t;...  % pt 30
    
    %leg coordinates
    leg_d+leg_t,leg_d+leg_t,0;... %pt 31
    leg_d+leg_t,leg_d-leg_t,0;... %pt 32
    leg_d-leg_t,leg_d-leg_t,0;... %pt 33
    leg_d-leg_t,leg_d+leg_t,0;... %pt 34
    
    base_l/2+leg_t,base_l/2+leg_t,leg_h;... %pt 35
    base_l/2+leg_t,base_l/2-leg_t,leg_h;... %pt 36
    base_l/2-leg_t,base_l/2-leg_t,leg_h;... %pt 37
    base_l/2-leg_t,base_l/2+leg_t,leg_h;... %pt 38
    
    leg_d+leg_t,-leg_d+leg_t,0;... %pt 39
    leg_d+leg_t,-leg_d-leg_t,0;... %pt 40
    leg_d-leg_t,-leg_d-leg_t,0;... %pt 41
    leg_d-leg_t,-leg_d+leg_t,0;... %pt 42
    
    base_l/2+leg_t,-base_l/2+leg_t,leg_h;... %pt 43
    base_l/2+leg_t,-base_l/2-leg_t,leg_h;... %pt 44
    base_l/2-leg_t,-base_l/2-leg_t,leg_h;... %pt 45
    base_l/2-leg_t,-base_l/2+leg_t,leg_h;... %pt 46
    
    -leg_d+leg_t,-leg_d+leg_t,0;... %pt 47
    -leg_d+leg_t,-leg_d-leg_t,0;... %pt 48
    -leg_d-leg_t,-leg_d-leg_t,0;... %pt 49
    -leg_d-leg_t,-leg_d+leg_t,0;... %pt 50
    
    -base_l/2+leg_t,-base_l/2+leg_t,leg_h;... %pt 51
    -base_l/2+leg_t,-base_l/2-leg_t,leg_h;... %pt 52
    -base_l/2-leg_t,-base_l/2-leg_t,leg_h;... %pt 53
    -base_l/2-leg_t,-base_l/2+leg_t,leg_h;... %pt 54
    
    -leg_d+leg_t,leg_d+leg_t,0;... %pt 55
    -leg_d+leg_t,leg_d-leg_t,0;... %pt 56
    -leg_d-leg_t,leg_d-leg_t,0;... %pt 57
    -leg_d-leg_t,leg_d+leg_t,0;... %pt 58
    
    -base_l/2+leg_t,base_l/2+leg_t,leg_h;... %pt 59
    -base_l/2+leg_t,base_l/2-leg_t,leg_h;... %pt 60
    -base_l/2-leg_t,base_l/2-leg_t,leg_h;... %pt 61
    -base_l/2-leg_t,base_l/2+leg_t,leg_h;... %pt 62

    arm_l + X_motor(1,1), arm_l + Y_motor(1,1), arm_t;... %63
    arm_l + X_motor(1,2), arm_l + Y_motor(1,2), arm_t;...  %64
    arm_l + X_motor(1,3), arm_l + Y_motor(1,3), arm_t;...  %65
    arm_l + X_motor(1,4), arm_l + Y_motor(1,4), arm_t;... %66
    arm_l + X_motor(1,5), arm_l + Y_motor(1,5), arm_t;... %67
    arm_l + X_motor(1,6), arm_l + Y_motor(1,6), arm_t;... %68
    arm_l + X_motor(1,7), arm_l + Y_motor(1,7), arm_t;... %69
    arm_l + X_motor(1,8), arm_l + Y_motor(1,8), arm_t;... %70
    arm_l + X_motor(1,9), arm_l + Y_motor(1,9), arm_t;... %71
    arm_l + X_motor(1,10), arm_l + Y_motor(1,10), arm_t;... %72
    arm_l + X_motor(1,11), arm_l + Y_motor(1,11), arm_t;... %73
    arm_l + X_motor(1,12), arm_l + Y_motor(1,12), arm_t;... %74
    arm_l + X_motor(1,13), arm_l + Y_motor(1,13), arm_t;... %75
    arm_l + X_motor(1,14), arm_l + Y_motor(1,14), arm_t;... %76
    arm_l + X_motor(1,15), arm_l + Y_motor(1,15), arm_t;... %77
    
    arm_l + X_motor(1,1), arm_l + Y_motor(1,1), -motor_h+arm_t;... %78
    arm_l + X_motor(1,2), arm_l + Y_motor(1,2), -motor_h+arm_t;... %79
    arm_l + X_motor(1,3), arm_l + Y_motor(1,3), -motor_h+arm_t;... %80
    arm_l + X_motor(1,4), arm_l + Y_motor(1,4), -motor_h+arm_t;... %81
    arm_l + X_motor(1,5), arm_l + Y_motor(1,5), -motor_h+arm_t;... %82
    arm_l + X_motor(1,6), arm_l + Y_motor(1,6), -motor_h+arm_t;... %83
    arm_l + X_motor(1,7), arm_l + Y_motor(1,7), -motor_h+arm_t;... %84
    arm_l + X_motor(1,8), arm_l + Y_motor(1,8), -motor_h+arm_t;... %85
    arm_l + X_motor(1,9), arm_l + Y_motor(1,9), -motor_h+arm_t;... %86
    arm_l + X_motor(1,10), arm_l + Y_motor(1,10), -motor_h+arm_t;... %87
    arm_l + X_motor(1,11), arm_l + Y_motor(1,11), -motor_h+arm_t;... %88
    arm_l + X_motor(1,12), arm_l + Y_motor(1,12), -motor_h+arm_t;... %89
    arm_l + X_motor(1,13), arm_l + Y_motor(1,13), -motor_h+arm_t;... %90
    arm_l + X_motor(1,14), arm_l + Y_motor(1,14), -motor_h+arm_t;... %91
    arm_l + X_motor(1,15), arm_l + Y_motor(1,15), -motor_h+arm_t;... %92
    
    arm_l + X_motor(1,1), -arm_l + Y_motor(1,1), arm_t;... %63+30
    arm_l + X_motor(1,2), -arm_l + Y_motor(1,2), arm_t;...  %64+30
    arm_l + X_motor(1,3), -arm_l + Y_motor(1,3), arm_t;...  %65+30
    arm_l + X_motor(1,4), -arm_l + Y_motor(1,4), arm_t;... %66+30
    arm_l + X_motor(1,5), -arm_l + Y_motor(1,5), arm_t;... %67+30
    arm_l + X_motor(1,6), -arm_l + Y_motor(1,6), arm_t;... %68+30
    arm_l + X_motor(1,7), -arm_l + Y_motor(1,7), arm_t;... %69+30
    arm_l + X_motor(1,8), -arm_l + Y_motor(1,8), arm_t;... %70+30
    arm_l + X_motor(1,9), -arm_l + Y_motor(1,9), arm_t;... %71+30
    arm_l + X_motor(1,10), -arm_l + Y_motor(1,10), arm_t;... %72+30
    arm_l + X_motor(1,11), -arm_l + Y_motor(1,11), arm_t;... %73+30
    arm_l + X_motor(1,12), -arm_l + Y_motor(1,12), arm_t;... %74+30
    arm_l + X_motor(1,13), -arm_l + Y_motor(1,13), arm_t;... %75+30
    arm_l + X_motor(1,14), -arm_l + Y_motor(1,14), arm_t;... %76+30
    arm_l + X_motor(1,15), -arm_l + Y_motor(1,15), arm_t;... %77+30
    
    arm_l + X_motor(1,1), -arm_l + Y_motor(1,1), -motor_h+arm_t;... %78+30
    arm_l + X_motor(1,2), -arm_l + Y_motor(1,2), -motor_h+arm_t;... %79+30
    arm_l + X_motor(1,3), -arm_l + Y_motor(1,3), -motor_h+arm_t;... %80+30
    arm_l + X_motor(1,4), -arm_l + Y_motor(1,4), -motor_h+arm_t;... %81+30
    arm_l + X_motor(1,5), -arm_l + Y_motor(1,5), -motor_h+arm_t;... %82+30
    arm_l + X_motor(1,6), -arm_l + Y_motor(1,6), -motor_h+arm_t;... %83+30
    arm_l + X_motor(1,7), -arm_l + Y_motor(1,7), -motor_h+arm_t;... %84+30
    arm_l + X_motor(1,8), -arm_l + Y_motor(1,8), -motor_h+arm_t;... %85+30
    arm_l + X_motor(1,9), -arm_l + Y_motor(1,9), -motor_h+arm_t;... %86+30
    arm_l + X_motor(1,10), -arm_l + Y_motor(1,10), -motor_h+arm_t;... %87+30
    arm_l + X_motor(1,11), -arm_l + Y_motor(1,11), -motor_h+arm_t;... %88+30
    arm_l + X_motor(1,12), -arm_l + Y_motor(1,12), -motor_h+arm_t;... %89+30
    arm_l + X_motor(1,13), -arm_l + Y_motor(1,13), -motor_h+arm_t;... %90+30
    arm_l + X_motor(1,14), -arm_l + Y_motor(1,14), -motor_h+arm_t;... %91+30
    arm_l + X_motor(1,15), -arm_l + Y_motor(1,15), -motor_h+arm_t;... %92+30
    
    -arm_l + X_motor(1,1), arm_l + Y_motor(1,1), arm_t;... %63+60
    -arm_l + X_motor(1,2), arm_l + Y_motor(1,2), arm_t;...  %64+60
    -arm_l + X_motor(1,3), arm_l + Y_motor(1,3), arm_t;...  %65+60
    -arm_l + X_motor(1,4), arm_l + Y_motor(1,4), arm_t;... %66+60
    -arm_l + X_motor(1,5), arm_l + Y_motor(1,5), arm_t;... %67+60
    -arm_l + X_motor(1,6), arm_l + Y_motor(1,6), arm_t;... %68+60
    -arm_l + X_motor(1,7), arm_l + Y_motor(1,7), arm_t;... %69+60
    -arm_l + X_motor(1,8), arm_l + Y_motor(1,8), arm_t;... %70+60
    -arm_l + X_motor(1,9), arm_l + Y_motor(1,9), arm_t;... %71+60
    -arm_l + X_motor(1,10), arm_l + Y_motor(1,10), arm_t;... %72+60
    -arm_l + X_motor(1,11), arm_l + Y_motor(1,11), arm_t;... %73+60
    -arm_l + X_motor(1,12), arm_l + Y_motor(1,12), arm_t;... %74+60
    -arm_l + X_motor(1,13), arm_l + Y_motor(1,13), arm_t;... %75+60
    -arm_l + X_motor(1,14), arm_l + Y_motor(1,14), arm_t;... %76+60
    -arm_l + X_motor(1,15), arm_l + Y_motor(1,15), arm_t;... %77+60
    
    -arm_l + X_motor(1,1), arm_l + Y_motor(1,1), -motor_h+arm_t;... %78+60
    -arm_l + X_motor(1,2), arm_l + Y_motor(1,2), -motor_h+arm_t;... %79+60
    -arm_l + X_motor(1,3), arm_l + Y_motor(1,3), -motor_h+arm_t;... %80+60
    -arm_l + X_motor(1,4), arm_l + Y_motor(1,4), -motor_h+arm_t;... %81+60
    -arm_l + X_motor(1,5), arm_l + Y_motor(1,5), -motor_h+arm_t;... %82+60
    -arm_l + X_motor(1,6), arm_l + Y_motor(1,6), -motor_h+arm_t;... %83+60
    -arm_l + X_motor(1,7), arm_l + Y_motor(1,7), -motor_h+arm_t;... %84+60
    -arm_l + X_motor(1,8), arm_l + Y_motor(1,8), -motor_h+arm_t;... %85+60
    -arm_l + X_motor(1,9), arm_l + Y_motor(1,9), -motor_h+arm_t;... %86+60
    -arm_l + X_motor(1,10), arm_l + Y_motor(1,10), -motor_h+arm_t;... %87+60
    -arm_l + X_motor(1,11), arm_l + Y_motor(1,11), -motor_h+arm_t;... %88+60
    -arm_l + X_motor(1,12), arm_l + Y_motor(1,12), -motor_h+arm_t;... %89+60
    -arm_l + X_motor(1,13), arm_l + Y_motor(1,13), -motor_h+arm_t;... %90+60
    -arm_l + X_motor(1,14), arm_l + Y_motor(1,14), -motor_h+arm_t;... %91+60
    -arm_l + X_motor(1,15), arm_l + Y_motor(1,15), -motor_h+arm_t;... %92+60
    
    -arm_l + X_motor(1,1), -arm_l + Y_motor(1,1), arm_t;... %63+90
    -arm_l + X_motor(1,2), -arm_l + Y_motor(1,2), arm_t;...  %64+90
    -arm_l + X_motor(1,3), -arm_l + Y_motor(1,3), arm_t;...  %65+90
    -arm_l + X_motor(1,4), -arm_l + Y_motor(1,4), arm_t;... %66+90
    -arm_l + X_motor(1,5), -arm_l + Y_motor(1,5), arm_t;... %67+90
    -arm_l + X_motor(1,6), -arm_l + Y_motor(1,6), arm_t;... %68+90
    -arm_l + X_motor(1,7), -arm_l + Y_motor(1,7), arm_t;... %69+90
    -arm_l + X_motor(1,8), -arm_l + Y_motor(1,8), arm_t;... %70+90
    -arm_l + X_motor(1,9), -arm_l + Y_motor(1,9), arm_t;... %71+90
    -arm_l + X_motor(1,10), -arm_l + Y_motor(1,10), arm_t;... %72+90
    -arm_l + X_motor(1,11), -arm_l + Y_motor(1,11), arm_t;... %73+90
    -arm_l + X_motor(1,12), -arm_l + Y_motor(1,12), arm_t;... %74+90
    -arm_l + X_motor(1,13), -arm_l + Y_motor(1,13), arm_t;... %75+90
    -arm_l + X_motor(1,14), -arm_l + Y_motor(1,14), arm_t;... %76+90
    -arm_l + X_motor(1,15), -arm_l + Y_motor(1,15), arm_t;... %77+90
    
    -arm_l + X_motor(1,1), -arm_l + Y_motor(1,1), -motor_h+arm_t;... %78+90
    -arm_l + X_motor(1,2), -arm_l + Y_motor(1,2), -motor_h+arm_t;... %79+90
    -arm_l + X_motor(1,3), -arm_l + Y_motor(1,3), -motor_h+arm_t;... %80+90
    -arm_l + X_motor(1,4), -arm_l + Y_motor(1,4), -motor_h+arm_t;... %81+90
    -arm_l + X_motor(1,5), -arm_l + Y_motor(1,5), -motor_h+arm_t;... %82+90
    -arm_l + X_motor(1,6), -arm_l + Y_motor(1,6), -motor_h+arm_t;... %83+90
    -arm_l + X_motor(1,7), -arm_l + Y_motor(1,7), -motor_h+arm_t;... %84+90
    -arm_l + X_motor(1,8), -arm_l + Y_motor(1,8), -motor_h+arm_t;... %85+90
    -arm_l + X_motor(1,9), -arm_l + Y_motor(1,9), -motor_h+arm_t;... %86+90
    -arm_l + X_motor(1,10), -arm_l + Y_motor(1,10), -motor_h+arm_t;... %87+90
    -arm_l + X_motor(1,11), -arm_l + Y_motor(1,11), -motor_h+arm_t;... %88+90
    -arm_l + X_motor(1,12), -arm_l + Y_motor(1,12), -motor_h+arm_t;... %89+90
    -arm_l + X_motor(1,13), -arm_l + Y_motor(1,13), -motor_h+arm_t;... %90+90
    -arm_l + X_motor(1,14), -arm_l + Y_motor(1,14), -motor_h+arm_t;... %91+90
    -arm_l + X_motor(1,15), -arm_l + Y_motor(1,15), -motor_h+arm_t;... %92+90
    
    %Prop fr (+120)
    arm_l + X_prop(1,1), arm_l + Y_prop(1,1), -motor_h+arm_t;... %63
    arm_l + X_prop(1,2), arm_l + Y_prop(1,2), -motor_h+arm_t;...  %64
    arm_l + X_prop(1,3), arm_l + Y_prop(1,3), -motor_h+arm_t;...  %65
    arm_l + X_prop(1,4), arm_l + Y_prop(1,4), -motor_h+arm_t;... %66
    arm_l + X_prop(1,5), arm_l + Y_prop(1,5), -motor_h+arm_t;... %67
    arm_l + X_prop(1,6), arm_l + Y_prop(1,6), -motor_h+arm_t;... %68
    arm_l + X_prop(1,7), arm_l + Y_prop(1,7), -motor_h+arm_t;... %69
    arm_l + X_prop(1,8), arm_l + Y_prop(1,8), -motor_h+arm_t;... %70
    arm_l + X_prop(1,9), arm_l + Y_prop(1,9), -motor_h+arm_t;... %71
    arm_l + X_prop(1,10), arm_l + Y_prop(1,10), -motor_h+arm_t;... %72
    arm_l + X_prop(1,11), arm_l + Y_prop(1,11), -motor_h+arm_t;... %73
    arm_l + X_prop(1,12), arm_l + Y_prop(1,12), -motor_h+arm_t;... %74
    arm_l + X_prop(1,13), arm_l + Y_prop(1,13), -motor_h+arm_t;... %75
    arm_l + X_prop(1,14), arm_l + Y_prop(1,14), -motor_h+arm_t;... %76
    arm_l + X_prop(1,15), arm_l + Y_prop(1,15), -motor_h+arm_t;... %77
    
    arm_l + X_prop(1,1), arm_l + Y_prop(1,1), -motor_h;... %78
    arm_l + X_prop(1,2), arm_l + Y_prop(1,2), -motor_h;... %79
    arm_l + X_prop(1,3), arm_l + Y_prop(1,3), -motor_h;... %80
    arm_l + X_prop(1,4), arm_l + Y_prop(1,4), -motor_h;... %81
    arm_l + X_prop(1,5), arm_l + Y_prop(1,5), -motor_h;... %82
    arm_l + X_prop(1,6), arm_l + Y_prop(1,6), -motor_h;... %83
    arm_l + X_prop(1,7), arm_l + Y_prop(1,7), -motor_h;... %84
    arm_l + X_prop(1,8), arm_l + Y_prop(1,8), -motor_h;... %85
    arm_l + X_prop(1,9), arm_l + Y_prop(1,9), -motor_h;... %86
    arm_l + X_prop(1,10), arm_l + Y_prop(1,10), -motor_h;... %87
    arm_l + X_prop(1,11), arm_l + Y_prop(1,11), -motor_h;... %88
    arm_l + X_prop(1,12), arm_l + Y_prop(1,12), -motor_h;... %89
    arm_l + X_prop(1,13), arm_l + Y_prop(1,13), -motor_h;... %90
    arm_l + X_prop(1,14), arm_l + Y_prop(1,14), -motor_h;... %91
    arm_l + X_prop(1,15), arm_l + Y_prop(1,15), -motor_h;... %92
    
    %prop br (+150)
    arm_l + X_prop(1,1), -arm_l + Y_prop(1,1), -motor_h+arm_t;... %63+30
    arm_l + X_prop(1,2), -arm_l + Y_prop(1,2), -motor_h+arm_t;...  %64+30
    arm_l + X_prop(1,3), -arm_l + Y_prop(1,3), -motor_h+arm_t;...  %65+30
    arm_l + X_prop(1,4), -arm_l + Y_prop(1,4), -motor_h+arm_t;... %66+30
    arm_l + X_prop(1,5), -arm_l + Y_prop(1,5), -motor_h+arm_t;... %67+30
    arm_l + X_prop(1,6), -arm_l + Y_prop(1,6), -motor_h+arm_t;... %68+30
    arm_l + X_prop(1,7), -arm_l + Y_prop(1,7), -motor_h+arm_t;... %69+30
    arm_l + X_prop(1,8), -arm_l + Y_prop(1,8), -motor_h+arm_t;... %70+30
    arm_l + X_prop(1,9), -arm_l + Y_prop(1,9), -motor_h+arm_t;... %71+30
    arm_l + X_prop(1,10), -arm_l + Y_prop(1,10), -motor_h+arm_t;... %72+30
    arm_l + X_prop(1,11), -arm_l + Y_prop(1,11), -motor_h+arm_t;... %73+30
    arm_l + X_prop(1,12), -arm_l + Y_prop(1,12), -motor_h+arm_t;... %74+30
    arm_l + X_prop(1,13), -arm_l + Y_prop(1,13), -motor_h+arm_t;... %75+30
    arm_l + X_prop(1,14), -arm_l + Y_prop(1,14), -motor_h+arm_t;... %76+30
    arm_l + X_prop(1,15), -arm_l + Y_prop(1,15), -motor_h+arm_t;... %77+30
    
    arm_l + X_prop(1,1), -arm_l + Y_prop(1,1), -motor_h;... %78+30
    arm_l + X_prop(1,2), -arm_l + Y_prop(1,2), -motor_h;... %79+30
    arm_l + X_prop(1,3), -arm_l + Y_prop(1,3), -motor_h;... %80+30
    arm_l + X_prop(1,4), -arm_l + Y_prop(1,4), -motor_h;... %81+30
    arm_l + X_prop(1,5), -arm_l + Y_prop(1,5), -motor_h;... %82+30
    arm_l + X_prop(1,6), -arm_l + Y_prop(1,6), -motor_h;... %83+30
    arm_l + X_prop(1,7), -arm_l + Y_prop(1,7), -motor_h;... %84+30
    arm_l + X_prop(1,8), -arm_l + Y_prop(1,8), -motor_h;... %85+30
    arm_l + X_prop(1,9), -arm_l + Y_prop(1,9), -motor_h;... %86+30
    arm_l + X_prop(1,10), -arm_l + Y_prop(1,10), -motor_h;... %87+30
    arm_l + X_prop(1,11), -arm_l + Y_prop(1,11), -motor_h;... %88+30
    arm_l + X_prop(1,12), -arm_l + Y_prop(1,12), -motor_h;... %89+30
    arm_l + X_prop(1,13), -arm_l + Y_prop(1,13), -motor_h;... %90+30
    arm_l + X_prop(1,14), -arm_l + Y_prop(1,14), -motor_h;... %91+30
    arm_l + X_prop(1,15), -arm_l + Y_prop(1,15), -motor_h;... %92+30
    
    %prop fl (+180)
    -arm_l + X_prop(1,1), arm_l + Y_prop(1,1), -motor_h+arm_t;... %63+60
    -arm_l + X_prop(1,2), arm_l + Y_prop(1,2), -motor_h+arm_t;...  %64+60
    -arm_l + X_prop(1,3), arm_l + Y_prop(1,3), -motor_h+arm_t;...  %65+60
    -arm_l + X_prop(1,4), arm_l + Y_prop(1,4), -motor_h+arm_t;... %66+60
    -arm_l + X_prop(1,5), arm_l + Y_prop(1,5), -motor_h+arm_t;... %67+60
    -arm_l + X_prop(1,6), arm_l + Y_prop(1,6), -motor_h+arm_t;... %68+60
    -arm_l + X_prop(1,7), arm_l + Y_prop(1,7), -motor_h+arm_t;... %69+60
    -arm_l + X_prop(1,8), arm_l + Y_prop(1,8), -motor_h+arm_t;... %70+60
    -arm_l + X_prop(1,9), arm_l + Y_prop(1,9), -motor_h+arm_t;... %71+60
    -arm_l + X_prop(1,10), arm_l + Y_prop(1,10), -motor_h+arm_t;... %72+60
    -arm_l + X_prop(1,11), arm_l + Y_prop(1,11), -motor_h+arm_t;... %73+60
    -arm_l + X_prop(1,12), arm_l + Y_prop(1,12), -motor_h+arm_t;... %74+60
    -arm_l + X_prop(1,13), arm_l + Y_prop(1,13), -motor_h+arm_t;... %75+60
    -arm_l + X_prop(1,14), arm_l + Y_prop(1,14), -motor_h+arm_t;... %76+60
    -arm_l + X_prop(1,15), arm_l + Y_prop(1,15), -motor_h+arm_t;... %77+60
    
    -arm_l + X_prop(1,1), arm_l + Y_prop(1,1), -motor_h;... %78+60
    -arm_l + X_prop(1,2), arm_l + Y_prop(1,2), -motor_h;... %79+60
    -arm_l + X_prop(1,3), arm_l + Y_prop(1,3), -motor_h;... %80+60
    -arm_l + X_prop(1,4), arm_l + Y_prop(1,4), -motor_h;... %81+60
    -arm_l + X_prop(1,5), arm_l + Y_prop(1,5), -motor_h;... %82+60
    -arm_l + X_prop(1,6), arm_l + Y_prop(1,6), -motor_h;... %83+60
    -arm_l + X_prop(1,7), arm_l + Y_prop(1,7), -motor_h;... %84+60
    -arm_l + X_prop(1,8), arm_l + Y_prop(1,8), -motor_h;... %85+60
    -arm_l + X_prop(1,9), arm_l + Y_prop(1,9), -motor_h;... %86+60
    -arm_l + X_prop(1,10), arm_l + Y_prop(1,10), -motor_h;... %87+60
    -arm_l + X_prop(1,11), arm_l + Y_prop(1,11), -motor_h;... %88+60
    -arm_l + X_prop(1,12), arm_l + Y_prop(1,12), -motor_h;... %89+60
    -arm_l + X_prop(1,13), arm_l + Y_prop(1,13), -motor_h;... %90+60
    -arm_l + X_prop(1,14), arm_l + Y_prop(1,14), -motor_h;... %91+60
    -arm_l + X_prop(1,15), arm_l + Y_prop(1,15), -motor_h;... %92+60
    
    %prop bl (+210)
    -arm_l + X_prop(1,1), -arm_l + Y_prop(1,1), -motor_h+arm_t;... %63+90
    -arm_l + X_prop(1,2), -arm_l + Y_prop(1,2), -motor_h+arm_t;...  %64+90
    -arm_l + X_prop(1,3), -arm_l + Y_prop(1,3), -motor_h+arm_t;...  %65+90
    -arm_l + X_prop(1,4), -arm_l + Y_prop(1,4), -motor_h+arm_t;... %66+90
    -arm_l + X_prop(1,5), -arm_l + Y_prop(1,5), -motor_h+arm_t;... %67+90
    -arm_l + X_prop(1,6), -arm_l + Y_prop(1,6), -motor_h+arm_t;... %68+90
    -arm_l + X_prop(1,7), -arm_l + Y_prop(1,7), -motor_h+arm_t;... %69+90
    -arm_l + X_prop(1,8), -arm_l + Y_prop(1,8), -motor_h+arm_t;... %70+90
    -arm_l + X_prop(1,9), -arm_l + Y_prop(1,9), -motor_h+arm_t;... %71+90
    -arm_l + X_prop(1,10), -arm_l + Y_prop(1,10), -motor_h+arm_t;... %72+90
    -arm_l + X_prop(1,11), -arm_l + Y_prop(1,11), -motor_h+arm_t;... %73+90
    -arm_l + X_prop(1,12), -arm_l + Y_prop(1,12), -motor_h+arm_t;... %74+90
    -arm_l + X_prop(1,13), -arm_l + Y_prop(1,13), -motor_h+arm_t;... %75+90
    -arm_l + X_prop(1,14), -arm_l + Y_prop(1,14), -motor_h+arm_t;... %76+90
    -arm_l + X_prop(1,15), -arm_l + Y_prop(1,15), -motor_h+arm_t;... %77+90
    
    -arm_l + X_prop(1,1), -arm_l + Y_prop(1,1), -motor_h;... %78+90
    -arm_l + X_prop(1,2), -arm_l + Y_prop(1,2), -motor_h;... %79+90
    -arm_l + X_prop(1,3), -arm_l + Y_prop(1,3), -motor_h;... %80+90
    -arm_l + X_prop(1,4), -arm_l + Y_prop(1,4), -motor_h;... %81+90
    -arm_l + X_prop(1,5), -arm_l + Y_prop(1,5), -motor_h;... %82+90
    -arm_l + X_prop(1,6), -arm_l + Y_prop(1,6), -motor_h;... %83+90
    -arm_l + X_prop(1,7), -arm_l + Y_prop(1,7), -motor_h;... %84+90
    -arm_l + X_prop(1,8), -arm_l + Y_prop(1,8), -motor_h;... %85+90
    -arm_l + X_prop(1,9), -arm_l + Y_prop(1,9), -motor_h;... %86+90
    -arm_l + X_prop(1,10), -arm_l + Y_prop(1,10), -motor_h;... %87+90
    -arm_l + X_prop(1,11), -arm_l + Y_prop(1,11), -motor_h;... %88+90
    -arm_l + X_prop(1,12), -arm_l + Y_prop(1,12), -motor_h;... %89+90
    -arm_l + X_prop(1,13), -arm_l + Y_prop(1,13), -motor_h;... %90+90
    -arm_l + X_prop(1,14), -arm_l + Y_prop(1,14), -motor_h;... %91+90
    -arm_l + X_prop(1,15), -arm_l + Y_prop(1,15), -motor_h;... %92+90
    ]';

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 5, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base top
        2, 3, 5, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base top
        3, 4, 5, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base top
        1, 4, 5, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base top
        1, 2, 6, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base bottom
        2, 3, 6, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base bottom
        3, 4, 6, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base bottom
        1, 4, 6, NaN,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  % base bottom
        
        7 , 8, 9,10,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % arm fr
        7 , 8,20,19,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        10, 9,21,22,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        19,20,21,22,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   
        
        10,11,12,13,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % arm fl
        10,11,23,22,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        13,12,24,25,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        22,23,24,25,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...  
        
        13,14,15,16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % arm bl
        13,14,26,25,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        15,16,28,27,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        25,26,27,28,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   
        
        16,17,18, 7,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % arm br
        17,16,28,29,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        18, 7,19,30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        28,29,30,19,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;... 
        
        32,36,37,33,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % leg fr
        33,37,38,34,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        34,38,35,31,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        31,32,36,35,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        35,36,37,38,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        32+8,36+8,37+8,33+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % leg br
        33+8,37+8,38+8,34+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        34+8,38+8,35+8,31+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        31+8,32+8,36+8,35+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        35+8,36+8,37+8,38+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        32+16,36+16,37+16,33+16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % leg bl
        33+16,37+16,38+16,34+16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        34+16,38+16,35+16,31+16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        31+16,32+16,36+16,35+16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        35+16,36+16,37+16,38+16,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        32+24,36+24,37+24,33+24,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...   % leg fl
        33+24,37+24,38+24,34+24,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        34+24,38+24,35+24,31+24,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        31+24,32+24,36+24,35+24,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        35+24,36+24,37+24,38+24,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        63,64,79,78,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN; ...              %motor fr
        63+1,64+1,79+1,78+1,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+2,64+2,79+2,78+2,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+3,64+3,79+3,78+3,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+4,64+4,79+4,78+4,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+5,64+5,79+5,78+5,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+6,64+6,79+6,78+6,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+7,64+7,79+7,78+7,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+8,64+8,79+8,78+8,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+9,64+9,79+9,78+9,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+10,64+10,79+10,78+10,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+11,64+11,79+11,78+11,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+12,64+12,79+12,78+12,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+13,64+13,79+13,78+13,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+14,63,78,78+14,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        63+30,64+30,79+30,78+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN; ...  %motor br
        63+1+30,64+1+30,79+1+30,78+1+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+2+30,64+2+30,79+2+30,78+2+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+3+30,64+3+30,79+3+30,78+3+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+4+30,64+4+30,79+4+30,78+4+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+5+30,64+5+30,79+5+30,78+5+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+6+30,64+6+30,79+6+30,78+6+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+7+30,64+7+30,79+7+30,78+7+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+8+30,64+8+30,79+8+30,78+8+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+9+30,64+9+30,79+9+30,78+9+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+10+30,64+10+30,79+10+30,78+10+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+11+30,64+11+30,79+11+30,78+11+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+12+30,64+12+30,79+12+30,78+12+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+13+30,64+13+30,79+13+30,78+13+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+14+30,63+30,78+30,78+14+30,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        63+60,64+60,79+60,78+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN; ...  %motor fl
        63+1+60,64+1+60,79+1+60,78+1+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+2+60,64+2+60,79+2+60,78+2+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+3+60,64+3+60,79+3+60,78+3+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+4+60,64+4+60,79+4+60,78+4+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+5+60,64+5+60,79+5+60,78+5+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+6+60,64+6+60,79+6+60,78+6+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+7+60,64+7+60,79+7+60,78+7+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+8+60,64+8+60,79+8+60,78+8+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+9+60,64+9+60,79+9+60,78+9+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+10+60,64+10+60,79+10+60,78+10+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+11+60,64+11+60,79+11+60,78+11+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+12+60,64+12+60,79+12+60,78+12+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+13+60,64+13+60,79+13+60,78+13+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+14+60,63+60,78+60,78+14+60,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        63+90,64+90,79+90,78+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN; ...  %motor bl
        63+1+90,64+1+90,79+1+90,78+1+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+2+90,64+2+90,79+2+90,78+2+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+3+90,64+3+90,79+3+90,78+3+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+4+90,64+4+90,79+4+90,78+4+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+5+90,64+5+90,79+5+90,78+5+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+6+90,64+6+90,79+6+90,78+6+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+7+90,64+7+90,79+7+90,78+7+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+8+90,64+8+90,79+8+90,78+8+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+9+90,64+9+90,79+9+90,78+9+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+10+90,64+10+90,79+10+90,78+10+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+11+90,64+11+90,79+11+90,78+11+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+12+90,64+12+90,79+12+90,78+12+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+13+90,64+13+90,79+13+90,78+13+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        63+14+90,63+90,78+90,78+14+90,...
        NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN;...
        
        183,184,185,186,187,188,189,190,191,192,193,194,195,196,197;...
        183+30,184+30,185+30,186+30,187+30,188+30,189+30,190+30,191+30,192+30,193+30,194+30,195+30,196+30,197+30;...
        183+60,184+60,185+60,186+60,187+60,188+60,189+60,190+60,191+60,192+60,193+60,194+60,195+60,196+60,197+60;...
        183+90,184+90,185+90,186+90,187+90,188+90,189+90,190+90,191+90,192+90,193+90,194+90,195+90,196+90,197+90;...
         ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  myblack = [0, 0, 0];
  mywhite = [1, 1, 1];

  facecolors = [...
    myred;...    % body top
    myblack;...    % body top
    myred;...    % body top
    myblack;...    % body top
    myred;...    % body bottom
    myblack;...    % body bottom
    myred;...    % body bottom
    myblack;...    % body bottom
    myblack;...   % arm fr
    myblack;...   % arm fr
    myblack;...   % arm fr
    myblack;...   % arm fr
    myblack;...   % arm fl
    myblack;...   % arm fl
    myblack;...   % arm fl
    myblack;...   % arm fl
    myblack;...   % arm bl
    myblack;...   % arm bl
    myblack;...   % arm bl
    myblack;...   % arm bl
    myblack;...   % arm br
    myblack;...   % arm br
    myblack;...   % arm br
    myblack;...   % arm br
    
    %legs
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    myyellow;...
    
    %motors
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    myblack;...
    
    %props
    myblue;...
    myblue;...
    myred;...
    myred;...
    ];
end