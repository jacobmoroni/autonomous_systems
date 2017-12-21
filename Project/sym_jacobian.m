syms t p psi az g pn pe pd u v w real
R_p_u = [cos(t)*cos(psi), sin(p)*sin(t)*cos(psi)-cos(p)*sin(psi),cos(p)*sin(t)*cos(psi)+sin(p)*sin(psi);
         cos(t)*sin(psi), sin(p)*sin(t)*sin(psi)+cos(p)*cos(psi),cos(p)*sin(t)*sin(psi)-sin(p)*cos(psi);
         -sin(t), sin(p)*cos(t), cos(p)*cos(t)];
P_dot = [cos(p)*sin(t)*az;
        -sin(p)*az;
        g+cos(p)*cos(t)*az];
uvw_dot = R_p_u.'*P_dot;

x = [p,t,psi];
pn_dot = diff(pn,
f = [pn_dot, pe_dot, pd_dot, uvw_dot, p_dot, t_dot, psi_dot]
J = jacobian(uvw_dot,x)

% syms mx my x y t real
% 
% meas = [sqrt((mx-x)^2+(my-y)^2);
%         atan2(my-y,mx-x)-t];
%     
% state = [x,y];
% 
% jacobian(meas,state)
