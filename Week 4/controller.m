function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

%des_theta = (1/params.gravity)*(com_state.acc(1)*cos(state.rot(3)) + com_state.acc(2)*sin(state.rot(3)));

%des_phi = (1/params.gravity)* (com_state.acc(1)*sin(state.rot(3)) - com_state.acc(2)*cos(state.rot(3)));
kd_z=4;
kp_z=9;
% Thurst
F = params.mass*params.gravity + params.mass*[ kd_z*( des_state.vel(3) - state.vel(3) ) + kp_z*( des_state.pos(3) - state.pos(3) ) ] ;
% Moment
e1_ddot=des_state.acc(1);
q=des_state.yaw;
e2_ddot=des_state.acc(2);
des_phi=(1/params.gravity)*(e1_ddot*sin(q)-e2_ddot*cos(q));
des_theta=(1/params.gravity)*(e1_ddot*cos(q)+e2_ddot*sin(q));
M=[3,1]
ephi=des_phi-state.rot(1);
etheta=des_theta-state.rot(2);
epsi=des_state.yaw-state.rot(3);
ep=state.omega(1);
eq=state.omega(2);
e_r=des_state.yawdot-state.omega(3);
kpphi=0.75;
kptheta=0.75;
kppsi=0.75;
kdphi=1.1;
kdtheta=0.75;
kdpsi=1.1;
M = [(kpphi*ephi-kdphi*ep);(kptheta*etheta-kdtheta*eq);(kppsi*epsi+kdpsi*e_r)];


% =================== Your code ends here ===================

end
