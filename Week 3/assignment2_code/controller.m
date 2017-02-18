function [ F, M ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

Kv_z=30 %Kv_z=3
Kp_z=10 %Kp_z=20
Kv_phi=25 %Kv_phi=10
Kp_phi=2000 %Kp_phi=2000
Kv_y=3 %Kv_y=1
Kp_y=80 %Kp_y=50
F = params.mass*(params.gravity + des_state.acc(2) + Kv_z*(des_state.vel(2)-state.vel(2)) + Kp_z*(des_state.pos(2)-state.pos(2)));
phi_des=-(1/params.gravity) * (des_state.acc(1) + Kv_y*(des_state.vel(1)-state.vel(1)) + Kp_y*(des_state.pos(1)-state.pos(1))) 
%phi_des=-0.5
M = params.Ixx*(Kv_phi*(-state.omega) + Kp_phi*(phi_des-state.rot));

% 

end

