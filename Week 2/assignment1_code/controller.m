function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

Kp=145;
Kv=20;

%u = params.mass * ( s_des_double_diff + Kp*e + Kv*e_diff + params.gravity);
u=params.mass*( Kp*(s_des(1)-s(1)) + Kv*(s_des(2)-s(2)) + params.gravity );
% FILL IN YOUR CODE HERE


end

