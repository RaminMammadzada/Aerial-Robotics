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
% mass: 0.1800
% I: [3x3 double]
% invI: [3x3 double]
% gravity: 9.8100
% arm_length: 0.0860
% minF: 0
% maxF: 3.5316

g = params.gravity;
m = params.mass;

% Thrust

%Derivative constant
Kdz = 30;
%Proportion constant
Kpz = 1000;

%Current values
z = state.pos(3);
z_dot = state.vel(3);

%Desired values
z_t = des_state.pos(3);
z_dot_t = des_state.vel(3);
z_ddot_t = des_state.acc(3);

%Compute thrust
F = m*(g + z_t + Kdz*(z_dot_t-z_dot) + Kpz*(z_t-z));

% Moment

%Constants
Kdx = 15;
Kpx = 70;
Kdy = 12;
Kpy = 70;


%Current state
rx = state.pos(1);
rx_dot = state.vel(1);

ry = state.pos(2);
ry_dot = state.vel(2);

%Desired state
rx_des = des_state.pos(1);
rx_dot_des = des_state.vel(1);
rx_ddot_des = des_state.acc(1);

ry_des = des_state.pos(2);
ry_dot_des = des_state.vel(2);
ry_ddot_des = des_state.acc(2);
%-------------
%Current roll, pitch and yaw
phi = state.rot(1);
theta = state.rot(2);
sigh = state.rot(3);

%Desired roll, pitch and yaw
phi_des = 0; %Unused
theta_des = 0; %Unused
sigh_des = des_state.yaw;
%-------------

%Commanded State
rx_ddot_c =rx_ddot_des + Kdx*(rx_dot_des-rx_dot)+ Kpx*(rx_des-rx);
ry_ddot_c =ry_ddot_des + Kdy*(ry_dot_des-ry_dot)+ Kpy*(ry_des-ry);
%--- ---
%Commanded roll, pitch and yaw
%phi_c = %Use eqn 14-a from handout
%theta_c = %Use eqn 14-b from handout
phi_c = (1/g)*(rx_ddot_c*sin(sigh_des)-ry_ddot_c*cos(sigh_des));
theta_c = (1/g)*(rx_ddot_c*cos(sigh_des)+ry_ddot_c*sin(sigh_des));
sigh_c = sigh_des;
%--- ---

%Constants
Kdphi = 0.02;
Kpphi = 0.3;
Kdtheta = 0.02;
Kptheta = 0.3;
Kdsigh = 0.02;
Kpsigh = 0.4;


%Current Angular velocities
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

%Desired Angular velocities
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

%Comanded Angular velocities
p_c = 0;
q_c = 0;
r_c = r_des;

%Compute moment elements
u2_11 = (Kpphi*(phi_c-phi)+Kdphi*(p_c-p));
u2_21 = (Kptheta*(theta_c-theta)+Kdtheta*(q_c-q));
u2_31 = (Kpsigh*(sigh_c-sigh)+Kdsigh*(r_c-r));


M = [u2_11;u2_21;u2_31];

% =================== Your code ends here ===================

end