function y = plant_x(in)
m = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x7 = in(1);  % x7 is the position along X-axis
x8 = in(2);  % x8 is the velocity along X-axis
d2 = in(3); % disturbance
Ux = in(4); % control input
U1 = in(5);

% X sub-system of the quadrotor UAV (state space representation) 
 dx7 = x8 ;
 dx8 = (Ux * (U1 / m)) + d2;


y  =  [dx7, dx8]';
