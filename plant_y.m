function y = plant_y(in)

m = 0.5; % mass

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x9 = in(1); % x9 is the position along Y-axis
x10 = in(2);  % x10 is the velocity along Y-axis
d2 = in(3); % disturbance
Uy = in(4);
U1 = in(5);

% Y sub-system of the quadrotor UAV (state space representation) 
 dx9 = x10 ;
 dx10 = (Uy * (U1 / m)) + d2;


y  =  [dx9, dx10]';
