function F=Uy(in)

k1 = 8; k2 = 10;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x9d = in(1); % x9d is the desired trajectory along Y-axis
x9d_d = in(2);
x9d_dd = in(3);
x9 = in(4); % x9 is the position along Z-axis
x10 = in(5); % x10 is the velocity along Z-axis
U1 = in(6);
fn = in(7);
B = in(8);
V = in(9);

hh1 = 5;
mu = 5;
C = (B*mu + 5*(hh1 + 4*mu^2))*V;
m = 0.5; % mass


e=x9-x9d; % error
de = x10 - x9d_d; % error_dot

s = k1*e + de;  % sliding surface

Sm = 0.01;
hh1 = 3;
rr = 4;
vv = 0.01;
if s > Sm
    B_dot = hh1*sqrt(rr/2)*tanh(s-vv); % B_dot is an adaptive SMC gain
else
    B_dot = 0;
end

v_dot = tanh(s/k2);

% Control law design

if U1==0
    u = 0;
else
   u=m/U1 * (-fn - e + x9d_dd - k1*de -B*abs(s)^0.5 * tanh(s/k2) -k2*s - C);
  
end



F  =  [u,B_dot, v_dot]';
