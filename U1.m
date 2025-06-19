function F=U1(in)
    m = 0.5; % mass
    g = 9.8; % gravity

k1 = 8; k2 = 10;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x11d = in(1); % x11d is the desired trajectory along Z-axis
x11d_d = in(2);
x11d_dd = in(3);
x11 = in(4); % x11 is the position along Z-axis
x12 = in(5); % x12 is the velocity along Z-axis
x1 = in(6);
x3 = in(7);
fn = in(8);
B = in(9); 
V = in(10); 

hh1 = 5;
mu = 5;
C = (B*mu + 5*(hh1 + 4*mu^2))*V; % C is an adaptive gain

 
e=x11-x11d; % error
de = x12 - x11d_d; % error_dot

s = k1*e + de; % sliding surface
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

  U1 =m / (cos(x1) * cos(x3)) * (-fn + g -k1*de - e + x11d_dd -B*abs(s)^0.5 * tanh(s/k2) -k2*s - C);



F  =  [U1,B_dot, v_dot]';
