function F=U3(in)
  % System parameters
    m = 0.5;
    d = 0.3;
    ob = 1;
    g = 9.8;
    Ix = 0.006; Iy = 0.006; Iz = 0.012; 
    
    Jr =5e-5;

    % System dynamics coefficients
    a1 = (Iy - Iz) / Ix;
    a2 = -Jr / Ix;
    a3 = (Iz - Ix) / Iy;
    a4 = Jr / Iy;
    a5 = (Ix - Iy) / Iz;
    b1 = d / Ix;
    b2 = d / Iy;
    b3 = 1 / Iz;

k1 = 8; k2 = 10;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%

x3d = in(1); % x3d is the desired angular position
x3d_d = in(2);
x3d_dd = in(3);
x3 = in(4); % x5d is the angular position (theta)
x4 = in(5); % x5d is the angular velocity (theta_dot)
x2 = in(6);
x6 = in(7);
fn = in(8);
B = in(9);
V = in(10);

hh1 = 5;
mu = 5;
C = (B*mu + 5*(hh1 + 4*mu^2))*V;

 
e=x3-x3d; % error
de = x4 - x3d_d; % error_dot

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

  U3 = 1 / b2 * (-fn - a3 * x2 * x6 - a4 * ob * x2 -k1*de - e + x3d_dd -B*abs(s)^0.5 * tanh(s/k2) -k2*s - C);
 


F  =  [U3,B_dot, v_dot]';
