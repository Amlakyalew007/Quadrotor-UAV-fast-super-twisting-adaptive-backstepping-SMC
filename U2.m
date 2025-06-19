function F=U2(in)
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

x1d = in(1); % x1d is the desired angular position 
x1d_d = in(2);
x1d_dd = in(3);
x1 = in(4);  % x1 is the angular position (phi)
x2 = in(5);  % x2 is the angular velocity (ph_dot)
x4 = in(6);
x6 = in(7);
fn = in(8);
B = in(9);
V = in(10);

hh1 = 5;
mu = 5;
C = (B*mu + 5*(hh1 + 4*mu^2))*V;

 
e=x1-x1d; % error
de = x2 - x1d_d; % error dot

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

  U2 = 1 / b1 * (-fn - a1 * x4 * x6 - a2 * ob * x4 -k1*de - e + x1d_dd -B*abs(s)^0.5 * tanh(s/k2) -k2*s - C);



F  =  [U2,B_dot, v_dot]';
