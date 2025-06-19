function F=U4(in)
   % System parameters
    m = 0.5;
    d = 0.3;
    ob = 1; 
    g = 9.8;
    Ix = 0.006; Iy = 0.006; Iz = 0.012; 
    % Kfax = 5.567e-4; Kfay = 5.567e-4; Kfaz = 6.354e-4;
    % Kftx = 5.567e-4; Kfty = 5.567e-4; Kftz = 6.354e-4; 
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

x5d = in(1);  % x5d is the desired angular position
x5d_d = in(2); 
x5d_dd = in(3);
x5 = in(4); % x5 is the angular position (psi)   
x6 = in(5);  % x6 is the angular velocity (psi_dot)
x2 = in(6);
x4 = in(7);
fn = in(8);
B = in(9);
V = in(10);

hh1 = 5;
mu = 5;
C = (B*mu + 5*(hh1 + 4*mu^2))*V;

 
e=x5-x5d; % error
de = x6 - x5d_d; % error_dot

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

  U4 = 1 / b3 * (-fn - a5 * x2 * x4 -k1*de - e + x5d_dd -B*abs(s)^0.5 * tanh(s/k2) -k2*s - C);
 


F  =  [U4,B_dot, v_dot]';
