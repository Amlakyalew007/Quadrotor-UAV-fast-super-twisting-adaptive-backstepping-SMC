function F=ARBF_x(in)

k1 = 8; k2 = 10; gamma = 0.06;

%%%%%%%%%%%%%%%%%%%%%%%
%   INPUTS    % 
%%%%%%%%%%%%%%%%%%%%%%%
x7d = in(1);
x7d_d = in(2);
x7d_dd = in(3);
x7=in(4);
x8=in(5);

W_h=in(6:24); % weight


e = x7 - x7d; % error
de = x8 - x7d_d; % error_dot

s = k1*e + de; % sliding surface


cij=[-1.4  -1   -0.8  -0.4  -0.2   0    0.4  0.8    1     1.2    1.6    2     2.4  2.8    3   3.2  3.4  3.8   4;
     -8   -7    -6   -6.5  -6   -5.5   -5   -4   -3.5    -2    -1.5   -0.5     0   0.5    1   1.5   2   2.5   3
];%19 center
  

% bj=[ 2; 2; 2; 2; 2; 2; 2]; spread 

bj=[0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2]*20;%19

J3=[e;de];
h=zeros(19,1);

% RBF activation Function (Gaussian)
for j=1:1:19
    h(j)=exp(-norm(J3-cij(:,j))^(2)/(2*bj(j)^(2)));
end

fn = W_h'*h; % the radial basis function neural network output

W_hd = 1/gamma * s * h; % adaptive whight (gamma is the learning rate)

F  =  [fn, W_hd']';
dbstop if error