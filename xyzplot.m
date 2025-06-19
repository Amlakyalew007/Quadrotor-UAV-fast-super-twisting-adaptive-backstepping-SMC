
figure;
plot3(out.x(:,1), out.y(:,1), out.z(:,1),'b');
hold on;
plot3((3 + cos(5*out.t)).*cos(out.t), (3 + cos(5*out.t)).*sin(out.t), sin(5*out.t),'r');
title('X, Y , Z Trajectory');
legend('Obtained Trajectory', 'Desired Trajectory');
xlabel('x(m)') 
ylabel('y(m)')
zlabel('z(m)')
grid on;

figure;

plot3(out.phi(:,1), out.theta(:,1), out.psi(:,1),'b');
hold on;
plot3(sin(out.t),cos(out.t),0.1*out.t,'r');
title('Roll, Pitch, Yaw Trajectory');
legend('Obtained Trajectory', 'Desired Trajectory');
xlabel('ϕ(rad)') 
ylabel('θ(rad)')
zlabel('ψ(rad)')
grid on;