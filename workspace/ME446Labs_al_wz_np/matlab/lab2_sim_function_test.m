function qddot = lab2_sim_function_test(q, qdot, tau)

theta2 = q(1);
theta3 = q(2);

omega2 = qdot(1);
omega3 = qdot(2);

p1 = 0.03;
p2 = 0.0128;
p3 = 0.0076;
p4 = 0.0753;
p5 = 0.0298;

% gravity acceleration constant
g = 9.81

D = [p1 -p3*sin(theta3-theta2); -p3*sin(theta3 - theta2) p2];
C = [0 -p3*cos(theta3 - theta2)*omega3; p3*cos(theta3 - theta2)*omega2 0];
G = [-p4*g*sin(theta2); -p5*g*cos(theta3)];

qddot = inv(D)*tau - inv(D)*C*qdot - inv(D)*G;
end