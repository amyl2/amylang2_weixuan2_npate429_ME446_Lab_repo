function [theta, theta_dot, theta_ddot] = DesiredTheta(time)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
clc;

syms t
f1 = -t^3 +1.5*t^2;
f2 = t^3-4.5*t^2+6*t-2;
f1dot = diff(f1);
f2dot = diff(f2);
f1ddot = diff(f1dot);
f2ddot = diff(f2dot);
zero = 0*t;

%Plotting
% Plot theta
figure
fplot(f1,[0 1],'-b')
hold on
fplot(f2, [1 2],'-b')
fplot(zero,[2 3],'-b')
hold off
ylabel("Theta [rad]")
xlabel("Time [s]")


figure
fplot(f1dot,[0 1],'-b')
hold on
fplot(f2dot, [1 2],'-b')
fplot(zero,[2 3],'-b')
hold off
ylabel("Theta_dot [rad/s]")
xlabel("Time [s]")

figure
fplot(f1ddot,[0 1],'-b')
hold on
fplot(f2ddot, [1 2],'-b')
fplot(zero,[2 3],'-b')
hold off
ylabel("Theta_ddot [rad/s^2]")
xlabel("Time [s]")

if (time>= 0 && time<=1)
    theta = subs(f1, t, time);
    theta_dot = subs(f1dot, t, time);
    theta_ddot = subs(f1ddot, t, time);
else if (time <= 2)
    theta = subs(f2, t, time);
    theta_dot = subs(f2dot, t, time);
    theta_ddot = subs(f2ddot, t, time);
else
    theta = 0;
    theta_dot = 0;
    theta_ddot = 0;
end

end