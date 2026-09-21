% Torque Free Motion script

clc;
clear;

% Initial Conditions
J = [0.096 0 0; 0 0.096 0; 0 0 0.166];
theta_b_0 = [0;0;180];
w_b_0 = [0.000; 0.000; 0.000];

% Gain Values for settling which matches observed data
Kp = .010;
Kd = .02;

% Gain Values from Platform
%Kp = 0.015;
%Kd = 0.015;
% These values are inaccurate compared to what we see in real life
% This is probably because the simulation does not take into account
% friction

% Schmitt Trigger Variables
dOn = 0.15;
dOff = 0.1;
thrust = 0.312 * 1 * 4;

tspan = 30;

out = sim('ControlledMotion.slx');

figure;
subplot(3, 1, 1)
plot(out.tout, out.theta(:,4))
grid on;
title("Theta vs Time")
xlabel("Time (s)")
ylabel("Theta (degrees)")

subplot(3, 1, 2)
plot(out.tout, out.w_b(:,4))
grid on;
title("Omega vs Time")
xlabel("Time (s)")
ylabel("Omega (degrees/s)")


subplot(3, 1, 3)
plot(out.tout, out.commanded_torque(:,2))
hold on
plot(out.tout, out.commanded_u(:,2))
grid on;
title("Desired and Commanded Torques vs Time")
legend("Commanded Torque", "Desired Torque")
xlabel("Time (s)")
ylabel("Nm")