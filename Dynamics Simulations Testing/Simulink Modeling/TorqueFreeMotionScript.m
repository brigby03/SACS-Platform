% Torque Free Motion script

clc;
clear;

J = [0.016 0 0; 0 0.016 0; 0 0 0.025];
T = [0;0;0.31*2];

tspan = 100;

w_b_0 = [0.000; 0.000; 0.000];

out = sim('TorqueFreeMotion.slx');

spanw = out.w_b_ECI(:,1);
wx = out.w_b_ECI(:,2);
wy = out.w_b_ECI(:,3);
wz = out.w_b_ECI(:,4);

figure;
plot(wz)
title("Theta-Z vs Time")
ylabel("Theta-Z [deg]")
xlabel("Time [seconds]")

