clc; clear; close all;

Iz = .166; % in kg m^2
dampingFactor = .2;

A = [0,       1; 
     0, -dampingFactor/Iz];
B = [    0; 
        1/Iz;];

C = [1, 0];
D = 0;

sys = ss(A, B, C, D);

control_moment_arm_length = 0.312; % in m
control_thrust = 3.125; % in N
control_moment = 2*control_thrust*control_moment_arm_length;
R_control = 1/control_moment^2;
    
    
    
        % lqr controller
        Q = [1/(180^2) 0;% Q should be a matrix 1/(maximum_allowable_error)^2
               0     1/(10^2)]; % Therefore, theta has maximum error of 180 degrees and theta_dot is given maximum error of 10 deg/s
     
        K_lqr = lqr(sys,Q,R_control);
        
        x0 = [0; 0];
        xStar = [180; 0];
        tspan = [0, 20];
        [t, x] = ode45(@(t, x) ode_SACS(t, x, Iz, dampingFactor, K_lqr, xStar), tspan, x0);

figure;
plot(t, x(:,1))
hold on
plot(t,x(:,2))
title("Critically Damped Response")
xlabel("Time (s)")
ylabel("Position (degs)")