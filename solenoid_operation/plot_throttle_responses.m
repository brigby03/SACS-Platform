% Plot Thrust Responses for various duty cycles

clc;
clear;
close all;

duty100 = load("60psiStepResponse_FullyOpen.mat");
duty90 = load("60psi_5hz_90per.mat");
duty83 = load("60psi_5hz_83per.mat");
duty75 = load("60psi_5hz_75per.mat");
duty50 = load("60psi_5hz_50per.mat");
duty25 = load("60psi_5hz_25per.mat");
duty10 = load("60psi_5hz_10per.mat");
temp = load("eightyFive_psi_fullyOpen.mat");

disp(duty10)


duty100.sixty_psi = duty100.sixty_psi(13:end);
duty100.t1 = duty100.t1(13:end);
duty90.sixty_psi_5Hz_90per = duty90.sixty_psi_5Hz_90per(9:end);
duty90.t1 = duty90.t1(9:end);
duty83.sixty_psi_5Hz_83per = duty83.sixty_psi_5Hz_83per(12:end);
duty83.t1 = duty83.t1(12:end) - 0.2;
duty75.sixty_psi_5Hz_75per = duty75.sixty_psi_5Hz_75per(9:end);
duty75.t1 = duty75.t1(9:end);
duty50.sixty_psi_5Hz_50per = duty50.sixty_psi_5Hz_50per(8:end);
duty50.t1 = duty50.t1(8:end);
duty25.sixty_psi_5Hz_25per = duty25.sixty_psi_5Hz_25per(9:end);
duty25.t1 = duty25.t1(9:end);
duty10.hundred_psi_10Hz_10per = duty10.hundred_psi_10Hz_10per(33:end);
duty10.t1 = duty10.t1(33:end) - 2.04;

figure;
plot(duty100.t1, duty100.sixty_psi)
hold on
grid on
plot(duty90.t1, duty90.sixty_psi_5Hz_90per)
plot(duty83.t1, duty83.sixty_psi_5Hz_83per)
plot(duty75.t1, duty75.sixty_psi_5Hz_75per)
plot(duty50.t1, duty50.sixty_psi_5Hz_50per)
plot(duty25.t1, duty25.sixty_psi_5Hz_25per)
plot(duty10.t1, duty10.hundred_psi_10Hz_10per)
xlabel("Time (s)")
ylabel("Thrust (gf)")
title("60 Psi Step Response")
legend("100% Duty Cycle", "90% Duty Cycle", "83% Duty Cycle", "75% Duty Cycle", "50% Duty Cycle", "25% Duty Cycle", "10% Duty Cycle")

% plot(temp.t1, temp.eightyFive_psi_fullyOpen)