% PolySim - Load Cell Data Reading

clc;
clear;
close all;

[t1, hundred_psi_fullyOpen] = serial_reader_nV2(300);

figure;
plot(t1, hundred_psi_fullyOpen);
xlabel("Time (s)")
ylabel("Thrust (g)")


