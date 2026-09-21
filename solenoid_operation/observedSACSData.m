
% Taking observed SACS Platform data

clc;
clear;

[t1, data] = serial_reader_nV2(1000);

subplot(3,1,1)
plot(t1, data(:,1))
title("Theta vs Time")
xlabel("Time (s)")
ylabel("Theta (degrees)")
subplot(3,1,2)
plot(t1, data(:,2))
title("Omega vs Time")
xlabel("Time (s)")
ylabel("Omega (degrees/s)")
subplot(3,1,3)
plot(t1, data(:,4))
hold on;
plot(t1, data(:,3))
grid on;
title("Desired and Commanded Torques vs Time")
legend("Commanded Torque", "Desired Torque")
xlabel("Time (s)")
ylabel("Nm")