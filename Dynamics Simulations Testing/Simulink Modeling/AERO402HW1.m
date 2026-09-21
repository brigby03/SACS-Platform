% AERO 402 HW 1

%% Question 3

finert = linspace(0, 1, 200);

c = 60 * 9.81;
delV = c .* log(1./finert);
figure;
semilogy(finert, delV)
hold on;

c = 320 * 9.81;
delV = c .* log(1./finert);
semilogy(finert, delV)


c = 950 * 9.81;
delV = c .* log(1./finert);
semilogy(finert, delV)


c = 3000 * 9.81;
delV = c .* log(1./finert);
semilogy(finert, delV)
title("Finert vs DeltaV")
xlabel("Finert")
ylabel("Delta V (m/s)")

%% Question 5

figure;
c = linspace(50,10000,200);
delV = c .* log(10);
semilogy(c, delV)
title("DeltaV vs Effective Thrust")
xlabel("Effective Thrust (m/s)")
ylabel("Delta V (m/s)")

% As shown by the graphs produced in both question 3 and question 5, we can
% see the way that deltaV changes as the variables within its equation
% change. For example, as finert increases from zero towards one, the
% deltaV is shown to decrease by the graphs in question 3. This makes sense
% because 