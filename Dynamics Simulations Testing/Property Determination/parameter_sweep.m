clc; clear; close all

%% constants
m_wheel = 0.35;                   % kg - mass can be adjusted w/ bolts + print settings 
m_length = 0.048;                 % kg - mass of only the memember L
m_motor = 0.11;                   % kg
m_pend = m_motor + m_length;      % kg - mass of memeber L + motor on top
wt = 0.01;                        % m - wheel thickness
radius_motor = 0.01;              % m
g = 9.81;                         % m/s^2

% motor parameters from data sheet & motor label
rated_voltage = 12;                          % V
rated_speed = 120 * 2*pi / 60;               % rad/s
rated_torque = 0.85 * 0.09806;               % N.m
stall_torque = 4.2 * 0.09806;                % N.m
I_stall = 1;                                 % A

% calculate motor constants
R = rated_voltage/ I_stall;                  % ohms - motor resistance
Kt = stall_torque / rated_voltage;           % torque constant
Kv = rated_speed / rated_voltage;            % RPM/V constant
 
% parameter sweep ranges~ change as needed (limited by 3D printer bed side)
% Note: change n<<20 if run time is long
wheel_radius_range = linspace(0.05, 0.11, 25); 
pendulum_length_range = linspace(0.21, 0.125, 25);

% create results matrix
results = zeros(length(wheel_radius_range), length(pendulum_length_range));

%% parameter sweep
for r_idx = 1:length(wheel_radius_range)
    for L_idx = 1:length(pendulum_length_range)
        r_wheel = wheel_radius_range(r_idx);
        L = pendulum_length_range(L_idx);
        
        % mass moment of inertia for system
        Im = 2/5*m_motor*radius_motor^2 + m_motor*(L+radius_motor)^2; % motor
        Is = 1/3*m_length*L^2 + Im;                                   % pendulum
        Ir = 1/2*m_wheel*(r_wheel^2 + (r_wheel- wt)^2);               % wheel
       
        % matrices
        A = [     0,            1,           0; 
            m_pend*g*L/Is,      0,       Kt^2/(R*Is); 
                  0,            0,      -Kt^2/(R*Ir)];
        B = [0; -Kt/(R*Is); Kt/(R*Ir)];
        C = [1, 0, 0];
        D = 0;

        % system
        sys = ss(A, B, C, D);

        % lqr controller
        Q = eye(3);
        R_control = 0.25;
        K_lqr = lqr(sys,Q,R_control);

        % optimization
        max_voltage = 12;
        max_angle = 90;
        
        %
        for init_angle_deg = 1:90
            x0 = [deg2rad(init_angle_deg); 0; 0];
            tspan = [0, 10];
            [t, x] = ode45(@(t, x) ode_inverted_pendulum(t, x, K_lqr, m_pend, g, L, Is, Kt, R, Ir), tspan, x0);
            rated_voltage = max(K_lqr * x');

            if rated_voltage < max_voltage
                max_angle = init_angle_deg;
            else
                break;
            end
        end

        results(r_idx, L_idx) = max_angle;
        fprintf('Wheel radius: %.4f m, Pendulum length: %.4f m, Max angle: %d degrees\n', r_wheel, L, max_angle);
    end
end

disp(results);

%% optimal wheel radius and pendulum length combination
[max_angle, max_idx] = max(results(:));
[r_idx, L_idx] = ind2sub(size(results), max_idx);
optimal_r_wheel = wheel_radius_range(r_idx);
optimal_L = pendulum_length_range(L_idx);

fprintf('Optimal wheel radius: %.4f m\nOptimal pendulum length: %.4f m\nMax Angle: %.1f deg\n', optimal_r_wheel, optimal_L, max_angle);


% meshgrid for the wheel radius and pendulum length ranges
[R_mesh, L_mesh] = meshgrid(wheel_radius_range, pendulum_length_range);

% plot the results as a surface
figure;
surf(R_mesh, L_mesh, results);
xlabel('Wheel Radius (m)');
ylabel('Pendulum Length (m)');
zlabel('Max Angle (degrees)');
title('Max Angle vs. Wheel Radius and Pendulum Length');
colorbar;
