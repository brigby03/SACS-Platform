function dxdt = ode_SACS(t, x, Iz, dampingFactor, K_lqr, xStar)
    u = (x(1)-xStar(1))*K_lqr(1,2);

    A = [0,       1; 
         0, -dampingFactor/Iz];

    B = [    0; 
           -1/Iz;];

    control_moment_arm_length = 1; % in m
    control_thrust = 5; % in N
    control_moment = 2*control_thrust*control_moment_arm_length;


    if u > control_moment
        u = control_moment;
    elseif u < -control_moment
        u = -control_moment;
    end

    dxdt = A * x + B * u;
end
