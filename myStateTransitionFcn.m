function x_next = myStateTransitionFcn(x, u)
    % x: 16x1 [4x Vx; 4x Vy; 4x Omega; 4x Yaw Rate]
    % u: 12x1 [4x Angle; 4x Torque; 4x Brake]
    
    % --- Constants ---
    m = 1181; Iz = 2066; R = 0.298; B = 50000; 
    dt = 0.001; lf = 1.1; lr = 1.4; Iw = 0.5; kB = 0.1;

    % 1. Initialize x_next as a copy of x to maintain 16x1 dimension
    x_next = x; 

    % 2. Extract "Master" States 
    % We take the first index of each redundant group for the physics math
    vx = x(1);   % First of the 4 Vx states
    vy = x(5);   % First of the 4 Vy states (indices 5-8)
    w  = x(9:12);% The 4 wheel speeds (indices 9-12)
    r  = x(13);  % First of the 4 Yaw Rate states (indices 13-16)

    % 3. Singularity Protection
    vx_safe = max(abs(vx), 0.1) * sign(vx);

    % 4. Physics (Bicycle Model)
    delta_f = mean(u(1:2));
    alpha_f = delta_f - (vy + lf * r) / vx_safe;
    alpha_r = 0 - (vy - lr * r) / vx_safe;
    
    Fyf = 2 * B * alpha_f; 
    Fyr = 2 * B * alpha_r; 
    Fx_total = sum(u(5:8)) / R;

    % Derivatives
    dvx = (Fx_total - Fyf * sin(delta_f)) / m + (vy * r);
    dvy = (Fyf * cos(delta_f) + Fyr) / m - (vx * r);
    dr  = (lf * Fyf * cos(delta_f) - lr * Fyr) / Iz;
    dw  = (u(5:8) - (u(9:12) * kB .* sign(w))) / Iw;

    % 5. Update 16x1 State Vector
    % We update ALL 4 slots for each redundant state to keep the filter stable
    x_next(1:4)   = vx + dvx * dt;    % All 4 Longitudinal velocities
    x_next(5:8)   = vy + dvy * dt;    % All 4 Lateral velocities
    x_next(9:12)  = w  + dw  * dt;    % 4 unique Wheel speeds
    x_next(13:16) = r  + dr  * dt;    % All 4 Yaw rates
end