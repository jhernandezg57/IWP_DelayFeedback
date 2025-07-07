function dx = InertiaWheelPendulum(t, x, Z)
    % System parameters
    Jd = 0.00035871; % kg/m^2  Disc
    Jp = 0.00035756; % kg/m^2  Pendulum
    lcp = 2.25 * 0.0254; l1 = 2 * lcp; % m
    mp = 0.14379649; % kg 
    md = 0.27799999; % kg
    g = 9.807; % m/s^2

    % Delay (h = 0.01)
    h = 0.01;
    x_delayed = Z(:,1); % Delayed state [qp_h; qpp_h; qdp_h]
    
    % Compute inertia terms
    D11 = (mp * lcp^2 + md * l1^2 + Jp + Jd);
    D22 = Jp; 
    D21 = D22; 
    D12 = D21;
    Dd = D11 * D22 - D12 * D21;
    
    D11b = D11 / Dd; 
    D12b = D12 / Dd; 
    D21b = D21 / Dd; 
    D22b = D22 / Dd;
    
    mb = mp * lcp + md * l1;

    % State variables
    qp  = x(1);  % Pendulum angle
    qpp = x(2);  % Pendulum angular velocity
    qdp = x(3);  % Wheel speed
    
    qp_h  = x_delayed(1); % Delayed pendulum angle
    qpp_h = x_delayed(2); % Delayed pendulum velocity
    qdp_h = x_delayed(3); % Delayed wheel speed

    % Controller gains
    K = [-0.438444, -0.0297807, -0.0000228586];

    % Energy-based swing-up control
    E = (1/2) * Dd / D22 * (qpp_h)^2 + mb * g * (1 + cos(qp_h));
    Eo = 2 * mb * g;
    tau = 0;
    
    tmin = (12e-3);
    d = tmin / Eo;     
    satqpp = qpp_h; 
    sigma = 0.08; 
    first_use = false; 
    qdpd = 0;

    if qp_h <= 0.1818
        qph = 0;
    else
        qph = 2 * pi;
    end 

    if ((qp_h - qph)^2 + qpp_h^2) < sigma
        if first_use == false 
            qdpd = qdp_h; % reference wheel speed
            first_use = true; % reference set
        end
        z = [qp_h - qph; qpp_h; qdp_h - qdpd];
        tau = -K * z;
    else
        if qpp_h > d
            satqpp = d;
        elseif qpp_h < -d
            satqpp = -d;
        elseif abs(qpp_h) <= d
            satqpp = qpp_h;
        end
        tau = (E - Eo) * satqpp;    
    end    

    % System dynamics (LModel)
    qppp = D22b * mb * g * sin(qp) - D21b * tau;
    qdpp = -D12b * mb * g * sin(qp) + D11b * tau;

    % State derivatives
    dx = zeros(3,1);
    dx(1) = qpp;   % dq/dt = qpp (pendulum angular velocity)
    dx(2) = qppp;  % d(qpp)/dt = qppp (pendulum angular acceleration)
    dx(3) = qdpp;  % d(qdp)/dt = qdpp (wheel acceleration)
end
