h = 0.01; % Delay considered as control parameter
lags = h;
tspan = [0 100];        % Simulation time
x0 = [pi+pi/64; 0; 0];  % Initial conditions [qp, qpp, qpd]

% History function (constant for t < 0)
history = @(t) x0;

% Solve DDE
options = ddeset('RelTol', 1e-6, 'AbsTol', 1e-8);
sol = dde23(@InertiaWheelPendulum, lags, history, tspan);

% Plot results
figure;
plot(sol.x, sol.y(1, :), LineWidth=1.5); 
xlabel('Time $[s]$', Interpreter='latex'); 
ylabel('Pendulum Angle $[rad]$', Interpreter='latex');
grid on;
box on;
title('Inertia Wheel Pendulum Motion with Delay-Based Controller', Interpreter='latex');
