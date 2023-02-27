clc; clear all; close all;

% System parameters
m = 1;      % mass of point mass (kg)
g = 9.81;   % acceleration due to gravity (m/s^2)
L = 5;      % length of pendulum (m)
theta0 = pi/4; % initial angle (rad)

% Define system
sys = make_system();
sys = set_solver_settings(sys, 10, 0.01);


% Add bodies
sys = add_body(sys, "pivot");
sys = add_body(sys, "pendulum", [L*sin(theta0); -L*cos(theta0)], 10, 1e3);

% Add joints
sys = add_joint_simple(sys, "pivot", "x");
sys = add_joint_simple(sys, "pivot", "y");
sys = add_joint_simple(sys, "pivot", "fi");

sys = add_joint_revolute(sys, "pivot", "pendulum", [0;0], [L;0]);

% % Add forces
% sys = add_force_gravity(sys, "pendulum", m, [0;-g]);
% sys = add_force_damping(sys, "pendulum", 0.1); % add damping force


tf=2.0;


% q0 = initial_coordinates(sys);
% qd0 = zeros(size(q0));
% accfun = @(u, v, t)acceleration(sys, u, v, t);
% h = 0.001;
% [t2, Q, Qp] = EulerCromer(accfun, tf, q0, qd0, h);
% % 
% plot(t2, Q(:, 5), '--')
% % legend('reference', 'theta of the pendulum')
% 



% Solve dynamics
[T, Q] = solve_dynamics_ode45(sys);

% % Calculate velocity
% V = calculate_velocity(sys, Q);

% Plot results
figure;
subplot(2,1,1);
plot(T, Q(:,6), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity)');
title('Simple Pendulum');

subplot(2,1,2);
plot(T, Q(:,6+6), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration)');
