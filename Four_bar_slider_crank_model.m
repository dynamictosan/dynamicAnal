clc, clear all, close all
%% PREPROCESSOR
% Globally we have a complete multibody system
% It must contain bodies, joints, analysis settings
sys = make_system();

% Bodies

% What do we need to describe our body?
% location, orientation, name 

sys = add_body(sys, "ground");
sys = add_body(sys, "crank", [-0.1, 0.05], -deg2rad(30));
sys = add_body(sys, "intermediate", [-0.4, 0.05], deg2rad(45));
sys = add_body(sys, "link", [-0.9, 0.05], deg2rad(15));
sys = add_body(sys, "slider", [-1.2, 0]);

% Joints - kinematic (revolute and simple)
sys = add_joint_revolute(sys, "ground", "crank", [0; 0], [0.1; 0]);
sys = add_joint_revolute(sys, "crank", "intermediate", [-0.1; 0.05], [0.2; 0.05]);
sys = add_joint_revolute(sys, "intermediate", "link", [-0.2; 0.05], [0.3; 0.05]);
%---sys = add_joint_revolute(sys, "crank", "link", [-0.1; 0], [0.3; 0]);
sys = add_joint_revolute(sys, "link", "slider", [-0.3; 0]);

sys = add_joint_translation(sys, "slider", "ground", [0; 0], [0.05; 0], [0.0, 0.0]);




sys = add_joint_simple(sys, "ground", "x");
sys = add_joint_simple(sys, "ground", "y");
sys = add_joint_simple(sys, "ground", "fi");

sys = add_joint_simple_driving(sys, "crank", "fi", ...,
    @(t) -deg2rad(30) - 1.2 * t, ...
    @(t) - 1.2);

sys = set_solver_settings(sys, 10, 0.001);

%% SOLVER fsolve

%[Tf, Qf] = solve_kinematics_fsolve(sys);

%% SOLVER NR

[T, Q, Qd, Qdd] = solve_kinematics_NR(sys);

%% POSTPROCESSING
pidx = 4;
subplot(2,2,1)
plot(T, Qd(pidx, :), ...
    T(1:end-1), (Q(pidx, 2:end)-Q(pidx, 1:end-1))/0.001, '--')
% axis equal
subplot(2,2,2)
plot(Q(4,:), Q(5,:), Q(7,:), Q(8,:), ...
    Q(10,:), Q(11,:), 'LineWidth', 3)
axis equal
title('Kinematic analysis on position')

subplot(2,2,3)
plot(Qd(4,:), Qd(5,:), Qd(7,:), Qd(8,:), ...
    Qd(10,:), Qd(11,:), 'LineWidth', 3)
axis equal
title('Kinematic analysis on velocity')

subplot(2,2,4)
plot(Qdd(4,:), Qdd(5,:), Qdd(7,:), Qdd(8,:), ...
    Qdd(10,:), Qdd(11,:), 'LineWidth', 3)
axis equal
title('Kinematic analysis on acceleration')
