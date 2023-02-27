function [T, Q, Qd] = solve_dynamics_EC(sys)
%SOLVE_DYNAMICS_EC_NO_CONSTRAINTS Solve the multibody system sys on
%dynamics using Euler-Cromer integration scheme and assuming that there can
%be only simple constraints!!!

q0 = initial_coordinates(sys);
qd0 = zeros(size(q0));

C = constraints(sys, q0, 0.0);
Cq = constraints_dq(sys, q0);
Ct = constraints_dt(sys, q0);
nC = length(C);

if norm(C) > 1e-12
    warning("Initial constraint norm is too large - expect incorrect results!")
end

M = mass_matrix(sys);
f = forces(sys);

% Here put proper code for g vector. But for simple constraints this is
% accurate
Cp = Cq * qd0 + Ct;


    function qdd = accfun(q, qd, t)
        g = constraints(sys,q0,t);
        alpha = sys.balpha;
        beta = sys.bbeta;
        g = g - 2 * alpha * Cp - beta^2 * C;
        Cq = constraints_dq(sys, q);
        A = [M, Cq'
            Cq, zeros(nC)];
        b = [f;
            g];
        qdd_lambda = A \ b;
        qdd = qdd_lambda(1:end - nC);
    end

[T, Q, Qd] = odeEulerCromer(@accfun, q0, qd0, ...
    sys.solver.t_step, sys.solver.t_final);
end