function qdd = acceleration(sys, q, qd, t)




C = constraints(sys, q, t);


M = mass_matrix(sys);
f = forces(sys);
if length(C) == 0
    qdd = M \ f;
    return
end

C = constraints(sys, q, t);
Cq = constraints_dq(sys, q);
Ct = constraints_dt(sys, t);
Cp = Cq * qd + Ct;
nC = length(C);
alpha = sys.balpha;
beta = sys.bbeta;
g = C - 2 * alpha * Cp - beta^2 * C;


LHS = [M, Cq'; Cq, zeros(nC)];
rhs = [f; g - 2 * alpha * Cp - beta^2 * C];
qddlambda = LHS \ rhs;
qdd = qddlambda(1:1:end - nC);