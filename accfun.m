function qdd = accfun(t,ty)
    q=ty(1:ty-12);
%     qd=ty(length(q0)+1:end);
    g = constraints(sys,q0,t);
    alpha = sys.balpha;
    beta = sys.bbeta;
    g = g - 2 * alpha * Cp - beta^2 * C;
    Cq = constraints_dq(sys, q);
    A = [M, Cq'
        Cq, zeros(nC)];
    b = [f;
        g];
    qdd = A \ b;
%         qdd = qdd_lambda(1:end - nC)
end