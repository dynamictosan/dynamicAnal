function Ct = constraints_dt(sys, t)

Ct = [
    zeros(2 * length(sys.joints.revolute) + 2*length(sys.joints.translation) + length(sys.joints.simple) , 1)
    constraints_dt_simple_driving(sys, t)
    ];