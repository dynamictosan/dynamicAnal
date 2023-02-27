function g = accel_eqn(sys, q, qd, t)
% Return LHS  of acceleration equation "g"

cons_no= (2 * length(sys.joints.revolute)) + (2*length(sys.joints.translation)) + ...
    length(sys.joints.simple) + length(sys.joints.simple_driving);

g = zeros(cons_no, 1);
c_idx = 0;

for rj = sys.joints.revolute
    q1 = q(rj.body_i_qidx);
    q2 = q(rj.body_j_qidx);
    qd1 = qd(rj.body_i_qidx);
    qd2 = qd(rj.body_j_qidx);
    qd1 = qd1(3);
    qd2 = qd2(3);
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    g(c_idx + (1:2)) = A1 * rj.s_i * qd1 .* qd1  -  A2 * rj.s_j * qd2 .* qd2;
    c_idx = c_idx + 2;
end

for pj = sys.joints.translation
    q1 = q(pj.body_i_qidx);
    q2 = q(pj.body_j_qidx);
    qd1 = qd(pj.body_i_qidx);
    qd2 = qd(pj.body_j_qidx);
    
    q0_s2 = pj.p_j;
    phi_1 = q1(3);
    A_s2 = rot(phi_1);
    q_s2 = q1(1:2) + A_s2 * q0_s2;
    
    g(c_idx + 1) = - 2 * ((q1(1) - q_s2(1)) * (qd1(1) - qd2(1))... 
                   + (q1(2) - q_s2(2)) * (qd1(2) - qd2(2))) * qd1(3)...
                   - ((q1(1) - q_s2(1)) * (q1(2) - q2(2))...
                   - (q1(2) - q_s2(2)) * (q1(1) - q2(1))) * qd2(3) * qd2(3);
    g(c_idx + 2) = 0;
    c_idx = c_idx + 2;
end

for sj = sys.joints.simple
    qb = qd(sj.body_qidx);
    g(c_idx + 1) = qb(sj.coord_id) - sj.coord_value;
    c_idx = c_idx + 1;
end

for dj = sys.joints.simple_driving
    g(c_idx + 1) =  - dj.coord_fun_dtt(t);
    c_idx = c_idx + 1;
end

