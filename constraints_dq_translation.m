function Cq = constraints_dq_translation(sys, q)

Cq = zeros(2 * length(sys.joints.translation), length(q));
c_id = 0;

O = omega();
I = eye(2);
for j = sys.joints.translation
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    phi_1 = qi(3);
    q0_s2 = j.p_j;
    A_s2 = rot(phi_1);
    q_s2 = qi(1:2) + A_s2 * q0_s2;
    
    dF_dx_i = qi(2) -  q_s2(2);
    dF_dy_i = -(qi(1) - q_s2(1));
    dF_dfi_i = -(qj(1) - qi(1)) * (qi(1) - q_s2(1))...
               -(qj(2) - qi(2)) * (qi(2) - q_s2(2));
    dF_dx_j = -dF_dx_i;
    dF_dy_j = -dF_dy_i;
    dF_dfi_j = (qj(1) - qj(1)) * (qi(1) - q_s2(1))...
             + (qj(2) - qj(2)) * (qi(2) - q_s2(2));
    
    Cq(c_id + (1:2), j.body_i_qidx) = [dF_dx_i, dF_dy_i, dF_dfi_i
                                0, 0, 1];
    Cq(c_id + (1:2), j.body_j_qidx) = [dF_dx_j, dF_dy_j, dF_dfi_j;
                                0, 0, -1];
    c_id = c_id + 2;
    
    
end

end

