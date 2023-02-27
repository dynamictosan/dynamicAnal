function C = constraints_translation(sys, q)
%CONSTRAINTS_TRANSLATION Compute constraints for the translational joints
C = zeros(2 * length(sys.joints.translation), 1);
c_id = 0;

for j = sys.joints.translation
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    Ai = qi(3);
    Aj = qj(3);
    
    q0_s2 = j.p_j;
    A_s2 = rot(Ai);
    q_s3 = qi(1:2) + A_s2 * q0_s2;
    
    n = [-(qi(2) - q_s3(2)); 
        qi(1) - q_s3(1)];
    d = [qj(1) - qi(1); 
        qj(2) - qi(2)];
    
    C(c_id + 1) = n' * d;
    C(c_id + 2) = Ai - Aj;
    c_id = c_id + 2;
    
end

end


