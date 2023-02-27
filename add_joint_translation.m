function sys = add_joint_translation(sys, body_i, ...
    body_j, p_i, p_j, q_i)
%ADD_JOINT_REVOLUTE Add revolute joint definition to the system
    arguments
        sys (1,1) struct
        body_i (1,1) string
        body_j (1,1) string
        p_i (2,1) double = [0; 0]
        p_j (2,1) double = [0; 0]
        q_i (2,1) double = [0; 0]
    end
    % Manual checking of bodies names
    check_body_exists(sys, body_i)
    check_body_exists(sys, body_j)
    
    joint = struct();
    joint.body_i_qidx = body_name_to_qidx(sys, body_i);
    joint.body_j_qidx = body_name_to_qidx(sys, body_j);
    joint.p_i = p_i;
    joint.p_j = p_j;
    joint.q_i = q_i;

    sys.joints.translation = [sys.joints.translation, joint];
end
