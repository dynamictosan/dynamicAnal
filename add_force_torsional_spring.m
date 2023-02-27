function sys = add_force_torsional_spring(sys, body_a, ...
    body_b, k_n, c_n, vall)
%ADD_JOINT_REVOLUTE Add revolute joint definition to the system
    arguments
        sys (1,1) struct
        body_a (1,1) string
        body_b (1,1) string
        k_n (1,1) double = 0
        c_n (1,1) double = 0 
        vall (1,1) double = 0
    end

    deformed_angle = 50;

    rotSpring = struct("name", body_a);
    rotSpring.body_a = body_a;
    rotSpring.body_b = body_b;

    if deformed_angle > vall
        mom = k_n*(deformed_angle - vall);
        %body i in the positive rotational direction and on body j in the negative rotational direction
        %NB: Body j comes before body i. E.g ground is j while link1 is i.
        body_a_mom =[0; 0; -mom];
        body_b_mom = [0; 0;  mom];        

        sys=body_name_to_forceid(sys, body_a, body_a_mom);
        sys=body_name_to_forceid(sys, body_b, body_b_mom);
    else
        body_a_mom = [0; 0; mom]; 
        body_b_mom = [0; 0; -mom];

        sys=body_name_to_forceid(sys, body_a, body_a_mom);
        sys=body_name_to_forceid(sys, body_b, body_b_mom);
    end

    %sys.rotational_spring = [sys.rotational_spring, rotSpring];
end
