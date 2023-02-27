function sys = add_force_single(sys, body_name, f_t_1, p_i)
%ADD_JOINT_REVOLUTE Add revolute joint definition to the system
%      arguments
%          sys (1,1) struct
%          body_name (1,1) string
%          f_t_1 (1,1) function_handle
%          p_i (2,1) double = [0; 0]
%  
%      end

    O = omega();
%     
%     jo = body_name_to_qidx(sys, body_name);
% %     qrev = q(jo);
%     sys.bodies(4).orientation
    Aa = rot(0.523599);
%     p_i = [-0.2;0.3];
    spp = O * Aa * p_i;
%     ff=[1.2;0.5];

    go=spp'*f_t_1;

    chap = [f_t_1;go];
    sys=body_name_to_forceid(sys, body_name, chap);
%     body_name_to_forceid(sys, body_b, body_b_mom);
%     sys.endF(1:2) = f_t_1;
%     sys.endF(3) = go;


end
