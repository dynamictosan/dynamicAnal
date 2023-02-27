function sys = body_name_to_forceid(sys, body_name, added)

    arguments
        sys (1,1) struct
        body_name (1,1) string
        added (3,1) = [0; 0; 0]
    end


idd=body_name_to_id(sys, body_name);
sys.bodies(idd).forces  = sys.bodies(idd).forces + added;





%get the forces from body using bid
%then use bid to get the cure loc + calculated force and assign to current


%then to apply, use force with bid number

