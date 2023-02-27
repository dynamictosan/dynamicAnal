function sys = make_system(gravity)
%MAKE_SYSTEM Create a data structure to store complete multibody system
arguments
    gravity (2,1) double = [0; -9.80665]
end
sys = struct();

sys.bodies = struct([]);

sys.joints = struct('revolute', struct([]), ...
    'simple', struct([]), ...
    'simple_driving', struct([]), ...
    'translation', struct([]));

sys.rotational_spring = struct([]);

sys.balpha=5;

sys.bbeta=5;

sys.gravity = gravity;

sys.solver = struct('t_final', 1, 't_step', 0.01);

end

