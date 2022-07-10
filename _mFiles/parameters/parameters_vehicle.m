% Parameters  - Tugger: Parameter estimation
function p = parameters_tug()
p.m = 4500; %mass
p.a = 0.79400000000000; %length front
p.b = 0.79400000000000; %length rear
p.I_z = [6152.69699338564]; %Inertia around axis z
p.h_s = [0.555961116496605]; % distance in Z of the c.g.
p.tire.p_dy1 = [1.46115553428051];%mu
p.tire.p_ky1 = - [1.07907726422807] *[1.46115553428051];%stifness