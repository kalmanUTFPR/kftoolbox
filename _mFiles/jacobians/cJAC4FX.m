% Author: Renata da Costa Ribeiro
% Date: 18-August-2020

% Class for the jacobian calculation for the discrete-time model with 4 states:
% Jacobian method to calculate the linearized matrix Fx at the time step t

%x1 = x-position in a global coordinate system
%x2 = y-position in a global coordinate system
%x3 = yaw angle
%x4 = velocity in x-direction

%u1 = steering angle of front wheels
%u2 = ax longitudinal acceleration

classdef cJAC4FX < handle
  properties(Access = private)
    jacFunc    = [];                 % right-hand side of differential equations
    p          = [];                 % vehicle parameter structure
    dt         = [];                 % time step duration 
    lwb        = [];                 % length wheel-base  
  end
  
  methods(Access = public)
    function obj  = cJAC4FX(arg_p, arg_dt)
      obj.dt = arg_dt;
      obj.p = arg_p;
      obj.lwb = obj.p.a + obj.p.b;  
      obj.jacFunc = @(x,u)[ 1 0   -obj.dt * x(4) * sin(x(3))   obj.dt * cos(x(3));
                            0 1    obj.dt * x(4) * cos(x(3))   obj.dt * sin(x(3)); 
                            0 0                           1    obj.dt * tan(u(1)) / obj.lwb; 
                            0 0                           0                     1; ];                                 
    end
    
    function currentFx = solver(obj,arg_x, arg_u)   
        
      % Calculate the Jacobian
      currentFx = obj.jacFunc(arg_x, arg_u);
    end
  end
end