% Author: Renata da Costa Ribeiro
% Class for the Kinematic Single Track Model for discrete-time with 4 states:
% Euler's discretization method

%x1 = x-position in a global coordinate system
%x2 = y-position in a global coordinate system
%x3 = yaw angle
%x4 = velocity

%u1 = steering angle of front wheels
%u2 =  longitudinal acceleration

classdef cKST4disc < handle
  properties(Access = private)
    stMdl = [];                 % right-hand side of differential equations
    p     = [];                 % vehicle parameter structure
    lwb   = [];                 % length wheel-base  
    dt    = []; 
    angleSt = 3
    wrapSt  = 3;
  end
  
  methods(Access = public)
    function obj = cKST4disc(arg_p, arg_dt)
      obj.dt     = arg_dt;  
      obj.p      = arg_p;
      obj.lwb    = obj.p.a + obj.p.b;  
      obj.stMdl  = @(x,u)[  x(1) + obj.dt * x(4) * cos(x(3)); ...
                            x(2) + obj.dt * x(4) * sin(x(3));...
                            x(3) + obj.dt * x(4) * tan(u(1)) / obj.lwb;...
                            x(4) + obj.dt * u(2);];                                 
    end
    
    function x_kp1 = solver(obj,arg_x, arg_u)        
      % steering restrictions based on parameters
      % acceleration restrictions based on parameters
      % Solve the discretized model
      x_kp1 = obj.stMdl(arg_x, arg_u);
    end
    function z = alignangles(obj,arg_x,arg_z)        
      %takes the states that corresponds to an angle
      diff = arg_x(obj.angleSt)-arg_z(obj.angleSt);
      int = length(obj.angleSt);
      z = arg_z;
      for i = 1:int
        if (abs(diff(i))>pi)
          if ~isequal(sign(arg_x(obj.angleSt(i))), sign(arg_z(obj.angleSt(i))))
            if(arg_x(obj.angleSt(i))>0)
              z(obj.angleSt(i)) = arg_z(obj.angleSt(i)) + 2*pi;
            else
              z(obj.angleSt(i)) = arg_z(obj.angleSt(i)) - 2*pi;
            end
          end
        end
      end  
    end
    
    function xwrap = wrapOut(obj,arg_x)
        xwrap = arg_x;
        xwrap(obj.wrapSt,:) = wrapToPi(arg_x(obj.wrapSt,:)); 
    end
  end
end