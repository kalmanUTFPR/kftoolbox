
classdef cKSTMeasMdl < handle
  properties(Access = private)
    msMdl = [];                 % right-hand side of differential equations
  end
  
  methods(Access = public)
    function obj = cKSTMeasMdl()
      obj.msMdl  = @(x) [1 0 0 0;  0 1 0 0; 0 0 0 1] * x;                                 
    end
    
    function x_kp1 = solver(obj,arg_x)
        
      x_kp1 = obj.msMdl(arg_x);
    end
  end
end