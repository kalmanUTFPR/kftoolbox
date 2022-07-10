% Class for the Resistor Indutor Capacitor Circuit (Linear)

classdef cSMMMeasMdl < handle
  properties(Access = private)
    MsMdl = []; % Measurement Model   
    C      = []; % Matrix C
    D      = []; % Matrix D

  end
  
  methods(Access = public)
    function obj = cSMMMeasMdl()
      obj.C = [ 1 0 0 0 0 0;
                0 0 1 0 0 0;
                0 0 0 0 1 0]; 
      obj.D = 0;
    end

    function Cmtrx = GetMtrxC(obj)
      Cmtrx = obj.C;
    end

    function Dmtrx = GetMtrxD(obj)        
      Dmtrx = obj.D;
    end
  end
end





