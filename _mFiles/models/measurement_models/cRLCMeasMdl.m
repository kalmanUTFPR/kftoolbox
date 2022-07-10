% Class for the Resistor Indutor Capacitor Circuit (Linear)

classdef cRLCMeasMdl < handle
  properties(Access = private)
    MsMdl = []; % Measurement Model   
    C      = []; % Matrix C
    D      = []; % Matrix D

  end
  
  methods(Access = public)
    function obj = cRLCMeasMdl()
      obj.C = [ 0 0 1 ]; 
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





