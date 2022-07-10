% Class for the Resistor Indutor Capacitor Circuit (Linear)

% Estados:
    %x1 = vc1
    %x2 = il
    %x3 = vc2
% Entradas: 
    %u1 = vi
% Saídas: 
    %y1 = x3
% EDOs: 
    %x1_dot = -1/(R*Ca)*x1 + 1/Ca*x2 -1/(R*Ca)x3 + 1/(R*Ca)*u1
    %x2_dot = -1/L*x1 + u1/L
    %x3_dot = -1/(R*Ca)*x1 -1/(R*Ca)*x3 + 1/(R*Ca)*u1

% Valor dos parâmetros do sistema:
%R  = 1e3;  Resistência [Ohms]
%Ca = 1e-6; Capacitância [Faraday]
%L  = 1;    Impedância [Henry]

% Esse sistema continuo foi discretizado para ser utilizado no filtro


classdef cRLCStMdl < handle
  properties(Access = private)
    A      = []; % Matrix A
    B      = []; % Matrix B
    p      = []; % Parameter structure
  end
  
  methods(Access = public)
    function obj = cRLCStMdl()
      obj.A = [     0.992527984900962    7.47194504961741	-0.00744396028009615;
                -7.47194504961741e-06	0.999971945181059	2.79847695212635e-08;
                 -0.00744396028009615 -0.0279847695212635	   0.992555969670484]; 

      obj.B = [0.00747201509903757;
               7.47194504961741e-06;
               0.00744396028009615];
    end
    
    function xkp1 = solSt(obj,arg_x, arg_u)        
      xkp1 = obj.A*arg_x + obj.B*arg_u; 
    end

    function Pkp1 = solCov(obj,arg_P, arg_Q)        
      Pkp1 = obj.A* arg_P* obj.A' + arg_Q; 
    end
  end
end





