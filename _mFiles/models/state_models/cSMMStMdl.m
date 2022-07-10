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


classdef cSMMStMdl < handle
  properties(Access = private)
    A      = []; % Matrix A
    B      = []; % Matrix B
    p      = []; % Parameter structure
  end
  
  methods(Access = public)
    function obj = cSMMStMdl()
      obj.A = [    0.999925125623905	0.00997483369859280	    -1.24529694925830e-07	2.48747933857088e-05	1.24529694925830e-07	3.11248449216378e-10;
                  -0.0149622505478892	   0.994950146171301	-3.73117232058894e-05	 0.00496241752621576	3.73117232058894e-05	1.24374070701222e-07;
                  -1.24530629297504e-07	2.48747933857088e-05	    0.999925251089214	 0.00994995890707583	7.47489107864239e-05	2.49061258595008e-07;
                  -3.73121900785632e-05	 0.00496241752621576	  -0.0149245647687259	   0.989987729578831	  0.0149245647687259	7.46243801571263e-05;
                  -9.34371673754423e-13	3.11248449216377e-10	 7.48732857914967e-05	2.49061258595008e-07	   0.999925126714209	 0.00997479224243797;
                  -4.66872673824566e-10	1.24374070701222e-07	   0.0149618147717691	7.46243801571263e-05	 -0.0149618147717691	   0.994937730592990]; 

      obj.B = [2.07551048829174e-08;
               6.21869834642719e-06;
               1.24583075784695e-05;
               0.00248748972676896;
               1.55780732236258e-10;
               6.22653146487521e-08];
    end
    
    function xkp1 = solSt(obj,arg_x, arg_u)        
      xkp1 = obj.A*arg_x + obj.B*arg_u; 
    end

    function Pkp1 = solCov(obj,arg_P, arg_Q)        
      Pkp1 = obj.A* arg_P* obj.A' + arg_Q; 
    end
  end
end





