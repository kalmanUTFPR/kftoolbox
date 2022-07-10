classdef cEKF < cFilter
  properties 
    hMdl   = [];  % dynamic model - f(x,u)
    xjac   = [];  % jacobian matrix - df/fx
    hmtrx  = [];  % measurement matrix (matrix C)
    P      = [];  % covariance matrix 
    x      = [];  % state vector
    Q      = [];  % process noise
    R      = [];  % measurement noise
    yk     = [];  % Innovation
    dk     = [];  % Residual
    Sk     = [];  % Innovation covariance
  end
  methods 
    function obj = cEKF(hMdl, xjac, hmtrx, Q, R)
      %  ExtendedKalmanFilter Constructor
      %  Construct an EKF given the state equation, measurement matrix,
      %  state Jacobian, and the proccess and sensors noises.
      obj.hMdl = hMdl;
      obj.xjac = xjac;
      obj.hmtrx = hmtrx;
      obj.Q = Q;
      obj.R = R;
    end		
    function initialize(obj,x0,u0,P0)
      obj.x = x0;
      obj.u = u0;  
      obj.P = P0; 
    end
    
    function prediction(obj, u) 
      % Prediction Step
      
      Fkx  = obj.xjac.solver(obj.x, u);               
      xkp1 = obj.hMdl.solver(obj.x, u);            
      Pkp1 = Fkx*obj.P*Fkx' + obj.Q;
      obj.P = Pkp1;
      obj.x = xkp1;
      % Update inputs
      obj.u = u;
    end
    
    function measurementUpdate(obj, z)
      % Correction Step
      xkp1 = obj.x;
      Pkp1 = obj.P;  
      Hk = obj.hmtrx;
      z = obj.hMdl.alignangles(xkp1,z);

      obj.yk = z - (Hk * xkp1);
      obj.Sk = (Hk * Pkp1 * Hk') + obj.R;           
      Kk = (Pkp1 * Hk') / obj.Sk;
      xkp1 = xkp1 + (Kk * obj.yk);
      xkp1 = obj.hMdl.wrapOut(xkp1);
      Pkp1 = (eye(length(obj.x)) - Kk * Hk) * Pkp1;            
      obj.dk = z - (Hk * xkp1);    
      obj.x = xkp1;
      obj.P = Pkp1;
    end
         
    function stateEstimate = getStateEstimate(obj)
      stateEstimate.mean = obj.x;
      stateEstimate.covariance = obj.P;
      stateEstimate.residual = obj.dk;
      stateEstimate.innovation = obj.yk;
      stateEstimate.covinnovation = obj.Sk;
      stateEstimate.r = obj.R;      
    end
  end
end




