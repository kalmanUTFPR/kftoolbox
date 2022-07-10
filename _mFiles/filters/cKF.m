classdef cKF < cFilter
  properties 
    StMdl      = [];  % Ax + Bu - state model 
    MsMdl      = [];  % Cx + D - measurement model
    P          = [];  % covariance matrix 
    x          = [];  % state vector
    Q          = [];  % process noise
    R          = [];  % measurement noise
    yk         = [];  % Innovation
    dk         = [];  % Residual
    Sk         = [];  % Innovation covariance 
  end
  
  methods 
    function obj = cKF(StMdl, MsMdl, Q, R)
      %  KalmanFilter Constructor
      obj.StMdl = StMdl;
      obj.MsMdl = MsMdl;
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
      xkp1 = obj.StMdl.solSt(obj.x, u);            
      Pkp1 = obj.StMdl.solCov(obj.P, obj.Q);
      obj.P = Pkp1;
      obj.x = xkp1;
      
      % Update inputs
      obj.u = u;
    end
    
    function measurementUpdate(obj, z)
      % Correction Step
      xkp1 = obj.x;
      Pkp1 = obj.P;  
      Hk = obj.MsMdl.GetMtrxC();

      obj.yk = z - (Hk * xkp1);
      obj.Sk = (Hk * Pkp1 * Hk') + obj.R;           
      Kk = (Pkp1 * Hk') / obj.Sk;
      xkp1 = xkp1 + (Kk * obj.yk);
      Pkp1 = (eye(length(obj.x)) - Kk * Hk) * Pkp1;            
      obj.dk = z - (Hk * xkp1);    
      obj.x = xkp1;
      obj.P = Pkp1;
    end
         
    function stateEstimate = getStateEstimate(obj)
      stateEstimate.mean = obj.x;
      stateEstimate.covariance = obj.P;
      stateEstimate.residual = obj.dk;
    end
  end
end

