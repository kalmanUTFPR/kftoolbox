% Author: Renata da Costa Ribeiro
% Title: Unscented Kalman Filter - Standard formulation
% Date: August, 2020
%
% Paper behind: Sigma-Point Kalman Filters for Probabilistic
% Inference in Dynamic State-Space Models

classdef cUKF < cFilter
  properties
    hMdl   = [];            % dynamic model - f(x,u)
    hMeas  = [];            % Measurement model - h(x,u)
    method = [];            % Unscented transform method to compute the sigma-points: spherical or symmetrical.
    kappa = [];             % Unscented transform scaling parameter, usually equal to 3-numStates (according to Merwe)
    alpha = [];             % Unscented transform scaling parameter, usually equal to 1 (cannot be 0!, according to NASA library of KFs)
    beta = [];              % Unscented transform scaling parameter, usually equal to 2 (according to Merwe)
    sigmaPoints = [];       % Sigma point struct array 
    P = [];                 % Covariance matrix 
    x = [];                 % State vector
    Q = [];                 % process noise
    R = [];                 % measurement noise
    yk = [];                % innovation (prediction - measurement)
    dk = [];                % residual (estimation - measurement)   
  end    
  methods  
    function obj = cUKF(hMdl,hMeas,method,kappa,alpha,beta,Q,R)
      obj.hMdl = hMdl;
      obj.hMeas = hMeas;
      obj.method = method;
      obj.kappa = kappa; 
      obj.alpha = alpha;
      obj.beta = beta;
      obj.Q = Q;
      obj.R = R;
    end  
    
    function initialize(obj,x0,u0,P0)
      obj.x = x0;
      obj.u = u0;
      obj.P = P0;  
      
      % Intialize sigma points            
      [X,W] = computeSigmaPoints(obj.x,obj.P,obj.method,obj.kappa,obj.alpha, obj.beta); % Compute Set of sigma points
      obj.sigmaPoints.x = X;
      obj.sigmaPoints.w = W;
    end 
    
    function prediction(obj,u)
      xa = obj.x;
      Pa = obj.P;
      [X,W] = computeSigmaPoints(xa,Pa,obj.method,obj.kappa,obj.alpha,obj.beta);
      Xkp1 = X;
      
      % Propagate sigma points through transition function
      for i = 1:length(Xkp1)
        Xkp1(:,i) = obj.hMdl.solver(Xkp1(:,i),u);
      end
      %Xkp1(1:length(obj.x),:) = obj.hMdl.solver(Xkp1(1:length(obj.x),:), u);
      xkp1 = wmean(Xkp1,W);
      Pkp1 = wcov(Xkp1,W,obj.alpha,obj.beta) + obj.Q;
      obj.x = xkp1;
      obj.P = Pkp1;
      obj.sigmaPoints.x = Xkp1;
      obj.sigmaPoints.w = W;
      obj.u = u; 
    end  
    
    function measurementUpdate(obj, z)   
      xkp1 = obj.x;
      Pkp1 = obj.P;
      Xkp1 = obj.sigmaPoints.x;
      W = obj.sigmaPoints.w;
      for i = 1:length(Xkp1)
        Zkk1(:,i) = obj.hMeas.solver(Xkp1(:,i));
      end 
      %Zkk1 =  obj.hMeas.solver(Xkp1); 
      z = obj.hMdl.alignangles(xkp1,z);
      
      % Recombine weighted sigma points to produce predicted
      % measurement and covariance
      zkk1 = wmean(Zkk1,W);
      Pzz = wcov(Zkk1,W,obj.alpha,obj.beta) + obj.R;
      
      % Compute state-measurement cross-covariance matrix
      Pxz = 0;
      for i=1:length(W)
        Pxz = Pxz + W(i)*(Xkp1(:,i)-xkp1)*(Zkk1(:,i)-zkk1)';
      end
      Pxz = Pxz + (1 - obj.alpha ^ 2 + obj.beta) * (Xkp1(:,1) - xkp1) * (Zkk1(:,1) - zkk1)';
      
      % Compute kalman gain
      Kk = Pxz / Pzz;
      obj.yk = z-zkk1;  % Innovation
      xk = Xkp1 + Kk * obj.yk;
      xk = obj.hMdl.wrapOut(xk);
      obj.x = xk(1:length(obj.x))';
      Pk1 = Pkp1 - Kk * Pzz * Kk';
      obj.P = Pk1(1:length(obj.x),1:length(obj.x));
      Zkk1 =  obj.hMeas.solver(Xkp1);
      zkk1 = wmean(Zkk1,W);
      obj.dk = z - zkk1;
      
      % Compute sigma points
      [X,~] = computeSigmaPoints(xkp1,Pkp1,obj.method,obj.kappa,obj.alpha, obj.beta);
      obj.sigmaPoints.x = X;
      obj.sigmaPoints.w = W;      
    end
    
    function stateEstimate = getStateEstimate(obj)
      stateEstimate.mean = obj.x;
      stateEstimate.covariance = obj.P;
      stateEstimate.residual = obj.dk;
      stateEstimate.r = obj.R;
    end       
  end
end









