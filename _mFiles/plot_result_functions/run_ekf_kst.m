function run_ekf_kst(arg_axes_state1, arg_axes_state2, arg_axes_state3, arg_axes_state4, arg_axes_traj, r_factor, q_factor)
% Set the parameters
p = parameters_vehicle();

dt = 0.01;

% inputs
loc_uinp = load('inputs_kst.mat');
uinp = loc_uinp.u(:,:);

% Time config
datalen = length(uinp);
t_sim = 0:dt:(datalen*dt-dt);

% Measurement and process noise
R = 10^r_factor*(diag([0.0022;0.0044;0.0089]));
Q = 10^q_factor*(diag([0.0022;0.0044;0.0055;0.0089])); 

% Measurement data
measMatrix  = importdata('measDataKST.mat');

% Models
ohMdl = cKST4disc(p, dt); % state model
hmtrx = [1 0 0 0; 
         0 1 0 0;
         0 0 0 1];           % measurement model
oxjac = cJAC4FX(p,dt);    % Jacobian class for the state equation

%  Initial X
sx0      = 0;
sy0      = 0;
Psi0     = 0;
vel0     = 0;
loc_x0   = [sx0; sy0;  Psi0; vel0;];

% Initial U
loc_u0 = uinp(:,1);

% Initial P
loc_P0 = diag([1;1;1;1])*10^-2;
         
% Alocations
XEst  = zeros(length(loc_x0), datalen);
PEst  = zeros(length(loc_x0), length(loc_x0), length(t_sim));
XPred = zeros(length(loc_x0), datalen);
PPred = zeros(length(loc_x0), length(loc_x0), length(t_sim));
XPredOnly = zeros(length(loc_x0), datalen);
PPredOnly = zeros(length(loc_x0), length(loc_x0), length(t_sim));

XEst(:,1) = loc_x0;
PEst(:,:,1) = loc_P0;
XPred(:,1) = loc_x0;
PPred(:,:,1) = loc_P0;

%% RUN FILTER
oekf = cEKF(ohMdl, oxjac, hmtrx, Q, R);
oekf.initialize(loc_x0, loc_u0, loc_P0);

for i = 1 : datalen 
    u = uinp(:,i);    
    oekf.prediction(u);    
    stateEstimate = oekf.getStateEstimate();
    PPredOnly(:,:,i) = stateEstimate.covariance;
    XPredOnly(:,i) = stateEstimate.mean;
end

%% RUN FILTER
oekf.initialize(loc_x0, loc_u0, loc_P0);

for i = 1 : datalen 
    u = uinp(:,i);    
    oekf.prediction(u);    
    stateEstimate = oekf.getStateEstimate();
    XPred(:,i) = stateEstimate.mean;
    PPred(:,:,i) = stateEstimate.covariance;

    z = measMatrix(:,i);    
    oekf.measurementUpdate(z);

    stateEstimate = oekf.getStateEstimate();

    XEst(:,i) = stateEstimate.mean;
    PEst(:,:,i) = stateEstimate.covariance;
end

hold(arg_axes_traj,'on');
plot(arg_axes_traj, XEst(1,:), XEst(2,:), 'ob'); 
plot(arg_axes_traj, measMatrix(1,:), measMatrix(2,:), '^m');
plot(arg_axes_traj, XPredOnly(1,:), XPredOnly(2,:), '*r');
axis(arg_axes_traj, 'equal');
hold(arg_axes_traj,'off');
legend(arg_axes_traj,'Estimation','Measurement', 'Open-Loop');

hold(arg_axes_state1,'on');
plot(arg_axes_state1, XEst(1,:), 'ob'); 
plot(arg_axes_state1, measMatrix(1,:), '^m');
plot(arg_axes_state1, XPred(1,:), '*r');
hold(arg_axes_state1,'off');
legend(arg_axes_state1,'Estimation','Measurement', 'Open-Loop');

hold(arg_axes_state2,'on');
plot(arg_axes_state2, XEst(2,:), 'ob'); 
plot(arg_axes_state2, measMatrix(2,:), '^m');
plot(arg_axes_state2, XPred(2,:), '*r');
hold(arg_axes_state2,'off');
legend(arg_axes_state2,'Estimation','Measurement', 'Open-Loop');

hold(arg_axes_state3,'on');
plot(arg_axes_state3, XEst(3,:), 'ob'); 
plot(arg_axes_state3, XPred(3,:), '*r');
hold(arg_axes_state3,'off');
legend(arg_axes_state3,'Estimation', 'Open-Loop');

hold(arg_axes_state4,'on');
plot(arg_axes_state4, XEst(4,:), 'ob'); 
plot(arg_axes_state4, measMatrix(3,:), '^m');
plot(arg_axes_state4, XPred(4,:), '*r');
hold(arg_axes_state4,'off');
legend(arg_axes_state4,'Estimation','Measurement', 'Open-Loop');
end