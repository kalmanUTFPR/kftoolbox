function run_kf_rlc(arg_axes_state1, arg_axes_state2, arg_axes_state3 ,r_factor, q_factor) 
% Input data
loc_u = ones(1, 2500);

% Measurement data
measMatrix = load('measDataRLC.mat');

% Time config.
datalen = length(loc_u');
t_sim = 0:1:(datalen - 1);

% Measurement and process noises
R = (10^r_factor)*eye(1);                           
Q = (10^q_factor)*eye(3);

oStMdl = cRLCStMdl();
oMsMdl = cRLCMeasMdl();

%  Initial X
loc_x0   = [0; 0; 0];

% Initial U
loc_u0 = loc_u(:,1);

% Initial P
loc_P0 = eye(3);

% Allocations
XEst  = zeros(length(loc_x0), datalen);
PEst  = zeros(length(loc_x0), length(loc_x0), length(t_sim));
XPred = zeros(length(loc_x0), datalen);
PPred = zeros(length(loc_x0), length(loc_x0), length(t_sim));
XOpenLoop = zeros(length(loc_x0), datalen);

XEst(:,1) = loc_x0;
PEst(:,:,1) = loc_P0;
XPred(:,1) = loc_x0;
PPred(:,:,1) = loc_P0;
XOpenLoop(:,1) = loc_x0; 

%% RUN OPEN-LOOP
okf = cKF(oStMdl, oMsMdl, Q, R);
okf.initialize(loc_x0, loc_u0, loc_P0); 

for i = 2 : datalen
    u = loc_u(:,i-1);
    okf.prediction(u);    
    stateEstimate = okf.getStateEstimate();
    XOpenLoop(:,i) = stateEstimate.mean;
end

%% RUN FILTER
okf = cKF(oStMdl, oMsMdl, Q, R);
okf.initialize(loc_x0, loc_u0, loc_P0);

for i = 2 : datalen
    u = loc_u(:,i-1);
    okf.prediction(u);    
    stateEstimate = okf.getStateEstimate();
    XPred(:,i) = stateEstimate.mean;
    PPred(:,:,i) = stateEstimate.covariance;
          
    z = measMatrix.measData(:,i);
    
    okf.measurementUpdate(z);    
    stateEstimate = okf.getStateEstimate();
    XEst(:,i) = stateEstimate.mean;
    PEst(:,:,i) = stateEstimate.covariance;
end

hold(arg_axes_state1,'on');
plot(arg_axes_state1, XEst(1,:), 'ob'); 
plot(arg_axes_state1, XOpenLoop(1,:), '*r');
hold(arg_axes_state1,'off');
legend(arg_axes_state1,'Estimativa', 'Modelo Dinâmico');

hold(arg_axes_state2,'on');
plot(arg_axes_state2, XEst(2,:), 'ob'); 
plot(arg_axes_state2, XOpenLoop(2,:), '*r');
hold(arg_axes_state2,'off');
legend(arg_axes_state2,'Estimativa', 'Modelo Dinâmico');

hold(arg_axes_state3,'on');
plot(arg_axes_state3, XEst(3,:), 'ob'); 
plot(arg_axes_state3, XOpenLoop(3,:), '*r');
plot(arg_axes_state3, measMatrix.measData(1,1:datalen), 'sk');
hold(arg_axes_state3,'off');
legend(arg_axes_state3,'Estimativa', 'Modelo Dinâmico', 'Medidas');
end

