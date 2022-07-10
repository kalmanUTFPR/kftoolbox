% Author: Renata da Costa Ribeiro
% Data Generator template

close all;
clear all;
clc

disp('Run Open-Loop Simulation')
p = parameters_vehicle();
dt = 0.01;
model = cKST4disc(p, dt);

% A =  [                0.999925125623905	 0.00997483369859280	    -1.24529694925830e-07	2.48747933857088e-05	1.24529694925830e-07	3.11248449216378e-10;
%                     -0.0149622505478892	   0.994950146171301	-3.73117232058894e-05	 0.00496241752621576	3.73117232058894e-05	1.24374070701222e-07;
%                   -1.24530629297504e-07	2.48747933857088e-05	    0.999925251089214	 0.00994995890707583	7.47489107864239e-05	2.49061258595008e-07;
%                   -3.73121900785632e-05	 0.00496241752621576	  -0.0149245647687259	   0.989987729578831	  0.0149245647687259	7.46243801571263e-05;
%                   -9.34371673754423e-13	3.11248449216377e-10	 7.48732857914967e-05	2.49061258595008e-07	   0.999925126714209	 0.00997479224243797;
%                   -4.66872673824566e-10	1.24374070701222e-07	   0.0149618147717691	7.46243801571263e-05	 -0.0149618147717691	   0.994937730592990]; 
% 
%     B = [2.07551048829174e-08;
%                6.21869834642719e-06;
%                1.24583075784695e-05;
%                0.00248748972676896;
%                1.55780732236258e-10;
%                6.22653146487521e-08];
% 
% 
% datalen = 1500; 
% 

% 
loc_uinp = load('inputs_kst.mat');
loc_u = loc_uinp.u;

loc_u0 = loc_u(:,1);
datalen = length(loc_u);
loc_x0 = [0; 0; 0; 0];
loc_x = zeros(length(loc_x0), datalen);
loc_x(:,1) = loc_x0;

% 
for i=1:1:datalen
    if i==1
        loc_x(:,i) = model.solver(loc_x0, loc_u0);
    else
        loc_x(:,i) = model.solver(loc_x(:,i-1), loc_u(:,i));
    end
end
% 
% 
figure();
plot(loc_x(1,:), loc_x(2,:), 'r');


% plot(loc_x(5,:), 'm');
% plot(loc_x(6,:), '--k');
% 
% 
 measData = [loc_x(1,:) + sqrt(0.2)*randn(1, datalen);
          loc_x(2,:) + sqrt(0.2)*randn(1, datalen);
          loc_x(4,:) + sqrt(0.2)*randn(1,datalen)];
% 
 figure();
plot(loc_x(1,:), loc_x(2,:), 'k'); hold on;
plot(measData(1,:), measData(2,:), 'b');
% 
 figure();
 plot(loc_x(4,:), 'k'); hold on;
 plot(measData(3,:), 'b'); 
% 
% figure();
% plot(loc_x(5,:), 'k'); hold on;
% plot(measData(3,:), 'b'); 