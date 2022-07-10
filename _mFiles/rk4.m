%Function Runge-Kutta method (4th):
function [xout] = rk4(func,Ts,x0,u)

k1 = Ts*func(x0,u);

k2_ = x0 + 0.5*k1;
k2 = Ts*func(k2_,u);

k3_ = x0 + 0.5*k2;
k3 = Ts*func(k3_,u);

k4_ = x0 + k3;
k4 = Ts*func(k4_,u);

xout = x0 + 1/6*(k1 + 2*k2 + 2*k3 + k4);
end
