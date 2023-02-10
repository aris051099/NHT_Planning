Kp_t = 229.58;%Wheel torque proportional gain 
K_u = 0.0277;%Longitudinal viscous coeff
K_y = 0.0298;%Lateral motion viscous coeff
m = 73.30 %kg;%mass
r = 0.165;%m  %radius of the wheel
g = 9.81;%kg/m %Gravity 
c = 0.67;%m %Half-Width 
I = 2.16%kgm2;
dt = 1/100; %Sample period 
t = 2*c; %The total width 

A = [0,1,0,0;...
    0,-2*(Kp_t/(r*m))-m*g*K_u,0,0;...
    0,0,0,1;...
    0,0,0,(1/I)*(((-Kp_t*t^2)/(2*r))-m*g*K_y*c)];
B = [0,0;...
    2*Kp_t/(r*m),0;...
    0,0;...
    0,(Kp_t*t^2)/(2*I*r)];
C = eye(4);
D = 0;

S_cont = ss(A,B,C,D)
S_dt = c2d(S_cont,dt)