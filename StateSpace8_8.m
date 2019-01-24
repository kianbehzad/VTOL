clc
close all
clear all
disp('Mass of motors and fans')
mf = 0.4; % mass of each front motor/prop (kg)
mb = 0.1; % mass o rear motor/fan (kg)
disp('Total mass')
Mp = 4; %Mass of plane (kg)
%disp('Length of fuselage (m)')
l1 = .05;
l2 = .8;
l3 = .5;
% Will need to work these out somehow
%disp('Moment of intertia for a single motor/fan')
J = mf;
Jre = mb;
%disp('Moments of inertia about each axis')
Jy = 0.25934; %calculated on solid edge
Jp = 0.094625;
Jr = 0.19712;
% prop weighs 0.0433lbs = 195g.
Jpr = 2.55e-3; % moment of inertia of prop, see p31 of allans book
disp('Motor/Fan Force and Torque constants')
% some of these constants are not in use currently as they have been
% replaced by experimentally determined values
Vhov = 8; % Voltage supplied to front motors for hover
kD = .004; % drag constant of props (assumed from prop calc)
kt = .1901; % torque constant of motor (mN.m/A) (published on website)
kB = 0; % torque constant of rear motor (Nm/A)
keb = kB;
Ra = .29; % armature resistance (ohms)
CT = 0.0828; %thrust coefficient for propeller 20-10 estimated propcalc
kl = 0.7; % coefficient of lift for props
klrear = 0.5; % coefficient of lift for rear fan
ke = kt; %back emf constant = torque const in SI units
Kmp = ((.1901)^2/0.56 + 0.008*3000/60); %(bm + kt*ke/Ra + kD*theta_0_dot);% was 0.06;
Kd = 0; % kD*theta_0_dot*kt/(Ra*Kmp); % Nm per volt Front Props (not measured)
Kdr = 0; % Nm per volt Rear Prop/Fan (not measured)
%Kp = kl*theta_0_dot*kt/(Ra*Kmp);
% From Thrust Testing
thr = 2.5*9.81; % from full power thrust test on 12V battery 49N for both props;
Kp = thr/10; % N x volt Front Props;
Kr = .15/10; % N per volt Rear Prop/Fan;
Pd = l1*Kp/Jp;
Rd = l3*Kp/Jr;
Zd = Kp/Mp;


A =[ 0  	0       0       0       0       0       0       0           0
     0      0       0       0       0       1       0       0           0
     0      0       0       0       0       0       1       0           0
     0      0       0       0       0       0       0       Zd          Zd
     0  	0       0       0       0       0       0       0           0   %%Z (we added)
     0      0       0       0       0       0       0       -Pd         -Pd
     0      0       0       0       0       0       0       -Rd         Rd
     0      0       0       0       0       0       0       -Kmp/Jpr    0
     0      0       0       0       0       0       0       0           -Kmp/Jpr ];



B  = [ Kd               -Kd              Kdr         l3*Kp*Vhov/Jy       -l3*Kp*Vhov/Jy 
       0                0                0           0                   0
       0                0                0           0                   0
       0                0                -Kr/Mp      0                   0
       0                0                0           0                   0  %%Z (we added)
       0                0                l2*Kr/Jp    0                   0
       0                0                0           0                   0 
       kt/(Ra*Jpr)      0                0           0                   0 
       0                kt/(Ra*Jpr)      0           0                   0  ]; 




C = [0     0       0       1       0       0       0       0       0];  %%output = z'


D = zeros(1, 5);


StateS = ss(A,B,C,D);
allTransferFunctions = tf(StateS);

[Zdot_VL_num, Zdot_VL_de] = tfdata(allTransferFunctions(1,1),'v');
Zdot_VL = tf(Zdot_VL_num, Zdot_VL_de);

[Zdot_VR_num, Zdot_VR_de] = tfdata(allTransferFunctions(1,2),'v');
Zdot_VR = tf(Zdot_VR_num, Zdot_VR_de);

[Zdot_VB_num, Zdot_VB_de] = tfdata(allTransferFunctions(1,3),'v');
Zdot_VB = tf(Zdot_VB_num, Zdot_VB_de);














