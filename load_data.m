clear all;close all;clc

if ~exist('LiftLinearSystem')
    addpath(['..',filesep,'lift_lumped_parater_model\'])
    addpath(['..',filesep,'lift_lumped_parater_model\NonLinearElasticModel'])
end

if ~exist('LiftLogic')
    addpath(['..',filesep,'LiftMotionLaw\'])
end

%addpath('NonLinearElasticModel')
%addpath('MotionLaw')
floor_height=3;
num_floors=20;
BuildingHeight=num_floors*floor_height;
min_length=3;
Jp=1;
Jm=0.1;
Mc=600;
Mw=1140;


LinearDamping=180000;
LinearStiffness=21000000;

MotorViscousFriction=1;
Rp=0.5;
g=9.806;
mu=0.8;
gearbox=1;

max_load=2*(Mw-Mc);

cabin_height=2.8;
proximity_distance=1;



MaxVel=1.5;
MinVel=0.2;
MaxAcc=1;
MaxJerk=5;

switch_distance=1;
waiting_time=8;
motion_profile=1;
cabin_offset=2.5; % it should be tuned based on the control performance

Ts=1e-3;



LinearizationCabinPosition=BuildingHeight*0.5; %meter
dc=0; % force on cabin in the linearizing point
dw=0; % force on counterweight in the linearizing point
% compute equilibrium in the linearizing point
[x_eq,u_eq] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,dc,dw,g,gearbox,min_length,mu,LinearizationCabinPosition); 
% linearized system
[A,B] = LiftLinearSystem(BuildingHeight,Jm,Jp,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,dc,dw,g,gearbox,min_length,mu,x_eq(9));
C=[0 0 0 0 0 0 0 0 0 1]; % velocity control
D=0;
sys=ss(A,B,C,D);


sysd=c2d(sys,Ts);
%% tune controller


% wc=3;
% opt = pidtuneOptions('NumUnstablePoles',sum(abs(pole(sysd))>=1));
% filter=c2d(tf(1,[1/(10*wc) 1]),Ts);
% pid_ctrl=pidtune(ss(sysd)*filter,'PI',wc,opt);

filter=c2d(tf(1,[1/(20) 1]),Ts);
pid_ctrl=pid(1600,1200,'Ts',Ts);
ctrl=pid_ctrl*filter;
margin(sysd*ctrl)
