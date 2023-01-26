clear all;close all;clc

if ~exist('LiftLinearSystem')
    addpath(['..',filesep,'lift_lumped_parater_model\'])
    addpath(['..',filesep,'lift_lumped_parater_model\NonLinearElasticModel'])
end

if ~exist('LiftLogic')
    addpath(['..',filesep,'LiftMotionLaw\'])
end


%% MODEL PARAMETERS
floor_height=3;  % height of a single floor
num_floors=20;   % number of floors of the building
min_length=3;    % distance between the cabin and the pullet at the maximum position (minimal lenght of the rope)
Jp=1;            % Pulley inertia
Jm=0.1;          % Motor inertial
Mc=600;          % Mass of the cabin (no people)
Mw=1140;         % Mass of the counterweight
MotorViscousFriction=1;
Rp=0.5;
g=9.806;
mu=0.8;
gearbox=1;

max_load=2*(Mw-Mc);  % maximum load
BuildingHeight=num_floors*floor_height; 
cabin_height=2.8;



%% estimation of the stiffness
% assumption => deflection of the rope at the maximum extension is 
deflection=2e-2; % in meter
stiffness=(Mc+max_load)*g/deflection;

LinearStiffness=stiffness*(BuildingHeight+min_length); % stiffness=LinearStiffness/max rope length

wn=sqrt(stiffness/(Mc+max_load)); %natural frequency at the maximum extension

%% estimation of the damping
% assumption => damping coefficient  at the maximum extension
% of the mass-spring-damper on the cabin side is

damping_coefficient=0.1; %2*damping_coefficient*wn=damping/(Mc+max_load)
damping=damping_coefficient*(Mc+max_load)*(2*wn);


LinearDamping=damping*(BuildingHeight+min_length);   

%% Motion Profile
motion_profile=1;  % 1: single speed, 2: dual speed(not implemented yet)
proximity_distance=1; % distance of proximity sensor w.r.t. the floor (used in profile 2: dual speed)

MaxVel=1.5; % max velocity of the lift (m/s)
MinVel=0.2; % low velocity of the lift (m/s)
MaxAcc=1;   % max acceleration of the lift (m/s^2)
MaxJerk=0.5%5;  % max jerk of the lift (m/s^3)

waiting_time_open_door=2;    % Time to open the doors
waiting_time_close_door=2;   % Time to close the doors
waiting_time_close_brake=1;  % Time to close the brake
waiting_time_open_brake=1;   % Time to open the brake
waiting_time_steady_state=5; % Time to reach zero velocity
cabin_offset=0; % it should be tuned based on the control performance

Ts=1e-3; % Sample period


if (motion_profile==1)
    if (MaxVel^(1/2)*MaxJerk^(1/2)>MaxAcc) % case 1
        deceleration_distance=(CruiseVel*(MaxAcc^2 + MaxVel*MaxJerk))/(2*MaxAcc*MaxJerk);
    else % case 2
        deceleration_distance =MaxVel^(3/2)/MaxJerk^(1/2);
    end
    if (deceleration_distance>=min(floor_height,cabin_height))
        error('Wrong limits: deceleration distance is %4.2f m while the switch distance is %4.2f m\n',deceleration_distance,floor_height);
    end
end

%% LINEARIZAZION

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


total_inertia_on_motor_side=(Jm+gearbox^2*(Jp+(Mw+Mc)*Rp^2));

%% tune controller (PI+ LowPassFilter)


if 1
    wc=30; % cut frequency
    filter=c2d(tf(1,[1/(10*wc) 1]),Ts); % filter frequency = 10*wc
    
    % the lift has an unstable pole (without control, it falls)
    opt = pidtuneOptions('NumUnstablePoles',sum(abs(pole(sysd))>=1));
    
    % use PIDTuner
    pid_ctrl=pidtune(sysd*filter,'PI',wc,opt);

    ctrl=pid_ctrl*filter;
else
    filter=c2d(tf(1,[1/(20) 1]),Ts);
    pid_ctrl=pid(1600,1200,'Ts',Ts);
    ctrl=pid_ctrl*filter;
end
margin(sysd*ctrl)
