clear all;close all;clc

if ~exist('LiftLinearSystem')
    addpath(['..',filesep,'IMOCO_LIFT_MODEL\'])
    addpath(['..',filesep,'IMOCO_LIFT_MODEL\NonLinearElasticModel'])
end

if ~exist('LiftLogic')
    addpath(['..',filesep,'IMOCO-Lift-Motion-Law\'])
    addpath(['..',filesep,'IMOCO-Lift-Motion-Law\internal_and_tests'])
end


%% MODEL PARAMETERS
weg_tower=true;
if (weg_tower)
    num_floors=3;   % number of floors of the building
    min_length=3;    % distance between the cabin and the pullet at the maximum position (minimal lenght of the rope)
    Jp=1;            % Pulley inertia
    Jm=0.1;          % Motor inertial
    Mc=442;          % Mass of the cabin (no people)
    Mw=1240;         % Mass of the counterweight
    MotorViscousFriction=1;
    Rp=0.32; % pulley radius
    g=9.806;
    mu=0.8;
    gearbox=1;
    BuildingHeight=12.4489;
    FloorHeight=[0 4.2321 12.4489];
else
    floor_height=3;  % height of a single floor
    num_floors=20;   % number of floors of the building
    min_length=3;    % distance between the cabin and the pullet at the maximum position (minimal lenght of the rope)
    Jp=1;            % Pulley inertia
    Jm=0.1;          % Motor inertial
    Mc=600;          % Mass of the cabin (no people)
    Mw=1140;         % Mass of the counterweight
    MotorViscousFriction=1;
    Rp=0.5; % pulley radius
    g=9.806;
    mu=0.8;
    gearbox=1;
    BuildingHeight=num_floors*floor_height;
    FloorHeight=(0:num_floors-1)*floor_height;
end

max_load=2*(Mw-Mc);  % maximum load
sensor_height=0.1;



%% estimation of the stiffness
% assumption => deflection of the rope at the maximum extension is
if weg_tower
    deflection=1e-3;
else
    deflection=2e-2; % in meter
end
stiffness=(Mc+max_load)*g/deflection;

LinearStiffness=stiffness*(BuildingHeight+min_length); % stiffness=LinearStiffness/max rope length

wn=sqrt(stiffness/(Mc+max_load)); %natural frequency at the maximum extension

%% estimation of the damping
% assumption => damping coefficient  at the maximum extension
% of the mass-spring-damper on the cabin side is
if weg_tower
    damping_coefficient=0.2; %2*damping_coefficient*wn=damping/(Mc+max_load)
else
    damping_coefficient=0.1;
end
damping=damping_coefficient*(Mc+max_load)*(2*wn);


LinearDamping=damping*(BuildingHeight+min_length);

%% Motion Profile DATA
HighSpeed = 1.0;
LowSpeed = 0.07;
PercentAccFactor = 1.0;

AccelInitialJerk = 0.2;
Acceleration = 0.6;
AccelEndJerk = 0.6;
DecelInitialJerk = 0.6;
Deceleration = 0.31;
DecelEndJerk = 0.5;
StopDeceleration = 0.3;

waiting_time_open_door=2;    % Time to open the doors
waiting_time_close_door=2;   % Time to close the doors
waiting_time_close_brake=1;  % Time to close the brake
waiting_time_open_brake=1;   % Time to open the brake
waiting_time_steady_state=5; % Time to reach zero velocity
cabin_offset=0; % it should be tuned based on the control performance

Ts=1e-3; % Sample period

%% Switch position

if weg_tower
    lower_floor_sensor_positions=    [-.1   2.335  10.5];
    lower_proximity_sensor_positions=[-.01  4.21   12.43];
    upper_proximity_sensor_positions=[0.017 4.2549 12.45];
    upper_floor_sensor_positions=    [1.892 6.166  12.45];
else
    [t1a,t2a,t3a,deceleration_distance]=computeSwitchDistance(DecelInitialJerk*PercentAccFactor,DecelEndJerk*PercentAccFactor,0,HighSpeed,LowSpeed,Deceleration*PercentAccFactor);
    [t1b,t2b,t3b,proximity_deceleration_distance]  =computeSwitchDistance(DecelInitialJerk*PercentAccFactor,DecelEndJerk*PercentAccFactor,0,LowSpeed,0,StopDeceleration*PercentAccFactor);
    lower_floor_sensor_positions=floor_height*(0:num_floors-1)'-proximity_deceleration_distance-deceleration_distance;
    upper_floor_sensor_positions=floor_height*(0:num_floors-1)'+proximity_deceleration_distance+deceleration_distance;
    lower_proximity_sensor_positions=floor_height*(0:num_floors-1)'-proximity_deceleration_distance;
    upper_proximity_sensor_positions=floor_height*(0:num_floors-1)'+proximity_deceleration_distance;
end
%%


%% LINEARIZAZION (TEST AND CONTROL PART)

LinearizationCabinPosition=BuildingHeight*0.5; %meter
dc=0; % force on cabin in the linearizing point
dw=0; % force on counterweight in the linearizing point
% compute equilibrium in the linearizing point
[x_eq,u_eq] = LiftEquilibrium(BuildingHeight,LinearStiffness,Mc,Mw,Rp,dc,dw,g,gearbox,min_length,mu,LinearizationCabinPosition);
% linearized system
[A,B,B_dc] = LiftLinearSystem(BuildingHeight,Jm,Jp,LinearDamping,LinearStiffness,Mc,MotorViscousFriction,Mw,Rp,dc,dw,g,gearbox,min_length,mu,x_eq(9));
C=[0 0 0 0 0 0 0 0 0 1]; % motor velocity control
Cp=[0 0 0 0 0 0 0 0 1 0]; % motor position
Ccabin=[0 0 0 0 0 0 1 0 0 0]; % cabin position
D=0;
mimo_sys=ss(A,[B,B_dc],[C;Cp;Ccabin],D);
mimo_sys.OutputName={'motor vel','motor pos','cabin pos'};
mimo_sys.InputName={'motor torque','force on cabin'};
sys=mimo_sys(1,1);
sysd=c2d(sys,Ts);




total_inertia_on_motor_side=(Jm+gearbox^2*(Jp+(Mw+Mc)*Rp^2));

%% tune controller (PI+ LowPassFilter)


if 1

    sys_poses=pole(sys);
    [Wn,damping]=damp(sys_poses);

    resonance_poles=boolean((damping<0.8).*(damping>0));
    damping_notch=0.8;
    notch_poles=-Wn(resonance_poles)*(damping_notch+ 1i*sqrt(1-damping_notch^2));
    notch_poles(2:2:end)=real(notch_poles(2:2:end))-1i*imag(notch_poles(2:2:end));
    filter_resonance=zpk(sys_poses(resonance_poles),[notch_poles' -5*max(Wn(resonance_poles))],1);
    filter_resonance=filter_resonance/dcgain(filter_resonance);
    filter_resonance_d=c2d(filter_resonance,Ts);
    if (weg_tower)
        wc=50;
    else
        wc=10; % cut frequency
    end
    filter=c2d(tf(1,[1/(10*wc) 1]),Ts); % filter frequency = 10*wc

    % the lift has an unstable pole (without control, it falls)
    opt = pidtuneOptions('NumUnstablePoles',sum(abs(pole(sysd))>=1));

    % use PIDTuner
    pid_ctrl=pidtune(sysd*filter*filter_resonance_d,'PI',wc,opt);

    ctrl=pid_ctrl*filter*filter_resonance_d;
else
    filter=c2d(tf(1,[1/(20) 1]),Ts);
    pid_ctrl=pid(1600,1200,'Ts',Ts);
    ctrl=pid_ctrl*filter;
end
margin(sysd*ctrl)
