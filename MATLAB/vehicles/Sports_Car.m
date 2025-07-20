%% Vehicle Parameters: Sports Car          %%
% ----------------------------------------- %
% Version: V1.1 - 2024.12.26                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Generic CV template                 %
% ----------------------------------------- %

%% Tire selection
tireFL  = 'Road_Tire_235_45R18';
tireFR  = 'Road_Tire_235_45R18';
tireRL  = 'Road_Tire_265_45R18';
tireRR  = 'Road_Tire_265_45R18';

%% Chassis
massVehicle = 1500; % Whole car weight in kg
Wheelbase   = 2.475;% Distance between axles in m
trackFA     = 1.515;% Distance between front wheel center points in m
trackRA     = 1.530;% Distance between rear wheel center points in m
ratioMassFA = 0.50; % Ratio of mass on front axle in -
heightCoG   = 0.10; % CoG height above wheel center plane in m

IChassisXX  = 1000;  % Chassis inertia around X-axis in kg*m^2
IChassisYY  = 1500;  % Chassis inertia around Y-axis kg*m^2
IChassisZZ  = 2000;  % Chassis inertia around Z-axis kg*m^2

%% Springs & dampers
cSpringFA       = 80E3/1.05^2; % Front spring rate (reduced by motion ratio^2) in N/m
cSpringRA       = 90E3/1.20^2; % Rear spring rate (reduced by motion ratio^2) in N/m

cSpringHeaveFA  = 0; % Front heave spring rate (reduced by motion ratio^2) in N/m
cSpringHeaveRA  = 0; % Rear heave spring rate (reduced by motion ratio^2) in N/m

cSpringRollFA   = 25E3/1.05^2; % Front roll spring or ARB rate (reduced by motion ratio^2) in N/m
cSpringRollRA   = 10E3/1.20^2; % Rear roll spring or ARB rate (reduced by motion ratio^2) in N/m

vDamperFA       = [-0.2 -0.05 0 0.05 0.2].*1.05;% Front damper speeds (increased by motion ratio) LUT in m/s
FDamperFA       = [-900 -400 0 400 900]./1.05;  % Front damper forces (reduced by motion ratio) LUT in N
vDamperRA       = [-0.2 -0.05 0 0.05 0.2].*1.20;% Rear damper speeds (increased by motion ratio) LUT in m/s
FDamperRA       = [-900 -400 0 400 900]./1.20;  % Rear damper forces (reduced by motion ratio) LUT in N

vDamperHeaveFA  = 0; % Front heave damper speeds (increased by motion ratio) LUT in m/s
FDamperHeaveFA  = 0; % Front heave damper forces (reduced by motion ratio) LUT in N
vDamperHeaveRA  = 0; % Rear heave damper speeds (increased by motion ratio) LUT in m/s
FDamperHeaveRA  = 0; % Rear heave damper forces (reduced by motion ratio) LUT in N

vDamperRollFA   = 0; % Front roll damper speeds (increased by motion ratio) LUT in m/s
FDamperRollFA   = 0; % Front roll damper forces (reduced by motion ratio) LUT in N
vDamperRollRA   = 0; % Rear roll damper speeds (increased by motion ratio) LUT in m/s
FDamperRollRA   = 0; % Rear roll damper forces (reduced by motion ratio) LUT in N

%% Aerodynamics
projectArea =  2.00;    % Frontal area in m^2
dragCoef    = -0.30;    % Drag coefficient (must be negative) in -
liftCoefFA  =  0.05;    % Front axle lift coefficient (negative for downforce) in -
liftCoefRA  = -0.10;    % Rear axle lift coefficient (negative for downforce) in -

%% Brakes
APstBFA     = 0.008;    % Brake piston area per front wheel in m^2
rmDiscBFA   = 0.165;    % Front brake disc radius in m
muPadBFA    = 0.33;     % Front brake disc friction coefficient in -

APstBRA     = 0.004;    % Brake piston area per rear wheel in m^2
rmDiscBRA   = 0.15;     % Rear brake disc radius in  m
muPadBRA    = 0.33;     % Rear brake disc friction coefficient in -

%% Powertrain
% Motor
MotorType = 'Combustion';   % Motor type as Electric / Combustion
nMotor  = [1000 2150 4500 6500 7500]; % Engine LUT steps in rpm
TMotor  = [200  380  380  320  250 ]; % Engine LUT steps in Nm
nMinMotor       = 750;  % rpm
nMaxMotor       = 7500; % rpm
ratioEngBrake   = 0.02; % Max. engine brake torque / max. drive torque
expoThrottle    = 0.8;  % Throttle -> torque exponent

% Driveline
Driveline   = 'RWD-LSD';    % Driveline type as Hub-Motors / RWD-LSD
iGears      = [3.91 2.29 1.65 1.30 1.08 0.88 0.62]; % Gear ratios in -
iFinal      = 3.62;         % Final drive ratio in -
effGearbox  = 0.95;         % Gearbox efficiency in -
dGearbox    = 0.005;        % Gearbox drag in Nm/rpm_in
ratioLockDriveLSD = 0.25;   % Differential drive locking ratio in -
ratioLockCoastLSD = 0.50;   % Differential coast locking ratio in -
TPreloadLSD = 50;           % Differential preload torque in Nm

%% Front suspension
angToeFA    = 0.2; % Front toe in ° (positive = toe in)
angCamberFA =-0.8; % Front camber in ° (negative typical)
steeringWhlMaxAng = 450; % Maximum steering wheel angle in one direction in °
steerRackTrvlMax  = 0.0745; % Maximum steering rack travel in m
steerDeltaMax = 30; % Maximum steering angle at the wheels (linear kinematics only)

suspType_FA = 'Double-Wishbone'; % Front suspension type as linear / Double-Wishbone
% Double-Wishbone only:
% Left double-wishbone points defined from axle center in m
A_FA = [-0.251;  0.320; -0.080]; % A: lower arm @ chassis rear
B_FA = [ 0.148;  0.320; -0.094]; % B: lower arm @ chassis front
C_FA = [ 0.013;  0.737; -0.145]; % C: lower arm @ upright
D_FA = [-0.105;  0.435;  0.196]; % D: upper arm @ chassis rear
E_FA = [ 0.122;  0.435;  0.230]; % E: upper arm @ chassis front
F_FA = [-0.025;  0.680;  0.162]; % F: upper arm @ upright
R_FA = [-0.150;  0.380; -0.038]; % R: tie rod @ steering rack
Q_FA = [-0.137;  0.690; -0.088]; % Q: tie rod @ upright
maxRotLwrWBone_FA = 10; % Maximum lower wishbone rotation angle in °

%% Rear suspension
angToeRA    = 0.4; % Rear toe in ° (positive = toe in)
angCamberRA =-0.8; % Rear camber in ° (negative typical)

% Left suspension points defined from axle center
suspType_RA = 'linear'; % Rear suspension type as linear / Double-Wishbone

%% Upright mass properties
m_UFL = 20;    % kg
I_UxxFL = 0.5; % kg*m^2
I_UyyFL = 0.5; % kg*m^2
I_UzzFL = 0.5; % kg*m^2

m_UFR = 20;    % kg
I_UxxFR = 0.5; % kg*m^2
I_UyyFR = 0.5; % kg*m^2
I_UzzFR = 0.5; % kg*m^2

m_URL = 20;     % kg
I_UxxRL = 0.5; % kg*m^2
I_UyyRL = 0.5; % kg*m^2
I_UzzRL = 0.5; % kg*m^2

m_URR = 20;     % kg
I_UxxRR = 0.5; % kg*m^2
I_UyyRR = 0.5; % kg*m^2
I_UzzRR = 0.5; % kg*m^2

%% Wheel mass properties
m_WFL = 20;   % kg
I_WxxFL = 0.7;  % kg*m^2
I_WyyFL = 1.2;  % kg*m^2
I_WzzFL = 0.7;  % kg*m^2

m_WFR = 20;   % kg
I_WxxFR = 0.7;  % kg*m^2
I_WyyFR = 1.2;  % kg*m^2
I_WzzFR = 0.7;  % kg*m^2

m_WRL = 20;     % kg
I_WxxRL = 0.7;  % kg*m^2
I_WyyRL = 1.2;  % kg*m^2
I_WzzRL = 0.7;  % kg*m^2

m_WRR = 20;     % kg
I_WxxRR = 0.7;  % kg*m^2
I_WyyRR = 1.2;  % kg*m^2
I_WzzRR = 0.7;  % kg*m^2