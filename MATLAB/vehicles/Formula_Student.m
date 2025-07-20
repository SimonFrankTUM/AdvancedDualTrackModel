%% Vehicle Parameters: Formula Student     %%
% ----------------------------------------- %
% Version: V1.1 - 2024.12.26                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Generic FS template                 %
% ----------------------------------------- %

%% Tire selection
tireFL  = 'Formula_Student';
tireFR  = 'Formula_Student';
tireRL  = 'Formula_Student';
tireRR  = 'Formula_Student';

%% Chassis
massVehicle = 180+80; % Whole car weight in kg
Wheelbase   = 1.60; % Distance between axles in m
trackFA     = 1.24; % Distance between front wheel center points in m
trackRA     = 1.15; % Distance between rear wheel center points in m
ratioMassFA = 0.48; % Ratio of mass on front axle in -
heightCoG   = 0.10; % CoG height above wheel center plane in m

IChassisXX  = 100;  % Chassis inertia around X-axis in kg*m^2
IChassisYY  = 200;  % Chassis inertia around Y-axis kg*m^2
IChassisZZ  = 200;  % Chassis inertia around Z-axis kg*m^2

%% Springs & dampers
cSpringFA       = 158E3/1.08^2; % Front spring rate (reduced by motion ratio^2) in N/m
cSpringRA       = 158E3/1.00^2; % Rear spring rate (reduced by motion ratio^2) in N/m

cSpringHeaveFA  = 0; % Front heave spring rate (reduced by motion ratio^2) in N/m
cSpringHeaveRA  = 0; % Rear heave spring rate (reduced by motion ratio^2) in N/m

cSpringRollFA   = 0; % Front roll spring/ARB rate (reduced by motion ratio^2) in N/m
cSpringRollRA   = 0; % Rear roll spring/ARB rate (reduced by motion ratio^2) in N/m

vDamperFA       = [-0.1 -0.05 0 0.05 0.1].*1.08;% Front damper speeds (increased by motion ratio) LUT in m/s
FDamperFA       = [-300 -200 0 200 300]./1.08;  % Front damper forces (reduced by motion ratio) LUT in N
vDamperRA       = [-0.1 -0.05 0 0.05 0.1].*1.00;% Rear damper speeds (increased by motion ratio) LUT in m/s
FDamperRA       = [-300 -200 0 200 300]./1.00;  % Rear damper forces (reduced by motion ratio) LUT in N

vDamperHeaveFA  = 0; % Front heave damper speeds (increased by motion ratio) LUT in m/s
FDamperHeaveFA  = 0; % Front heave damper forces (reduced by motion ratio) LUT in N
vDamperHeaveRA  = 0; % Rear heave damper speeds (increased by motion ratio) LUT in m/s
FDamperHeaveRA  = 0; % Rear heave damper forces (reduced by motion ratio) LUT in N

vDamperRollFA   = 0; % Front roll damper speeds (increased by motion ratio) LUT in m/s
FDamperRollFA   = 0; % Front roll damper forces (reduced by motion ratio) LUT in N
vDamperRollRA   = 0; % Rear roll damper speeds (increased by motion ratio) LUT in m/s
FDamperRollRA   = 0; % Rear roll damper forces (reduced by motion ratio) LUT in N

%% Aerodynamics
projectArea =  1.2; % Frontal area in m^2
dragCoef    = -1.3; % Drag coefficient (must be negative) in -
liftCoefFA  = -1.8; % Front axle lift coefficient (negative for downforce) in -
liftCoefRA  = -1.2; % Rear axle lift coefficient (negative for downforce) in -

%% Brakes
APstBFA     = 0.0016;   % Brake piston area per front wheel in m^2
rmDiscBFA   = 0.094;    % Front brake disc radius in m
muPadBFA    = 0.35;     % Front brake disc friction coefficient in -

APstBRA     = 0.001;    % Brake piston area per rear wheel in m^2
rmDiscBRA   = 0.090;    % Rear brake disc radius in  m
muPadBRA    = 0.35;     % Rear brake disc friction coefficient in -

%% Powertrain
% Motor
MotorType   = 'Electric';   % Motor type as Electric / Combustion
PMotor      = 35E3;         % Electric motor power in W
TMotor      = 21;           % Electric motor torque in Nm
effMotor    = 0.90;         % Electric motor efficiency in -
nMaxMotor   = 20E3;         % Maximum electric motor speed in rpm

% Driveline
Driveline   = 'Hub-Motors'; % Driveline type as Hub-Motors / RWD-LSD
iGears      = 15.13;        % Gear ratio in -
effGearbox  = 0.95;         % Gearbox efficiency in -
dGearbox    = 1E-4;         % Gearbox drag in Nm/rpm_in

%% Front suspension
angToeFA    =-0.2; % Front toe in ° (positive = toe in)
angCamberFA =-1.0; % Front camber in ° (negative typical)
steeringWhlMaxAng = 90;  % Maximum steering wheel angle in one direction in °
steerRackTrvlMax  = 0.04; % Maximum steering rack travel in m
steerDeltaMax = 30; % Maximum steering angle at the wheels (linear kinematics only)

suspType_FA = 'Double-Wishbone'; % Front suspension type as linear / Double-Wishbone
% Double-Wishbone only:
% Left double-wishbone points defined from axle center in m
A_FA = [-0.136;	0.187; -0.084]; % A: lower arm @ chassis rear
B_FA = [ 0.305;	0.028; -0.097]; % B: lower arm @ chassis front
C_FA = [ 0.000; 0.554; -0.088]; % C: lower arm @ upright
D_FA = [-0.262; 0.240;  0.020]; % D: upper arm @ chassis rear
E_FA = [ 0.158; 0.236;  0.069]; % E: upper arm @ chassis front
F_FA = [-0.008; 0.544;  0.086]; % F: upper arm @ upright
R_FA = [ 0.060; 0.192; -0.010]; % R: tie rod @ steering rack
Q_FA = [ 0.075; 0.555;  0.015]; % Q: tie rod @ upright
maxRotLwrWBone_FA = 7; % Maximum lower wishbone rotation angle in °

%% Rear suspension
angToeRA    = 0.5; % Rear toe in ° (positive = toe in)
angCamberRA =-1.0; % Rear camber in ° (negative typical)

suspType_RA = 'Double-Wishbone'; % linear / Double-Wishbone
% Double-Wishbone only:
% Left double-wishbone points defined from axle center in m
A_RA = [-0.032; 0.176; -0.101]; % A: lower arm @ chassis rear
B_RA = [ 0.160; 0.176; -0.089]; % B: lower arm @ chassis front
C_RA = [-0.028; 0.507; -0.083]; % C: lower arm @ upright
D_RA = [-0.150; 0.180;  0.012]; % D: upper arm @ chassis rear
E_RA = [ 0.160; 0.180;  0.002]; % E: upper arm @ chassis front
F_RA = [-0.036; 0.452;  0.067]; % F: upper arm @ upright
R_RA = [-0.109; 0.178; -0.030]; % R: tie rod @ chassis
Q_RA = [-0.088; 0.515;  0.014]; % Q: tie rod @ upright
maxRotLwrWBone_RA = 8; % Maximum lower wishbone rotation angle in °

%% Upright mass properties
m_UFL = 3;      % kg
I_UxxFL = 0.05; % kg*m^2
I_UyyFL = 0.05; % kg*m^2
I_UzzFL = 0.05; % kg*m^2

m_UFR = 3;      % kg
I_UxxFR = 0.05; % kg*m^2
I_UyyFR = 0.05; % kg*m^2
I_UzzFR = 0.05; % kg*m^2

m_URL = 3;      % kg
I_UxxRL = 0.05; % kg*m^2
I_UyyRL = 0.05; % kg*m^2
I_UzzRL = 0.05; % kg*m^2

m_URR = 3;      % kg
I_UxxRR = 0.05; % kg*m^2
I_UyyRR = 0.05; % kg*m^2
I_UzzRR = 0.05; % kg*m^2

%% Wheel mass properties
m_WFL = 4;      % kg
I_WxxFL = 0.05; % kg*m^2
I_WyyFL = 0.26; % kg*m^2
I_WzzFL = 0.05; % kg*m^2

m_WFR = 4;      % kg
I_WxxFR = 0.05; % kg*m^2
I_WyyFR = 0.26; % kg*m^2
I_WzzFR = 0.05; % kg*m^2

m_WRL = 4;      % kg
I_WxxRL = 0.05; % kg*m^2
I_WyyRL = 0.26; % kg*m^2
I_WzzRL = 0.05; % kg*m^2

m_WRR = 4;      % kg
I_WxxRR = 0.05; % kg*m^2
I_WyyRR = 0.26; % kg*m^2
I_WzzRR = 0.05; % kg*m^2