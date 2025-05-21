function veh = f_vehicleParameters()
%%          Vehicle Parameters             %%
% ----------------------------------------- %
% Version: V2.0 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
% Output:                                   %
%   veh     -   struct containing vehicle   %
%               parameters                  %
% ----------------------------------------- %
veh = struct();

%% Vehicle selection UI
listing = dir('.\vehicles');
tbl = struct2table(listing);
fileNames = tbl.name(~tbl.isdir,:);
userSelIdx = listdlg('Name','Select a vehicle file (.m)', ...
    'SelectionMode','single', 'ListSize',[512 256],'ListString',fileNames);

if isempty(userSelIdx)
    error('User selected Cancel.');
end

try
    loadfile = fileNames{userSelIdx};
    loadpathfile = ['.' filesep 'vehicles' filesep loadfile];
    run(loadpathfile);
catch
    error('Invalid vehicle file selected.');
end

disp(['Vehicle file: ' loadfile]);

%% Store parameters in struct
% Tire choice
veh.tires = {tireFL tireFR tireRL tireRR};

% Mass
veh.m_FL = m_UFL + m_WFL;
veh.m_FR = m_UFR + m_WFR;
veh.m_RL = m_URL + m_WRL;
veh.m_RR = m_URR + m_WRR;
veh.m    = massVehicle;
veh.m_C  = veh.m-veh.m_FL-veh.m_FR-veh.m_RL-veh.m_RR;
veh.z_cg = heightCoG;

% Vehicle dimensions
veh.wb   = Wheelbase;
veh.l_FA = Wheelbase * (1-ratioMassFA);
veh.l_RA = Wheelbase - veh.l_FA;
veh.t_FA = trackFA;
veh.t_RA = trackRA;

% Springs & dampers
veh.c_FA        = cSpringFA;
veh.c_RA        = cSpringRA;
veh.c_HeaveFA   = cSpringHeaveFA;
veh.c_HeaveRA   = cSpringHeaveRA;
veh.c_RollFA    = cSpringRollFA;
veh.c_RollRA    = cSpringRollRA;

veh.v_DFA        = vDamperFA;
veh.F_DFA        = FDamperFA;
veh.v_DRA        = vDamperRA;
veh.F_DRA        = FDamperRA;
veh.v_DHeaveRA   = vDamperHeaveRA;
veh.F_DHeaveFA   = FDamperHeaveFA;
veh.v_DHeaveFA   = vDamperHeaveFA;
veh.F_DHeaveRA   = FDamperHeaveRA;
veh.v_DRollRA    = vDamperRollRA;
veh.F_DRollFA    = FDamperRollFA;
veh.v_DRollFA    = vDamperRollFA;
veh.F_DRollRA    = FDamperRollRA;

% Spring pretension
veh.F_SFL0 = 9.80665*veh.m_C*veh.l_RA/(2*(veh.l_FA+veh.l_RA));
veh.F_SFR0 = veh.F_SFL0;
veh.F_SRL0 = 9.80665*veh.m_C*veh.l_FA/(2*(veh.l_FA+veh.l_RA));
veh.F_SRR0 = veh.F_SRL0;

% Aerodynamics
veh.A_A   = projectArea;
veh.C_LFA = liftCoefFA;
veh.C_LRA = liftCoefRA;
veh.C_D   = dragCoef;

% Motor
switch MotorType
    case 'Electric'
        veh.Motor   = 2;
        veh.P_MMax  = PMotor;
        veh.T_MMax  = TMotor;
        veh.eta_M   = effMotor;
        veh.omega_Mmax = nMaxMotor.*(pi/30);

    case 'Combustion'
        veh.Motor   = 1;
        veh.omega_M = nMotor.*(pi/30);
        veh.omega_Mmin = nMinMotor.*(pi/30);
        veh.omega_Mmax = nMaxMotor.*(pi/30);
        veh.T_M     = TMotor;
        veh.C_MBrake= ratioEngBrake;
        veh.C_expTh = expoThrottle;

    otherwise
        error('Unknown Motor type selected.');
end

% Driveline
switch Driveline
    case 'Hub-Motors'
        veh.Drive   = 2;
        veh.i_G     = iGears;
        veh.eta_G   = effGearbox;
        veh.d_G     = dGearbox*30/pi;

    case 'RWD-LSD'
        veh.Drive   = 1;
        veh.i_G     = iGears;
        veh.n_G     = 1:length(iGears);
        veh.i_F     = iFinal;
        veh.eta_G   = effGearbox;
        veh.d_G     = dGearbox*30/pi;
        veh.C_LSD   = [ratioLockCoastLSD 0 ratioLockDriveLSD];
        veh.T_LPL   = TPreloadLSD;

    otherwise
        error('Unknown Driveline type selected.');
end

% Brakes
veh.i_p2TBFA = APstBFA*rmDiscBFA*muPadBFA*1E5;
veh.i_p2TBRA = APstBRA*rmDiscBRA*muPadBRA*1E5;

% I Chassis
veh.I_C = zeros(3);
veh.I_C(1,1) = IChassisXX;
veh.I_C(2,2) = IChassisYY;
veh.I_C(3,3) = IChassisZZ;

% I Upright FL
veh.I_UFL = zeros(3);
veh.I_UFL(1,1) = I_UxxFL;
veh.I_UFL(2,2) = I_UyyFL;
veh.I_UFL(3,3) = I_UzzFL;

% I Upright FR
veh.I_UFR = zeros(3);
veh.I_UFR(1,1) = I_UxxFR;
veh.I_UFR(2,2) = I_UyyFR;
veh.I_UFR(3,3) = I_UzzFR;

% I Upright RL
veh.I_URL = zeros(3);
veh.I_URL(1,1) = I_UxxRL;
veh.I_URL(2,2) = I_UyyRL;
veh.I_URL(3,3) = I_UzzRL;

% I Upright RR
veh.I_URR = zeros(3);
veh.I_URR(1,1) = I_UxxRR;
veh.I_URR(2,2) = I_UyyRR;
veh.I_URR(3,3) = I_UzzRR;

% I Wheel FL
veh.I_WFL = zeros(3);
veh.I_WFL(1,1) = I_WxxFL;
veh.I_WFL(2,2) = I_WyyFL;
veh.I_WFL(3,3) = I_WzzFL;

% I Wheel FR
veh.I_WFR = zeros(3);
veh.I_WFR(1,1) = I_WxxFR;
veh.I_WFR(2,2) = I_WyyFR;
veh.I_WFR(3,3) = I_WzzFR;

% I Wheel RL
veh.I_WRL = zeros(3);
veh.I_WRL(1,1) = I_WxxRL;
veh.I_WRL(2,2) = I_WyyRL;
veh.I_WRL(3,3) = I_WzzRL;

% I Wheel RR
veh.I_WRR = zeros(3);
veh.I_WRR(1,1) = I_WxxRR;
veh.I_WRR(2,2) = I_WyyRR;
veh.I_WRR(3,3) = I_WzzRR;

% Suspension
veh.i_S     = steerRackTrvlMax/(steeringWhlMaxAng*pi/180); % Steer ratio
veh.u_max   = steerRackTrvlMax;
veh.delta_max = steerDeltaMax*pi/180;
veh.susFL.delta_0 = -1*angToeFA*pi/180;
veh.susFL.gamma_0 = -1*angCamberFA*pi/180;
veh.susFR.delta_0 = angToeFA*pi/180;
veh.susFR.gamma_0 = angCamberFA*pi/180;
veh.susRL.delta_0 = -1*angToeRA*pi/180;
veh.susRL.gamma_0 = -1*angCamberRA*pi/180;
veh.susRR.delta_0 = angToeRA*pi/180;
veh.susRR.gamma_0 = angCamberRA*pi/180;

% Suspension front
switch suspType_FA
    case 'Double-Wishbone'
        veh.susType_FA  = 2;

        if (C_FA(1)+F_FA(1))/2 > Q_FA(1) % Tie rod behind kingpin axis
            veh.i_S = veh.i_S*-1;
        end

        veh.susFL.phi_max = maxRotLwrWBone_FA/180*pi;
        veh.susFL.r_VWk = [veh.l_FA; veh.t_FA/2; -veh.z_cg]; % W: wheel center
        veh.susFL.r_VAk = A_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VBk = B_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VCk = C_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VDk = D_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VEk = E_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VFk = F_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VRk = R_FA + [veh.l_FA; 0; -veh.z_cg];
        veh.susFL.r_VQk = Q_FA + [veh.l_FA; 0; -veh.z_cg];

        % right mirrored from left side
        veh.susFR.phi_max = veh.susFL.phi_max;
        veh.susFR.r_VWk = veh.susFL.r_VWk.*[1;-1;1];
        veh.susFR.r_VAk = veh.susFL.r_VAk.*[1;-1;1];
        veh.susFR.r_VBk = veh.susFL.r_VBk.*[1;-1;1];
        veh.susFR.r_VCk = veh.susFL.r_VCk.*[1;-1;1];
        veh.susFR.r_VDk = veh.susFL.r_VDk.*[1;-1;1];
        veh.susFR.r_VEk = veh.susFL.r_VEk.*[1;-1;1];
        veh.susFR.r_VFk = veh.susFL.r_VFk.*[1;-1;1];
        veh.susFR.r_VRk = veh.susFL.r_VRk.*[1;-1;1];
        veh.susFR.r_VQk = veh.susFL.r_VQk.*[1;-1;1];

    case 'linear'
        veh.susType_FA  = 1;
    otherwise
        error('Chosen front suspension type does not exit.');
end

% Suspension rear
switch suspType_RA
    case 'Double-Wishbone'
        veh.susType_RA  = 2;

        veh.susRL.phi_max = maxRotLwrWBone_RA/180*pi;
        veh.susRL.r_VWk = [-veh.l_RA; veh.t_RA/2; -veh.z_cg]; % W: wheel center
        veh.susRL.r_VAk = A_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VBk = B_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VCk = C_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VDk = D_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VEk = E_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VFk = F_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VRk = R_RA + [-veh.l_RA; 0; -veh.z_cg];
        veh.susRL.r_VQk = Q_RA + [-veh.l_RA; 0; -veh.z_cg];

        % right mirrored from left side
        veh.susRR.phi_max = veh.susRL.phi_max;
        veh.susRR.r_VWk = veh.susRL.r_VWk.*[1;-1;1];
        veh.susRR.r_VAk = veh.susRL.r_VAk.*[1;-1;1];
        veh.susRR.r_VBk = veh.susRL.r_VBk.*[1;-1;1];
        veh.susRR.r_VCk = veh.susRL.r_VCk.*[1;-1;1];
        veh.susRR.r_VDk = veh.susRL.r_VDk.*[1;-1;1];
        veh.susRR.r_VEk = veh.susRL.r_VEk.*[1;-1;1];
        veh.susRR.r_VFk = veh.susRL.r_VFk.*[1;-1;1];
        veh.susRR.r_VRk = veh.susRL.r_VRk.*[1;-1;1];
        veh.susRR.r_VQk = veh.susRL.r_VQk.*[1;-1;1];

    case 'linear'
        veh.susType_RA  = 1;
    otherwise
        error('Chosen rear suspension type does not exit.');
end


end