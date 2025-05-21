%%          Tire Model Validation          %%
% ----------------------------------------- %
% Version: V1.2 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: MATLAB AddOn MFeval must be         %
%       installed for use                   %
% ----------------------------------------- %
clc; clear; close all; warning('off','backtrace');

%% Set up
resolution = 256;   % -
v = 16.7;           % m/s
mu_n = 1.0;         % -

% Line colors
colorsTUM = [   0   101 189; ...
                162 173 0; ...
                227 114 34;  ...
                100 160 200; ...
                0   82  147; ...
                152 198 234;  ...
                0   51  89]./255;
% colorsMatlab = [0       0.447   0.741
%                 0.850   0.325   0.098
%                 0.929   0.694   0.125
%                 0.494   0.184   0.556
%                 0.466   0.674   0.188
%                 0.301   0.745   0.933
%                 0.635   0.078   0.184];
% fiftyShadesOfBlue = [1 0.67 0.33 0.75 0.5 0.25]'*[0 0.6 0.9];
% fiftyShadesOfGrey = [0 0.6 0.2 0.4 0.8]'*ones(1,3);

colors = colorsTUM;

%% Load TMeasy file
listing = dir('.\tires');
tbl = struct2table(listing);
fileNames = tbl.name(~tbl.isdir,:);
userSelIdx = listdlg('Name','Select a tire file (.m)', ...
    'SelectionMode','single', 'ListSize',[512 256],'ListString',fileNames);

if isempty(userSelIdx)
    error('User selected Cancel.');
end

try
    loadfile = fileNames{userSelIdx};
    loadpathfile = ['.' filesep 'tires' filesep loadfile];
    run(loadpathfile);
catch
    error('Invalid tire file selected.');
end

disp(['Tire file: ' loadfile]);

%% Load tir file
[loadfile,loadpath] = uigetfile('*.tir','Open tire parameter file (.tir). Press CANCEL to skip.');
if isequal(loadfile,0)
    useMFTire = false;
else
    useMFTire = true;
    TIRpathfile = fullfile(loadpath,loadfile);
    useMode = 121;
    MFresolution = 19;
end

%% Load Sensitivity
F_z = [1 2].*tir.F_zN;  % N
IA  = 0;                % °

f1 = figure('Name','Load Sensitivity','Units','normalized','OuterPosition',[0 0 0.5 1]); hold on;

% FX
v_0QQ = [v; 0; 0];
omega_yW = linspace(0.67*v_0QQ(1),1.67*v_0QQ(1),resolution)'./tir.r_0;

opts.use3DRoad = false; road = struct.empty;
gamma=zeros(size(F_z));r_st=gamma;dz=gamma;w=gamma;y_Q=gamma;
for i = 1:length(F_z)
    A_0U = f_xrot(deg2rad(IA));
    r_0M0 = A_0U*[0;0;tir.r_0-F_z(i)/tir.c_z];
    e_yR  = A_0U*[0;1;0];
    [~,~,gamma(i), r_st(i), dz(i), w(i), r_MQ0] = f_roadContact(road,tir,r_0M0,[0;0;0],[0;0;0],e_yR,opts);
    y_Q(i) = r_MQ0(2);
end
r_dyn = tir.r_0.*2./3 + r_st./3;

s_x = zeros(resolution,length(dz));
F_x = zeros(resolution,length(dz));
T_y = F_x;
for i = 1:length(F_z)
    for ii = 1:resolution
        [F_T, T_T, s] = f_tireTMeasy(tir, 0, v_0QQ, [0; omega_yW(ii); 0],  r_st(i), dz(i), w(i), gamma(i), mu_n,[0;0;0],false);
        s_x(ii,i) = s(1);
        F_x(ii,i) = F_T(1);
        T_y(ii,i) = T_T(2);
    end
end

% FY
SA = linspace(-25,25,resolution);
v_0QQ = [v.*cosd(SA); v.*sind(SA); zeros(1,length(SA))];
omega_yW = [v_0QQ(1,:)'./r_dyn(1) v_0QQ(1,:)'./r_dyn(2)];
s_y = zeros(resolution,length(dz));
F_y = zeros(resolution,length(dz));
T_x = zeros(resolution,length(dz));
T_z = zeros(resolution,length(dz));
for i = 1:length(F_z)
    for ii = 1:resolution
        [F_T, T_T, s] = f_tireTMeasy(tir, 0, v_0QQ(:,ii), [0; omega_yW(ii,i); 0], r_st(i), dz(i), w(i), gamma(i), mu_n,[0;0;0],false);
        s_y(ii,i) = s(2);
        F_y(ii,i) = F_T(2);
        T_x(ii,i) = T_T(1)+F_T(1)*y_Q(i);
        T_z(ii,i) = T_T(3);
    end
end

% MF-Tire
if useMFTire
    % Virtual test bench signals
    FZ1     = ones(MFresolution,1).*F_z(1);           % vertical load 1 in kN
    FZ2     = ones(MFresolution,1).*F_z(2);           % vertical load 2 in kN
    KAPPA	= linspace(-0.15,0.15, MFresolution)';    % longitudinal slip in -
    ALPHA	= linspace(-0.20,0.20, MFresolution)';    % side slip angle in rad
    GAMMA	= ones(MFresolution,1).*deg2rad(IA);      % inclination angle in rad
    PHIT 	= ones(MFresolution,1).*0;                % turnslip in 1/m
    VX   	= ones(MFresolution,1).*v;                % forward velocity in m/s

    % Cases: Kappa and Alpha sweeps at 1*FZN and 2*FZN
    inputsLON1 = [FZ1   KAPPA 0*ALPHA GAMMA PHIT VX];
    inputsLAT1 = [FZ1 0*KAPPA   ALPHA GAMMA PHIT VX];
    inputsLON2 = [FZ2   KAPPA 0*ALPHA GAMMA PHIT VX];
    inputsLAT2 = [FZ2 0*KAPPA   ALPHA GAMMA PHIT VX];

    % Run MF Model
    LON1mat = mfeval(TIRpathfile, inputsLON1, useMode);
    LAT1mat = mfeval(TIRpathfile, inputsLAT1, useMode);
    LON2mat = mfeval(TIRpathfile, inputsLON2, useMode);
    LAT2mat = mfeval(TIRpathfile, inputsLAT2, useMode);

    % Create output structs
    LON1.FX     = LON1mat(:,1);
    LON1.FY     = LON1mat(:,2);
    LON1.FZ     = LON1mat(:,3);
    LON1.MX     = LON1mat(:,4);
    LON1.MY     = LON1mat(:,5);
    LON1.MZ     = LON1mat(:,6);
    LON1.KAPPA  = LON1mat(:,7);
    LON1.ALPHA  = LON1mat(:,8);

    LAT1.FX     = LAT1mat(:,1);
    LAT1.FY     = LAT1mat(:,2);
    LAT1.FZ     = LAT1mat(:,3);
    LAT1.MX     = LAT1mat(:,4);
    LAT1.MY     = LAT1mat(:,5);
    LAT1.MZ     = LAT1mat(:,6);
    LAT1.KAPPA  = LAT1mat(:,7);
    LAT1.ALPHA  = LAT1mat(:,8);

    LON2.FX     = LON2mat(:,1);
    LON2.FY     = LON2mat(:,2);
    LON2.FZ     = LON2mat(:,3);
    LON2.MX     = LON2mat(:,4);
    LON2.MY     = LON2mat(:,5);
    LON2.MZ     = LON2mat(:,6);
    LON2.KAPPA  = LON2mat(:,7);
    LON2.ALPHA  = LON2mat(:,8);

    LAT2.FX     = LAT2mat(:,1);
    LAT2.FY     = LAT2mat(:,2);
    LAT2.FZ     = LAT2mat(:,3);
    LAT2.MX     = LAT2mat(:,4);
    LAT2.MY     = LAT2mat(:,5);
    LAT2.MZ     = LAT2mat(:,6);
    LAT2.KAPPA  = LAT2mat(:,7);
    LAT2.ALPHA  = LAT2mat(:,8);
end

% F_x plot
ax1 = subplot(131); hold on;
plot(s_x(:,1).*100,F_x(:,1).*1E-3,'Color',colors(1,:));
plot(s_x(F_x(:,1)>=0,1).*-100,-1.*F_x(F_x(:,1)>=0,1).*1E-3,'--','Color',colors(1,:));
plot(s_x(:,2).*100,F_x(:,2).*1E-3,'Color',colors(2,:));
plot(s_x(F_x(:,2)>=0,2).*-100,-1.*F_x(F_x(:,2)>=0,2).*1E-3,'--','Color',colors(2,:));
title('Longitudinal Force');
xlabel('s_x in %');
ylabel('F_x in kN');
grid on;
xlim([-30 30]);

if useMFTire
    plot(LON1.KAPPA.*100,LON1.FX.*1E-3,'o','Color',colors(1,:));
    plot(LON2.KAPPA.*100,LON2.FX.*1E-3,'o','Color',colors(2,:));

    legend('TMeasy','TMeasy (sym.)','','','MF-Tire','Location','northwest');
else
    legend('TMeasy','TMeasy (sym.)','Location','northwest');
end

% F_y plot
ax2 = subplot(132); hold on;
plot(atand(s_y(:,1)),F_y(:,1).*1E-3,'Color',colors(1,:));
plot(atand(s_y(:,2)),F_y(:,2).*1E-3,'Color',colors(2,:));
title('Lateral Force');
xlabel('\alpha in °');
ylabel('F_y in kN');
grid on;

if useMFTire
    plot(rad2deg(-1.*LAT1.ALPHA),LAT1.FY.*1E-3,'o','Color',colors(1,:));
    plot(rad2deg(-1.*LAT2.ALPHA),LAT2.FY.*1E-3,'o','Color',colors(2,:));
end

legend(['F_z = ' num2str(F_z(1)) 'N, \gamma = ' num2str(IA) '°'],['F_z = ' num2str(F_z(2)) 'N, \gamma = ' num2str(IA) '°'],'Location','northwest');

% T_z plot
ax3 = subplot(133); hold on;
plot(atand(s_y(:,1)),T_z(:,1),'Color',colors(1,:));
plot(atand(s_y(:,2)),T_z(:,2),'Color',colors(2,:));
title('Aligning Torque');
xlabel('\alpha in °');
ylabel('T_z in Nm');
grid on;

if useMFTire
    plot(rad2deg(-1.*LAT1.ALPHA),LAT1.MZ,'o','Color',colors(1,:));
    plot(rad2deg(-1.*LAT2.ALPHA),LAT2.MZ,'o','Color',colors(2,:));
end

% Link plot axes
linkaxes([ax1 ax2],'y');
linkaxes([ax2 ax3],'x');
xlim([-20 20]);

%% Overturning Torque
figure('WindowState','minimized'); hold on;
plot(atand(s_y(:,1)),T_x(:,1),'Color',colors(1,:));
plot(atand(s_y(:,2)),T_x(:,2),'Color',colors(2,:));

if useMFTire
    plot(rad2deg(-1.*LAT1.ALPHA),LAT1.MX,'o','Color',colors(1,:));
    plot(rad2deg(-1.*LAT2.ALPHA),LAT2.MX,'o','Color',colors(2,:));
end

legend(['F_z = ' num2str(F_z(1)) 'N, \gamma = ' num2str(IA) '°'],['F_z = ' num2str(F_z(2)) 'N, \gamma = ' num2str(IA) '°'],'Location','northwest');
title('Overturning Torque');
xlabel('\alpha in °');
ylabel('T_x in Nm');
grid on;

%% Camber Sensitivity
F_z = 1*tir.F_zN;   % N
IA  = [0 4];        % °

f2 = figure('Name','Camber Sensitivity','Units','normalized','OuterPosition',[0.5 0 0.5 1]); hold on;

% FX
v_0QQ = [v; 0; 0];
omega_yW = linspace(0.67*v_0QQ(1),1.67*v_0QQ(1),resolution)'./tir.r_0;

gamma=zeros(size(IA)); r_st=gamma;dz=gamma;w=gamma;
for i = 1:length(IA)
    A_0U = f_xrot(deg2rad(IA(i)));
    r_0M0 = A_0U*[0;0;tir.r_0-F_z/tir.c_z];
    e_yR  = A_0U*[0;1;0];
    [~,~,gamma(i), r_st(i), dz(i), w(i), ~] = f_roadContact(road,tir, r_0M0, [0;0;0], [0;0;0], e_yR,opts);
end
r_dyn = tir.r_0.*2./3 + r_st./3;

s_x = zeros(resolution,length(IA));
F_x = zeros(resolution,length(IA));

for i = 1:length(IA)
    for ii = 1:resolution
        [F_T, T_T, s] = f_tireTMeasy(tir, 0, v_0QQ, [0; omega_yW(ii); 0], r_st(i), dz(i), w(i), gamma(i), mu_n,[0;0;0],false);
        s_x(ii,i) = s(1);
        F_x(ii,i) = F_T(1);
    end
end

% FY
SA = linspace(-25,25,resolution);
v_0QQ = [v.*cosd(SA); v.*sind(SA); zeros(1,length(SA))];
omega_yW = v_0QQ(1,:)'./r_dyn;

gamma=zeros(size(IA)); r_st=gamma;dz=gamma;w=gamma;
for i = 1:length(IA)
    A_0U = f_xrot(deg2rad(IA(i)));
    r_0M0 = A_0U*[0;0;tir.r_0-F_z/tir.c_z];
    e_yR  = A_0U*[0;1;0];
    [~,~,gamma(i), r_st(i), dz(i), w(i), ~] = f_roadContact(road,tir,r_0M0,[0;0;0],[0;0;0],e_yR,opts);
end
r_dyn = tir.r_0.*2./3 + r_st./3;

s_y = zeros(resolution,length(IA)); F_y = s_y; T_x = s_y; T_z = s_y;

for i = 1:length(IA)
    for ii = 1:resolution
        [F_T, T_T, s] = f_tireTMeasy(tir, 0, v_0QQ(:,ii), [0; omega_yW(ii); 0], r_st(i), dz(i), w(i), gamma(i), mu_n,[0;0;0],false);
        s_y(ii,i) = s(2);
        F_y(ii,i) = F_T(2);
        T_x(ii,i) = T_T(1);
        T_z(ii,i) = T_T(3);
    end
end

% MF-Tire
if useMFTire
    % Virtual test bench signals
    FZ      = ones(MFresolution,1).*F_z;              % vertical load in kN
    KAPPA	= linspace(-0.15,0.15, MFresolution)';    % longitudinal slip in -
    ALPHA	= linspace(-0.20,0.20, MFresolution)';    % side slip angle in rad
    GAMMA1	= ones(MFresolution,1).*deg2rad(IA(1));   % inclination angle in rad
    GAMMA2	= ones(MFresolution,1).*deg2rad(IA(2));   % inclination angle in rad
    PHIT 	= ones(MFresolution,1).*0;                % turnslip in 1/m
    VX   	= ones(MFresolution,1).*v;                % forward velocity in m/s

    % Cases: Kappa and Alpha sweeps at 1*FZN and 2*FZN
    inputsLON1 = [FZ   KAPPA 0*ALPHA GAMMA1 PHIT VX];
    inputsLAT1 = [FZ 0*KAPPA   ALPHA GAMMA1 PHIT VX];
    inputsLON2 = [FZ   KAPPA 0*ALPHA GAMMA2 PHIT VX];
    inputsLAT2 = [FZ 0*KAPPA   ALPHA GAMMA2 PHIT VX];

    % Run MF Model
    LON1mat = mfeval(TIRpathfile, inputsLON1, useMode);
    LAT1mat = mfeval(TIRpathfile, inputsLAT1, useMode);
    LON2mat = mfeval(TIRpathfile, inputsLON2, useMode);
    LAT2mat = mfeval(TIRpathfile, inputsLAT2, useMode);

    % Create output structs
    LON1.FX     = LON1mat(:,1);
    LON1.FY     = LON1mat(:,2);
    LON1.FZ     = LON1mat(:,3);
    LON1.MX     = LON1mat(:,4);
    LON1.MY     = LON1mat(:,5);
    LON1.MZ     = LON1mat(:,6);
    LON1.KAPPA  = LON1mat(:,7);
    LON1.ALPHA  = LON1mat(:,8);

    LAT1.FX     = LAT1mat(:,1);
    LAT1.FY     = LAT1mat(:,2);
    LAT1.FZ     = LAT1mat(:,3);
    LAT1.MX     = LAT1mat(:,4);
    LAT1.MY     = LAT1mat(:,5);
    LAT1.MZ     = LAT1mat(:,6);
    LAT1.KAPPA  = LAT1mat(:,7);
    LAT1.ALPHA  = LAT1mat(:,8);

    LON2.FX     = LON2mat(:,1);
    LON2.FY     = LON2mat(:,2);
    LON2.FZ     = LON2mat(:,3);
    LON2.MX     = LON2mat(:,4);
    LON2.MY     = LON2mat(:,5);
    LON2.MZ     = LON2mat(:,6);
    LON2.KAPPA  = LON2mat(:,7);
    LON2.ALPHA  = LON2mat(:,8);

    LAT2.FX     = LAT2mat(:,1);
    LAT2.FY     = LAT2mat(:,2);
    LAT2.FZ     = LAT2mat(:,3);
    LAT2.MX     = LAT2mat(:,4);
    LAT2.MY     = LAT2mat(:,5);
    LAT2.MZ     = LAT2mat(:,6);
    LAT2.KAPPA  = LAT2mat(:,7);
    LAT2.ALPHA  = LAT2mat(:,8);
end

% F_x plot
ax1 = subplot(131); hold on;
for i = 1:length(IA)
    plot(s_x(:,i).*100,F_x(:,i).*1E-3,'Color',colors(i,:));
end
title('Longitudinal Force');
xlabel('s_x in %');
ylabel('F_x in kN');
grid on;
xlim([-30 30]);

if useMFTire
    plot(LON1.KAPPA.*100,LON1.FX.*1E-3,'o','Color',colors(1,:));
    plot(LON2.KAPPA.*100,LON2.FX.*1E-3,'o','Color',colors(2,:));

    legend('TMeasy','','MF-Tire','Location','northwest');
end

% F_y plot
ax2 = subplot(132); hold on;
for i = 1:length(IA)
    plot(atand(s_y(:,i)),F_y(:,i).*1E-3,'Color',colors(i,:));
end
title('Lateral Force');
xlabel('\alpha in °');
ylabel('F_y in kN');
grid on;

if useMFTire
    plot(rad2deg(-1.*LAT1.ALPHA),LAT1.FY.*1E-3,'o','Color',colors(1,:));
    plot(rad2deg(-1.*LAT2.ALPHA),LAT2.FY.*1E-3,'o','Color',colors(2,:));
end

leg = cell(size(IA));
for i = 1:length(IA)
    leg{i} = ['F_z = ' num2str(F_z) 'N, \gamma = ' num2str(IA(i)) '°'];
end
legend(leg,'Location','northwest');

% T_z plot
ax3 = subplot(133); hold on;
for i = 1:length(IA)
    plot(atand(s_y(:,i)),T_z(:,i),'Color',colors(i,:));
end
title('Aligning Torque');
xlabel('\alpha in °');
ylabel('T_z in Nm');
grid on;

if useMFTire
    plot(rad2deg(-1.*LAT1.ALPHA),LAT1.MZ,'o','Color',colors(1,:));
    plot(rad2deg(-1.*LAT2.ALPHA),LAT2.MZ,'o','Color',colors(2,:));
end

% Link plot axes
linkaxes([ax1 ax2],'y');
linkaxes([ax2 ax3],'x');
xlim([-20 20]);

%% Parking Torque
% figure('Name','Parking Torque');
% 
% delta = pi/6*sin(0:.1:2*pi);
% om_z = [0 diff(delta)./1E-2];
% MZ = zeros(size(om_z));
% for i = 1:length(om_z)
%     [F_T, T_T, s] = f_tireTMeasy(tir, 0, [0;0;0], [0;0;om_z(i)], r_st(1), dz(1), w(1), y_Q(1), gamma(1), mu_n,[0;0;0],false);
%     MZ(i) = T_T(3);
% end
% 
% subplot(211); hold on; grid on;
% plot(linspace(0,4,length(delta)),rad2deg(delta));
% title('Wheel Angle');
% xlabel('t in s');
% ylabel('\delta_W in °');
% 
% subplot(212); hold on; grid on;
% plot(rad2deg(delta),MZ);
% title('Bore Torque');
% xlabel('\delta_W in °');
% ylabel('T_z in kNm');

%% TEST Camber Model
% figure('Name','Camber Test');
% 
% subplot(121); hold on; grid on;
% title('Lateral Force');
% xlabel('\alpha in °');
% ylabel('F_y in kN');
% grid on;
% 
% subplot(122); hold on; grid on;
% title('Aligning Torque');
% xlabel('\alpha in °');
% ylabel('T_z in kNm');
% grid on;
% 
% SA = linspace(-25,25,resolution);
% v_0QQ = [v.*cosd(SA); v.*sind(SA); zeros(1,length(SA))];
% omega_yW = v_0QQ(1,:)'./r_dyn;
% 
% IAtest = 0:2:10;
% leg = cell(size(IAtest));
% SA = zeros(resolution,1);
% FY = SA; MZ = SA;
% 
% gamma=zeros(size(IAtest)); r_st=gamma;dz=gamma;w=gamma;
% for i = 1:length(IAtest)
%     A_0U = f_xrot(deg2rad(IAtest(i)));
%     r_0M0 = A_0U*[0;0;tir.r_0-F_z/tir.c_z];
%     e_yR  = A_0U*[0;1;0];
%     [~,~,gamma(i), r_st(i), dz(i), w(i)] = f_roadContact(road,tir,r_0M0,[0;0;0],[0;0;0],e_yR,opts);
% end
% r_dyn = tir.r_0.*2./3 + r_st./3;
% 
% for i = 1:length(IAtest)
%     for ii = 1:resolution
%         [F_T, T_T, s] = f_tireTMeasy(tir, 0, v_0QQ(:,ii), [0; omega_yW(ii,1); 0], r_st(i), dz(i), w(i), gamma(i), mu_n,[0;0;0],false);
%         SA(ii) = atand(s(2));
%         FY(ii) = F_T(2);
%         MZ(ii) = T_T(3);
%     end
%     subplot(121);
%     plot(SA,FY.*1E-3);
%     subplot(122);
%     plot(SA,MZ);
%     leg{i} = num2str(IAtest(i));
% end
% legend(leg);
