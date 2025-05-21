%%  Get TMeasy Parameters from TIR file    %%
% ----------------------------------------- %
% Version: V1.1 - 2024.03.13                %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: MATLAB AddOn MFeval must be         %
%       installed for use                   %
% ----------------------------------------- %
clc; close all; clear;

%% Load tir file
[loadfile,loadpath] = uigetfile('*.tir','Open tire parameter file (.tir)');
if isequal(loadfile,0)
    error('User selected Cancel');
end
TIRpathfile = fullfile(loadpath,loadfile);

%% Get MF Model Data
resolution  = 255; % Number of data points
useMode     = 111; % Select a MF Use Mode

% Get tire parameters
params = mfeval.readTIR(TIRpathfile);

% Virtual test bench signals
FZ      = ones(resolution,1).*params.FNOMIN;    % vertical load in N
KAPPA	= linspace(-0.20,0.20, resolution)';    % longitudinal slip in -
ALPHA	= linspace(-0.30,0.30, resolution)';    % side slip angle in rad
GAMMA	= ones(resolution,1).*0;                % inclination angle in rad
PHIT 	= ones(resolution,1).*0;                % turn slip in 1/m
VX   	= ones(resolution,1).*params.LONGVL;    % forward velocity in m/s

% Cases: Kappa and Alpha sweeps at 1*FZN and 2*FZN
inputsLON1 = [1*FZ   KAPPA 0*ALPHA GAMMA PHIT VX];
inputsLAT1 = [1*FZ 0*KAPPA   ALPHA GAMMA PHIT VX];
inputsLON2 = [2*FZ   KAPPA 0*ALPHA GAMMA PHIT VX];
inputsLAT2 = [2*FZ 0*KAPPA   ALPHA GAMMA PHIT VX];

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

%% TMeasy Parameters
clc;
disp(['File: ' loadfile]);
disp('---------------');
disp(['F_zN = ' num2str(params.FNOMIN)]);
disp('---------------');

disp(['dF_x01 = ' num2str(round(max(abs( diff(LON1.FX)./diff(LON1.KAPPA) )) ,-3))]);
disp(['s_xMA1 = ' num2str(round( LON1.KAPPA(LON1.FX==max(LON1.FX)) ,3))]);
disp(['F_xMA1 = ' num2str(round(max( LON1.FX ) ,-1))]);
disp(['s_xMB1 = ' num2str(round( -1*LON1.KAPPA(LON1.FX==min(LON1.FX)) ,3))]);
disp(['F_xMB1 = ' num2str(round(max( -1.*LON1.FX ) ,-1))]);
disp('---------------');

disp(['dF_x02 = ' num2str(round(max(abs( diff(LON2.FX)./diff(LON2.KAPPA) )) ,-3))]);
disp(['s_xMA2 = ' num2str(round( LON2.KAPPA(LON2.FX==max(LON2.FX)) ,3))]);
disp(['F_xMA2 = ' num2str(round(max( LON2.FX ) ,-1))]);
disp(['s_xMB2 = ' num2str(round( -1*LON2.KAPPA(LON2.FX==min(LON2.FX)) ,3))]);
disp(['F_xMB2 = ' num2str(round(max( -1.*LON2.FX ) ,-1))]);
disp('---------------');

disp(['dF_y01 = ' num2str(round(max(abs( diff(LAT1.FY)./diff(tan(LAT1.ALPHA)) )) ,-3))]);
disp(['s_yM1  = ' num2str(round(abs( tan(LAT1.ALPHA(LAT1.FY==min(LAT1.FY))) ) ,3))]);
disp(['F_yM1  = ' num2str(round(abs(min( LAT1.FY )) ,-1))]);
disp('---------------');

disp(['dF_y02 = ' num2str(round(max(abs( diff(LAT2.FY)./diff(tan(LAT2.ALPHA)) )) ,-3))]);
disp(['s_yM2  = ' num2str(round(abs( tan(LAT2.ALPHA(LAT2.FY==min(LAT2.FY))) ) ,3))]);
disp(['F_yM2  = ' num2str(round(abs(min( LAT2.FY )) ,-1))]);

%% Plots
close all;
figure;

% FX
ax1 = subplot(131); hold on;
plot(LON1.KAPPA.*100,LON1.FX.*1E-3);
plot(LON2.KAPPA.*100,LON2.FX.*1E-3);
title('Longitudinal Force');
xlabel('\kappa in %');
ylabel('F_x in kN');
grid on;


% FY
ax2 = subplot(132); hold on;
plot(rad2deg(LAT1.ALPHA),LAT1.FY.*1E-3);
plot(rad2deg(LAT2.ALPHA),LAT2.FY.*1E-3);
title('Lateral Force');
xlabel('\alpha in °');
ylabel('F_y in kN');
grid on;

% MZ
ax3 = subplot(133); hold on;
plot(rad2deg(LAT1.ALPHA),LAT1.MZ);
plot(rad2deg(LAT2.ALPHA),LAT2.MZ);
title('Aligning Torque');
xlabel('\alpha in °');
ylabel('M_z in Nm');
grid on;

% Link plot axes
linkaxes([ax1 ax2],'y'); linkaxes([ax2 ax3],'x'); xlim([-15 15]);
