%%  Get TMeasy Parameters from tire data   %%
% ----------------------------------------- %
% Version: V1.0 - 2025.05.30                %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Info: Select any file containing FX or FY %
%       tire force sweeps with the          %
%       following fields:                   %
%       FX - lon. force                     %
%       FY - lat. force                     %
%       FZ - normal force                   %
%       IA - inclination angle              %    
%       MX - tipping torque                 %
%       MZ - aligning torque                %
%       P  - pressure                       %
%       SA - slip angle                     %
%       SL - lon. slip                      %
%       V  - velocity                       %
% ----------------------------------------- %
ccc;

%% User selections
% Options
tolerance       = 0.1; % -
smoothingFactor = 0.2; % -

% Data ranges (same units as data file)
P_sel   = 97;
FZ_sel  = 1150;
IA_sel  = 0;
V_sel   = 40;

%% Load tire data
[loadfile,loadpath] = uigetfile('*.mat','Open tire test data (.mat)');
if isequal(loadfile,0)
    disp('User selected Cancel.');
    return
end
loadpathfile = fullfile(loadpath,loadfile);
tireData = load(loadpathfile);
disp(['Loaded file: ' loadfile]);
if isfield(tireData,'tireid')
    disp(tireData.tireid);
end

%% Find data range
if max(abs(tireData.SL)) > tolerance
    forceInXdirection = true;
else
    forceInXdirection = false;
end

if forceInXdirection
    cond = abs(tireData.P) > P_sel *(1-tolerance) & abs(tireData.P)  < P_sel *(1+tolerance) ...
        & abs(tireData.FZ) > FZ_sel*(1-tolerance) & abs(tireData.FZ) < FZ_sel*(1+tolerance) ...
        & abs(tireData.IA) > IA_sel-tolerance     & abs(tireData.IA) < IA_sel+tolerance     ...
        & abs(tireData.V)  > V_sel *(1-tolerance) & abs(tireData.V)  < V_sel *(1+tolerance) ...
        & abs(tireData.SA) < tolerance;
else
    cond = abs(tireData.P) > P_sel *(1-tolerance) & abs(tireData.P)  < P_sel *(1+tolerance) ...
        & abs(tireData.FZ) > FZ_sel*(1-tolerance) & abs(tireData.FZ) < FZ_sel*(1+tolerance) ...
        & abs(tireData.IA) > IA_sel-tolerance     & abs(tireData.IA) < IA_sel+tolerance     ...
        & abs(tireData.V)  > V_sel *(1-tolerance) & abs(tireData.V)  < V_sel *(1+tolerance) ...
        & abs(tireData.SL) < tolerance;
end

if trapz(cond) < 100
    error('No matching data found.');
end

% Select data by condition
FX = tireData.FX(cond);
FY = tireData.FY(cond);
FZ = tireData.FZ(cond);
IA = tireData.IA(cond);
MX = tireData.MX(cond);
MZ = tireData.MZ(cond);
P  = tireData.P(cond) ;
SA = tireData.SA(cond);
SL = tireData.SL(cond);
V  = tireData.V(cond) ;

%% Isolate single slip sweep
if forceInXdirection
    [~,SLmaxpos] = max(SL);
    [~,SLminpos] = min(SL);
    startPoint = min([SLmaxpos SLminpos]);
    endPoint   = max([SLmaxpos SLminpos]);
else
    [~,SAmaxpos] = max(SA);
    [~,SAminpos] = min(SA);
    startPoint = min([SAmaxpos SAminpos]);
    endPoint   = max([SAmaxpos SAminpos]);
end

FX = FX(startPoint:endPoint);
FY = FY(startPoint:endPoint);
FZ = FZ(startPoint:endPoint);
IA = IA(startPoint:endPoint);
MX = MX(startPoint:endPoint);
MZ = MZ(startPoint:endPoint);
P  = P(startPoint:endPoint) ;
SA = SA(startPoint:endPoint);
SL = SL(startPoint:endPoint);
V  = V(startPoint:endPoint) ;

%% Smoothing
if smoothingFactor > 0
    FXraw = FX;
    FYraw = FY;
    FZraw = FZ;
    MXraw = MX;
    MZraw = MZ;
    SAraw = SA;
    SLraw = SL;
    
    FX = smoothdata(FXraw,'loess','SmoothingFactor',smoothingFactor);
    FY = smoothdata(FYraw,'loess','SmoothingFactor',smoothingFactor);
    FZ = smoothdata(FZraw,'loess','SmoothingFactor',smoothingFactor);
    MX = smoothdata(MXraw,'loess','SmoothingFactor',smoothingFactor);
    MZ = smoothdata(MZraw,'loess','SmoothingFactor',smoothingFactor);
    SA = smoothdata(SAraw,'lowess','SmoothingFactor',smoothingFactor);
    SL = smoothdata(SLraw,'lowess','SmoothingFactor',smoothingFactor);
end

%% TMeasy Parameters
if forceInXdirection
    dF_x0 = max(abs( diff(FX)./diff(SL) ));
    s_xMA = SL(FX==max(FX));
    F_xMA = max( FX );
    s_xMB = -1*SL(FX==min(FX));
    F_xMB = -1*min( FX );

    disp(['FZ = ' num2str(FZ_sel) 'N, IA = ' num2str(IA_sel) '°, P = ' num2str(P_sel) 'kPa']);
    disp(['dF_x0 = ' num2str(round(dF_x0,-3))]);
    disp(['s_xMA = ' num2str(round(s_xMA,3))]);
    disp(['F_xMA = ' num2str(round(F_xMA,-1))]);
    disp(['s_xMB = ' num2str(round(s_xMB,3))]);
    disp(['F_xMB = ' num2str(round(F_xMB,-1))]);
else
    dF_y0 = max(abs( diff(FY)./diff(tand(SA)) ));
    s_yM  = abs( tand(SA(FY==min(FY))) );
    F_yM  = abs(min( FY ));

    disp(['FZ = ' num2str(FZ_sel) 'N, IA = ' num2str(IA_sel) '°, P = ' num2str(P_sel) 'kPa']);
    disp(['dF_y0 = ' num2str(round(dF_y0,-3))]);
    disp(['s_yM  = ' num2str(round(s_yM,3))]);
    disp(['F_yM  = ' num2str(round(F_yM,-1))]);
end

%% Plot selection
close all;
figure;

% FX
ax1 = subplot(131); hold on;
plot(SL.*100,FX);
if smoothingFactor > 0
    plot(SLraw.*100,FXraw,'k:');
end
if forceInXdirection
    plot([-s_xMB s_xMA].*100,[-F_xMB F_xMA],'ro');
end
title('Longitudinal Force');
xlabel('s_x in %');
ylabel('F_x in N');
grid on;

% FY
ax2 = subplot(132); hold on;
plot(SA,FY);
if smoothingFactor > 0
    plot(SAraw,FYraw,'k:');
end
if ~forceInXdirection
    plot(atand(s_yM),-F_yM,'ro');
end
title('Lateral Force');
xlabel('\alpha in °');
ylabel('F_y in N');
grid on;

% MZ
ax3 = subplot(233); hold on;
plot(SA,MZ);
if smoothingFactor > 0
    plot(SA,MZraw,'k:');
end
title('Aligning Torque');
xlabel('\alpha in °');
ylabel('M_z in Nm');
grid on;

% MX
ax4 = subplot(236); hold on;
plot(SA,MX);
if smoothingFactor > 0
    plot(SA,MXraw,'k:');
end
title('Tipping Torque');
xlabel('\alpha in °');
ylabel('M_x in Nm');
grid on;

% Link plot axes
linkaxes([ax1 ax2],'y'); linkaxes([ax2 ax3 ax4],'x');