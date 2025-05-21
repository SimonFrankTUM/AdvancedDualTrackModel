function f_plotResults(TTin, TTout, road, opts)
%%      Plot Simulation Results            %%
% ----------------------------------------- %
% Version: V1.2 - 2025.05.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   TTin    -   timetable of input signals  %
%   TTout   -   timetable of output signals %
%   road    -   struct of road              %
%               x, y, and Z data            %
%   opts    -   struct of additional model  %
%               options                     %
% Output:                                   %
% ----------------------------------------- %
close all;

%% Colors
% Line colors
colorsTUM = [   0   101 189; ...
                162 173 0;   ...
                227 114 34;  ...
                100 160 200; ...
                0   82  147; ...
                152 198 234; ...
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

%% General
inIdx = TTin.Time <= TTout.Time(end);

%% Plot 1: Position
figure('Name','Position','Units','normalized','OuterPosition',[0 0.5 0.5 0.5]); hold on;
title('Position');
plot3(TTout.x_V(1),TTout.y_V(1),TTout.z_V(1),'o','Color',colors(1,:));
plot3(TTout.x_V',TTout.y_V',TTout.z_V','Color',colors(1,:));

% Velocity vectors
nQuivers = floor(length(TTout.Time)/opts.dataFreq) -1;
u = zeros(1,nQuivers); v = u; w = u;
for i = 1:nQuivers
    dirvec = f_zrot(TTout.psi(i*opts.dataFreq)) * f_yrot(TTout.theta(i*opts.dataFreq)) * f_xrot(TTout.phi(i*opts.dataFreq)) ...
        * [TTout.v_x(i*opts.dataFreq);TTout.v_y(i*opts.dataFreq);TTout.v_z(i*opts.dataFreq)];
    u(i) = dirvec(1);
    v(i) = dirvec(2);
    w(i) = dirvec(3);
end
idxs = (1:nQuivers).*opts.dataFreq;
quiver3(TTout.x_V(idxs)',TTout.y_V(idxs)',TTout.z_V(idxs)',u,v,w,0.5,'Color',colors(1,:));

% Road
if opts.use3DRoad
    surf(road.x*0.1,road.y*0.1,road.Z','EdgeAlpha',0,'FaceColor',[0.67 .67 .67]);
    xlim([min(TTout.x_V)-4 max(TTout.x_V)+4]);
    ylim([min(TTout.y_V)-4 max(TTout.y_V)+4]);
    zlim([min(TTout.z_V)-4 max(TTout.z_V)+4]);
end

axis equal;
view(-45,60);
xlabel('X in m');
ylabel('Y in m');
zlabel('Z in m');


%% Plot 2: Velocities
figure('Name','Velocities','Units','normalized','OuterPosition',[0.5 0.5 0.5 0.5]);
title('Velocities');

ax1 = subplot(211); hold on;
plot(TTout.Time,TTout.v_x.*3.6,'Color',colors(1,:));
if opts.plotInputSigs && any(strcmp('v_x',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.v_x(inIdx).*3.6,'--','Color',colors(1,:));
end
title('Longitudinal velocity v_x');
ylabel('km/h');
grid on;

ax2 = subplot(212); hold on;
plot(TTout.Time,TTout.v_y.*3.6,'Color',colors(2,:));
if opts.plotInputSigs && any(strcmp('v_y',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.v_y(inIdx).*3.6,'--','Color',colors(2,:));
end
title('Lateral velocity v_y');
ylabel('km/h');
grid on;

% ax3 = subplot(313); hold on;
% plot(TTout.Time,TTout.v_z.*3.6,'Color',colors(3,:));
% if opts.plotInputSigs && any(strcmp('v_z',string(TTin.Properties.VariableNames)))
%     plot(seconds(TTin.Time(inIdx)),TTin.v_z(inIdx).*3.6,'--','Color',colors(3,:));
% end

linkaxes([ax1 ax2],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);
if max(abs(TTout.v_y.*3.6))<1; ylim(ax2,[-1 1]); end


%% Plot 3: Accelerations
figure('Name','Accelerations','Units','normalized','OuterPosition',[0 0 0.5 0.5]);

ax1 = subplot(211); hold on;
plot(TTout.Time,TTout.a_x,'Color',colors(1,:));
plot(TTout.Time,TTout.a_y,'Color',colors(2,:));
plot(TTout.Time,TTout.a_z,'Color',colors(3,:));

if opts.plotInputSigs && any(strcmp('a_x',string(TTin.Properties.VariableNames)))
    axOffset = 0; % TTout.a_x(1) - TTin.a_x(1);
    plot(seconds(TTin.Time(inIdx)),TTin.a_x(inIdx)+axOffset,'--','Color',colors(1,:));
end
if opts.plotInputSigs && any(strcmp('a_y',string(TTin.Properties.VariableNames)))
    ayOffset = 0; % TTout.a_y(1) - TTin.a_y(1);
    plot(seconds(TTin.Time(inIdx)),TTin.a_y(inIdx)+ayOffset,'--','Color',colors(2,:));
end
if opts.plotInputSigs && any(strcmp('a_z',string(TTin.Properties.VariableNames)))
    azOffset = 0; % TTout.a_z(1) - TTin.a_z(1);
    plot(seconds(TTin.Time(inIdx)),TTin.a_z(inIdx)+azOffset,'--','Color',colors(3,:));
end
title('Accelerations');
legend('a_x','a_y','a_z','Location','northwest');
ylabel('m/s^2');
grid on;

ax2 = subplot(212); hold on;
plot(TTout.Time,TTout.omega_z.*(180/pi),'Color',colors(3,:));
if opts.plotInputSigs && any(strcmp('omega_z',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_z(inIdx).*(180/pi),'--','Color',colors(3,:));
end
title('Yaw rate \omega_z');
ylabel('°/s');
grid on;

linkaxes([ax1 ax2],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);
if max(abs(TTout.omega_z))<1/57.3; ylim(ax2,[-1 1]); end


%% Plot 4: Wheel speeds
figure('Name','Wheel speeds','Units','normalized','OuterPosition',[0.5 0 0.5 0.5]); hold on;

plot(TTout.Time,TTout.omega_FL.*(30/pi),'Color',colors(1,:));
plot(TTout.Time,TTout.omega_FR.*(30/pi),'Color',colors(2,:));
plot(TTout.Time,TTout.omega_RL.*(30/pi),'Color',colors(3,:));
plot(TTout.Time,TTout.omega_RR.*(30/pi),'Color',colors(4,:));

if opts.plotInputSigs && any(strcmp('omega_FL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_FL(inIdx).*(30/pi),'--','Color',colors(1,:));
end
if opts.plotInputSigs && any(strcmp('omega_FR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_FR(inIdx).*(30/pi),'--','Color',colors(2,:));
end
if opts.plotInputSigs && any(strcmp('omega_RL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_RL(inIdx).*(30/pi),'--','Color',colors(3,:));
end
if opts.plotInputSigs && any(strcmp('omega_RR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_RR(inIdx).*(30/pi),'--','Color',colors(4,:));
end

xlim([ TTout.Time(1) seconds(opts.tEnd) ]);
title('Wheel speeds \omega');
ylabel('rpm');
legend({'FL' 'FR' 'RL' 'RR'},'Location','northwest');
grid on;



%% Plot 5: Chassis angles
figure('Name','Chassis angles','Units','normalized','OuterPosition',[0.25 0.25 0.5 0.5],'WindowState','minimized');

ax1 = subplot(311); hold on;
plot(TTout.Time,TTout.phi.*(180/pi),'Color',colors(1,:));
title('Roll angle \phi');
ylabel('°');
grid on;
if max(abs(TTout.phi))<1/57.3; ylim([-1 1]); end

ax2 = subplot(312); hold on;
plot(TTout.Time,TTout.theta.*(180/pi),'Color',colors(1,:));
title('Pitch angle \theta');
ylabel('°');
grid on;
if max(abs(TTout.theta))<1/57.3; ylim([-1 1]); end

ax3 = subplot(313); hold on;
plot(TTout.Time,TTout.psi.*(180/pi),'Color',colors(1,:));
title('Yaw angle \psi');
ylabel('°');
grid on;
if max(abs(TTout.psi))<1/57.3; ylim([-1 1]); end

linkaxes([ax1 ax2 ax3],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);


%% Plot 6: Wheel angles & tire forces
figure('Name','Wheel angles & tire forces','Units','normalized','OuterPosition',[0.275 0.275 0.5 0.5],'WindowState','minimized');

ax1 = subplot(321); hold on;
plot(TTout.Time,TTout.delta_FL.*180./pi,'Color',colors(1,:));
title('\delta Front left');
ylabel('°');
grid on;

ax2 = subplot(322); hold on;
plot(TTout.Time,TTout.delta_FL.*180./pi,'Color',colors(1,:));
title('\delta Front right');
ylabel('°');
grid on;

ax3 = subplot(323); hold on;
plot(TTout.Time,TTout.F_xFL.*1E-3,'Color',colors(1,:));
plot(TTout.Time,TTout.F_yFL.*1E-3,'Color',colors(2,:));
plot(TTout.Time,TTout.F_zFL.*1E-3,'Color',colors(3,:));
title('Front left force');
ylabel('kN');
legend({'F_x' 'F_y' 'F_z'},'Location','northwest');
grid on;

ax4 = subplot(324); hold on;
plot(TTout.Time,TTout.F_xFR.*1E-3,'Color',colors(1,:));
plot(TTout.Time,TTout.F_yFR.*1E-3,'Color',colors(2,:));
plot(TTout.Time,TTout.F_zFR.*1E-3,'Color',colors(3,:));
title('Front right force');
ylabel('kN');
grid on;

ax5 = subplot(325); hold on;
plot(TTout.Time,TTout.F_xRL.*1E-3,'Color',colors(1,:));
plot(TTout.Time,TTout.F_yRL.*1E-3,'Color',colors(2,:));
plot(TTout.Time,TTout.F_zRL.*1E-3,'Color',colors(3,:));
title('Rear left force');
ylabel('kN');
grid on;

ax6 = subplot(326); hold on;
plot(TTout.Time,TTout.F_xRR.*1E-3,'Color',colors(1,:));
plot(TTout.Time,TTout.F_yRR.*1E-3,'Color',colors(2,:));
plot(TTout.Time,TTout.F_zRR.*1E-3,'Color',colors(3,:));
title('Rear right force');
ylabel('kN');
grid on;

linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);


%% Plot 7: Wheel torques
figure('Name','Wheel torques','Units','normalized','OuterPosition',[0.3 0.3 0.5 0.5],'WindowState','minimized');

ax1 = subplot(221); hold on;
plot(TTout.Time,TTout.T_DriveFL,'Color',colors(1,:));
plot(TTout.Time,TTout.T_BrakeFL,'Color',colors(2,:));
title('Front left');
ylabel('Nm');
legend({'Drivetrain' 'Brakes'},'Location','northwest');
grid on;

ax2 = subplot(222); hold on;
plot(TTout.Time,TTout.T_DriveFR,'Color',colors(1,:));
plot(TTout.Time,TTout.T_BrakeFR,'Color',colors(2,:));
title('Front right');
ylabel('Nm');
grid on;

ax3 = subplot(223); hold on;
plot(TTout.Time,TTout.T_DriveRL,'Color',colors(1,:));
plot(TTout.Time,TTout.T_BrakeRL,'Color',colors(2,:));
title('Rear left');
ylabel('Nm');
grid on;

ax4 = subplot(224); hold on;
plot(TTout.Time,TTout.T_DriveRR,'Color',colors(1,:));
plot(TTout.Time,TTout.T_BrakeRR,'Color',colors(2,:));
title('Rear right');
ylabel('Nm');
grid on;

linkaxes([ax1 ax2 ax3 ax4]);
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);


%% Plot 8: Controls & Motor
figure('Name','Controls & Motor','Units','normalized','OuterPosition',[0.325 0.325 0.5 0.5],'WindowState','minimized');

ax1 = subplot(321); hold on;
plot(TTout.Time,TTout.T_Mot,'Color',colors(1,:));
title('Engine Torque');
ylabel('Nm');
grid on;

ax2 = subplot(322); hold on;
plot(seconds(TTin.Time(inIdx)),TTin.throttle(inIdx).*100,'Color',colors(1,:));
title('Throttle');
ylabel('%');
grid on;

ax3 = subplot(323); hold on;
plot(TTout.Time,TTout.omega_Mot.*(30/pi),'Color',colors(1,:));
if opts.plotInputSigs && any(strcmp('omega_M',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.omega_M(inIdx).*(30/pi),'--','Color',colors(1,:));
end
title('Engine Speed');
ylabel('rpm');
grid on;

ax4 = subplot(324); hold on;
plot(seconds(TTin.Time(inIdx)),TTin.delta_D(inIdx).*180./pi,'Color',colors(1,:));
title('Steering');
ylabel('°');
grid on;

ax5 = subplot(325); hold on;
if opts.plotInputSigs && any(strcmp('gear',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.gear(inIdx),'Color',colors(1,:));
end
title('Gear');
ylabel('-');
grid on;

ax6 = subplot(326); hold on;
plot(seconds(TTin.Time(inIdx)),TTin.p_BFA(inIdx),'Color',colors(1,:));
plot(seconds(TTin.Time(inIdx)),TTin.p_BRA(inIdx),'Color',colors(2,:));
title('Brakes');
ylabel('bar');
legend({'Front' 'Rear'},'Location','northwest');
grid on;

% linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x');
% xlim([ TTout.Time(1) seconds(opts.tEnd) ]);


%% Plot 9: Wheel travel
figure('Name','Wheel travel','Units','normalized','OuterPosition',[0.35 0.35 0.5 0.5],'WindowState','minimized');

ax1 = subplot(221); hold on;
plot(TTout.Time,TTout.z_FL.*1E3,'Color',colors(1,:));
if opts.plotInputSigs && any(strcmp('z_FL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),(TTin.z_FL(inIdx)-TTin.z_FL(200)+TTout.z_FL(200)).*1E3,'--','Color',colors(1,:));
end
title('Wheel travel FL');
ylabel('mm');
grid on;

ax2 = subplot(222); hold on;
plot(TTout.Time,TTout.z_FR.*1E3,'Color',colors(2,:));
if opts.plotInputSigs && any(strcmp('z_FR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),(TTin.z_FR(inIdx)-TTin.z_FR(200)+TTout.z_FR(200)).*1E3,'--','Color',colors(2,:));
end
title('Wheel travel FR');
ylabel('mm');
grid on;

ax3 = subplot(223); hold on;
plot(TTout.Time,TTout.z_RL.*1E3,'Color',colors(3,:));
if opts.plotInputSigs && any(strcmp('z_RL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),(TTin.z_RL(inIdx)-TTin.z_RL(200)+TTout.z_RL(200)).*1E3,'--','Color',colors(3,:));
end
title('Wheel travel RL');
ylabel('mm');
grid on;

ax4 = subplot(224); hold on;
plot(TTout.Time,TTout.z_RR.*1E3,'Color',colors(4,:));
if opts.plotInputSigs && any(strcmp('z_RR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),(TTin.z_RR(inIdx)-TTin.z_RR(200)+TTout.z_RR(200)).*1E3,'--','Color',colors(4,:));
end
title('Wheel travel RR');
ylabel('mm');
grid on;

linkaxes([ax1 ax2 ax3 ax4],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);


%% Plot 10: Aero forces
figure('Name','Aero forces','Units','normalized','OuterPosition',[0.375 0.375 0.5 0.5],'WindowState','minimized'); hold on;
plot(TTout.Time,TTout.F_xAero,'Color',colors(1,:));
plot(TTout.Time,TTout.F_yAero,'Color',colors(2,:));
plot(TTout.Time,TTout.F_zAero,'Color',colors(3,:));
title('Aerodynamic forces');
ylabel('N');
grid on;
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);
legend({'F_{xAero}' 'F_{yAero}' 'F_{zAero}'},'Location','northwest');

%% Plot 11: Tire inclination angles
figure('Name','Tire inclination angles','Units','normalized','OuterPosition',[0.4 0.4 0.5 0.5],'WindowState','minimized'); hold on;
plot(TTout.Time,TTout.gamma_FL.*180./pi,'Color',colors(1,:));
plot(TTout.Time,TTout.gamma_FR.*180./pi,'Color',colors(2,:));
plot(TTout.Time,TTout.gamma_RL.*180./pi,'Color',colors(3,:));
plot(TTout.Time,TTout.gamma_RR.*180./pi,'Color',colors(4,:));
title('Tire inclination angles');
ylabel('°');
grid on;
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);
legend({'\gamma_{FL}' '\gamma_{FR}' '\gamma_{RL}' '\gamma_{RR}'},'Location','northwest');

%% Plot 12: Vertical forces
figure('Name','Vertical pushrod forces','Units','normalized','OuterPosition',[0.425 0.425 0.5 0.5],'WindowState','minimized');

ax1 = subplot(221); hold on;
plot(TTout.Time,TTout.F_zPRFL.*1E-3,'Color',colors(1,:));
if opts.plotInputSigs && any(strcmp('F_PRFL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.F_PRFL(inIdx).*1E-3,'--','Color',colors(1,:));
end
title('Vertical pushrod force FL');
ylabel('kN');
grid on;

ax2 = subplot(222); hold on;
plot(TTout.Time,TTout.F_zPRFR.*1E-3,'Color',colors(2,:));
if opts.plotInputSigs && any(strcmp('F_PRFR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.F_PRFR(inIdx).*1E-3,'--','Color',colors(2,:));
end
title('Vertical pushrod force FR');
ylabel('kN');
grid on;

ax3 = subplot(223); hold on;
plot(TTout.Time,TTout.F_zPRRL.*1E-3,'Color',colors(3,:));
if opts.plotInputSigs && any(strcmp('F_PRRL',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.F_PRRL(inIdx).*1E-3,'--','Color',colors(3,:));
end
title('Vertical pushrod force RL');
ylabel('kN');
grid on;

ax4 = subplot(224); hold on;
plot(TTout.Time,TTout.F_zPRRR.*1E-3,'Color',colors(4,:));
if opts.plotInputSigs && any(strcmp('F_PRRR',string(TTin.Properties.VariableNames)))
    plot(seconds(TTin.Time(inIdx)),TTin.F_PRRR(inIdx).*1E-3,'--','Color',colors(4,:));
end
title('Vertical pushrod force RR');
ylabel('kN');
grid on;

linkaxes([ax1 ax2 ax3 ax4],'x');
xlim([ TTout.Time(1) seconds(opts.tEnd) ]);

end