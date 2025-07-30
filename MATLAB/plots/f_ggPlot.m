function [] = f_ggPlot(TTin,TTout,opts)
%%      GG Plot Of Simulation Results      %%
% ----------------------------------------- %
% Version: V1.2 - 2024.06.03                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   TTin    -   timetable of input signals  %
%   TTout   -   timetable of output signals %
% Output:                                   %
% ----------------------------------------- %

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

%% Smooth signals
smoothFac = 0.15;

a_x_OutSmth = smoothdata(TTout.a_x, 'lowess', 'SmoothingFactor', smoothFac);
a_y_OutSmth = smoothdata(TTout.a_y, 'lowess', 'SmoothingFactor', smoothFac);

[ang_ggOut, a_ggOut] = getggData(a_x_OutSmth,a_y_OutSmth);

plotInput = opts.plotInputSigs ...
    && any(strcmp('a_x',string(TTin.Properties.VariableNames))) ...
    && any(strcmp('a_y',string(TTin.Properties.VariableNames)));

if plotInput
    a_x_InSmth = smoothdata(TTin.a_x, 'lowess', 'SmoothingFactor', smoothFac);
    a_y_InSmth = smoothdata(TTin.a_y, 'lowess', 'SmoothingFactor', smoothFac);
    [ang_ggIn, a_ggIn] = getggData(a_x_InSmth,a_y_InSmth);
end

%% Plot
figure;
hold on;
plot(a_ggOut.*cosd(ang_ggOut), a_ggOut.*sind(ang_ggOut),'.-','Color',colors(1,:),'LineWidth',1.25);
plot(-a_ggOut.*cosd(ang_ggOut), a_ggOut.*sind(ang_ggOut),'.-','Color',colors(1,:),'LineWidth',1.25);
plot(a_y_OutSmth, a_x_OutSmth, ':','Color',colors(1,:));

if plotInput
    plot(a_ggIn.*cosd(ang_ggIn), a_ggIn.*sind(ang_ggIn),'.-','Color',colors(2,:),'LineWidth',1.25);
    plot(-a_ggIn.*cosd(ang_ggIn), a_ggIn.*sind(ang_ggIn),'.-','Color',colors(2,:),'LineWidth',1.25);
    plot(a_y_InSmth, a_x_InSmth, ':','Color',colors(2,:));
    legend('Simulation','','Sensor','Location','northwest');
end

title('Acceleration Limit');
xlabel('a_y in m/s^2');
ylabel('a_x in m/s^2');

bx = gca;
bx.XAxisLocation = 'origin';
bx.YAxisLocation = 'origin';
axis equal 'square';
xlim([-25 25]);
ylim([-25 25]);
grid('on');
grid('minor');

%% Additional functions
    function [a_ang, a_abs] = getggData(a_x,a_y)
        % Calculate angle and abs value for gg-diagram

        % max & min values at ay0
        a_y(1) = 0;         a_y(end) = 0;
        a_x(1) = max(a_x);  a_x(end) = min(a_x);

        % mirror a_y abs values
        a_y_mirr = [abs(a_y); abs(a_y).*-1];
        a_x_doub = [a_x; a_x];

        % find boundaries
        k = boundary(a_x_doub,a_y_mirr,0.1);

        % assign angle and abs value
        a_ang = (-90:5:90)';
        a_meas_ang = atan2d(a_x_doub(k),a_y_mirr(k));

        a_meas = zeros(length(a_meas_ang),2);
        a_meas(:,1) = a_meas_ang;
        a_meas(:,2) = sqrt(a_x_doub(k).^2+a_y_mirr(k).^2);
        a_meas = sortrows(a_meas);
        a_meas = unique(a_meas, "rows");
        a_abs  = zeros(length(a_ang),1);

        for i = 1:length(a_ang)
            [~, nearest_idx1] = min(abs(a_meas(:,1)-a_ang(i)));
            if nearest_idx1 > 1 && nearest_idx1 ~= size(a_meas,1)
                nearest_idx0 = nearest_idx1 -1;
                nearest_idx2 = nearest_idx1 +1;
            else
                nearest_idx0 = nearest_idx1;
                nearest_idx2 = nearest_idx0;
            end
            if nearest_idx1 ~= nearest_idx0 && nearest_idx1 ~= nearest_idx2
                y0 = a_meas(nearest_idx0,2);
                y1 = a_meas(nearest_idx1,2);
                y2 = a_meas(nearest_idx2,2);
                x  = a_ang(i);
                x0 = a_meas(nearest_idx0,1);
                x1 = a_meas(nearest_idx1,1);
                x2 = a_meas(nearest_idx2,1);
                a_abs(i)= interp1([x0 x1 x2], [y0 y1 y2], x, 'spline');
            else
                a_abs(i) = a_meas(nearest_idx1,2);
            end
        end

    end
end