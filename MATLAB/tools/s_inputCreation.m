%%          Create Control Inputs          %%
% ----------------------------------------- %
% Version: V1.2 - 2024.07.03                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %
ccc;

%% General
tEnd = 120; % s
freq = 100; % Hz

Time = (0:1/freq:tEnd)';
datalength = length(Time);

%% Selection
inputCase   = 'steerStep'; % steerSine / accelerateAndBrake / steerStep / steerRamp
muCase      = 'Normal'; % Normal / muStep / muRamp / muSine2

%% Driver inputs
switch inputCase
    case 'steerSine'
        throttle = zeros(datalength,1);
        throttle((1*freq):(end-10*freq)) = 0.3;

        gear = ones(datalength,1);
        gear((20*freq):end) = 2;

        delta_D = deg2rad(90)*sin(2*pi*0.5*Time);
        delta_D(1:(4*freq)) = 0;
        delta_D((end-10*freq):end) = 0;

        p_BFA = zeros(datalength,1);
        p_BFA((end-9*freq):end) = 20;
        p_BRA = p_BFA;

    case 'accelerateAndBrake'
        throttle = zeros(datalength,1);
        throttle((1*freq):(20*freq))  = 0.2;
        throttle((30*freq):(40*freq)) = 0.4;
        throttle((50*freq):(60*freq)) = 0.6;
        throttle((70*freq):(80*freq)) = 0.8;
        throttle((90*freq):(100*freq))= 1.0;

        gear = ones(datalength,1);
        gear((35*freq):(42*freq)) = 2;
        gear((53*freq):(56*freq)) = 2;
        gear((56*freq):(62*freq)) = 3;
        gear((72*freq):(74*freq)) = 2;
        gear((74*freq):(77*freq)) = 3;
        gear((77*freq):(82*freq)) = 4;
        gear((91*freq):(94*freq)) = 2;
        gear((94*freq):(96*freq)) = 3;
        gear((96*freq):(99*freq)) = 4;
        gear((99*freq):(103*freq))= 5;

        delta_D = zeros(datalength,1);

        p_BFA = zeros(datalength,1);
        p_BFA(ceil(20.1*freq):(30*freq)) = 20;
        p_BFA(ceil(40.1*freq):(50*freq)) = 25;
        p_BFA(ceil(60.1*freq):(70*freq)) = 30;
        p_BFA(ceil(80.1*freq):(90*freq)) = 35;
        p_BFA(ceil(100.1*freq):end)      = 40;
        p_BRA = p_BFA;

    case 'steerStep'
        throttle = zeros(datalength,1);
        throttle((1*freq):(end-10*freq)) = 0.3;

        gear = ones(datalength,1);
        gear((20*freq):end) = 2;

        delta_D =  zeros(datalength,1);
        delta_D((30*freq):(40*freq)) = deg2rad(30);
        delta_D((50*freq):(60*freq)) = deg2rad(60);
        delta_D((70*freq):(80*freq)) = deg2rad(90);
        delta_D((90*freq):(100*freq))= deg2rad(120);

        p_BFA = zeros(datalength,1);
        p_BFA((end-9*freq):end) = 20;
        p_BRA = p_BFA;

    case 'steerRamp'
        throttle = zeros(datalength,1);
        throttle((1*freq):(end-10*freq)) = 0.3;

        gear = ones(datalength,1);
        gear((20*freq):end) = 2;

        delta_D =  deg2rad(10.*(Time-1));
        for i = 1:datalength
            if delta_D(i) < 0
                delta_D(i) = 0;
            elseif delta_D(i) > deg2rad(1000)
                delta_D(i) = deg2rad(1000);
            end
        end
        delta_D((end-10*freq):end) = 0;

        p_BFA = zeros(datalength,1);
        p_BFA((end-9*freq):end) = 20;
        p_BRA = p_BFA;
    otherwise
        error('Unknown input case');
end

%% Normalized friction over time
switch muCase
    case 'Normal'
        mu_N = ones(datalength,1);

    case 'muStep'
        mu_N = ones(datalength,1);
        mu_N(1:round(end/2)) = 0.67;

    case 'muRamp'
        mu_N = linspace(0.67,1.33,datalength)';

    case 'muSine2'
        mu_N = 1 + 0.33*sin(2*pi*0.5*Time).^2;

    otherwise
        error('Unknown input case');
end

%% Timetable
TT = splitvars(timetable(seconds(Time),[delta_D p_BFA p_BRA throttle gear mu_N]));
TT.Properties.VariableNames = {'delta_D','p_BFA','p_BRA','throttle','gear','mu_N'};
TT.Time = duration(TT.Time,'Format','mm:ss.SSS');
TT = smoothdata(TT,'gaussian',seconds(0.2)); % Smooth steps

%% Plot
figure;

axs(1) = subplot(611);
plot(TT.Time,rad2deg(TT.delta_D));
ylabel('°');
title('\delta_D');
grid on;

axs(2) = subplot(612);
plot(TT.Time,TT.p_BFA);
ylabel('bar');
title('p_{BFA}');
grid on;

axs(3) = subplot(613);
plot(TT.Time,TT.p_BRA);
ylabel('bar');
title('p_{BRA}');
grid on;

axs(4) = subplot(614);
plot(TT.Time,TT.throttle.*100);
ylabel('%');
title('throttle');
grid on;

axs(5) = subplot(615);
plot(TT.Time,TT.gear);
ylabel('-');
title('gear');
grid on;

axs(6) = subplot(616);
plot(TT.Time,TT.mu_N);
ylabel('-');
title('\mu_N');
grid on;

linkaxes(axs,'x');
ylim(axs(4),[-10 110]);
ylim(axs(5),[-1 7]);

%% Save to csv
[savefile,savepath] = uiputfile('*.csv','Save as .csv');
if ~isequal(savefile,0)
    savepathfile = fullfile(savepath,savefile);
    writetimetable(TT,savepathfile,'WriteMode','overwrite');
end