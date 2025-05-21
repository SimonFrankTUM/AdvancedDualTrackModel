function [TTin,input,road,opts] = f_loadInput(opts)
%%              Load Input                 %%
% ----------------------------------------- %
% Version: V1.3 - 2025.05.21                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Input:                                    %
%   opts    -   struct of additional model  %
%               options                     %
% Output:                                   %
%   TTin    -   timetable of input signals  %
%   input   -   struct of model input       %
%               signals                     %
%   road    -   struct of road              %
%               x, y, and Z data            %
%   opts    -   struct of additional model  %
%               options                     %
% ----------------------------------------- %

%% Input
listing = dir('.\input');
tbl = struct2table(listing);
listNames = tbl.name(~tbl.isdir,:);
userSelIdx = listdlg('Name','Select simulation input file (.csv)', ...
    'SelectionMode','single', 'ListSize',[512 256],'ListString',listNames);

if isempty(userSelIdx)
    error('User selected Cancel.');
end

try
    loadfile = listNames{userSelIdx};
    loadpathfile = ['.' filesep 'input' filesep loadfile];
    TTin        = readtable(loadpathfile);
    TTin.Time   = duration(TTin.Time,'InputFormat','mm:ss.SSS','Format','mm:ss.SSSSS');
    TTin        = table2timetable(TTin);
catch
    error('Invalid input file selected.');
end

disp(['Input file: ' loadfile]);

%% Road
listing = dir('.\roads');
tbl = struct2table(listing);
listNames = [{'none (flat road)'}; tbl.name(~tbl.isdir,:)];
userSelIdx = listdlg('Name','Open road file (.mat)','CancelString','Skip', ...
    'SelectionMode','single', 'ListSize',[512 256],'ListString',listNames);

if isempty(userSelIdx) || userSelIdx==1
    % Don't use 3D road
    road = struct.empty();
    opts.use3DRoad = false;
    disp('Road file: none (flat road)');
else
    % Use 3D road
    try
        loadfile = listNames{userSelIdx};
        loadpathfile = ['.' filesep 'roads' filesep loadfile];
        road = load(loadpathfile);
    catch
        error('Invalid road file selected.');
    end

    opts.use3DRoad = true;
    disp(['Road file: ' loadfile]);
end

%% Interpolate inputs
if isfield(opts,'tEnd')
    opts.tEnd = min([opts.tEnd seconds(TTin.Time(end))]);
else
    opts.tEnd = seconds(TTin.Time(end));
end

TTinHF = timetable(TTin.Time,TTin.delta_D,TTin.p_BFA,TTin.p_BRA,TTin.throttle,TTin.gear,TTin.mu_N);
TTinHF.Properties.VariableNames = {'delta_D' 'p_BFA' 'p_BRA' 'throttle' 'gear' 'mu_N'};
TTinHF = retime(TTinHF,'regular','linear','TimeStep',seconds(1/opts.simFreq));
input = table2struct(TTinHF);

%% Initial states
opts.x_0 = zeros(opts.nStates,1);

% Initial velocity available
if any(strcmp('v_x',string(TTin.Properties.VariableNames))) && ...
   any(strcmp('v_y',string(TTin.Properties.VariableNames))) && ...
   any(strcmp('v_z',string(TTin.Properties.VariableNames)))
   
    opts.x_0(15) = TTin.v_x(1);
    opts.x_0(16) = TTin.v_y(1);
    opts.x_0(17) = TTin.v_z(1);
end

% Initial wheel speeds available
if any(strcmp('omega_FL',string(TTin.Properties.VariableNames))) && ...
   any(strcmp('omega_FR',string(TTin.Properties.VariableNames))) && ...
   any(strcmp('omega_RL',string(TTin.Properties.VariableNames))) && ...
   any(strcmp('omega_RR',string(TTin.Properties.VariableNames)))

    opts.x_0(25) = TTin.omega_FL(1);
    opts.x_0(26) = TTin.omega_FR(1);
    opts.x_0(27) = TTin.omega_RL(1);
    opts.x_0(28) = TTin.omega_RR(1);
end

end