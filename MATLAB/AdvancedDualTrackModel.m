%%      Advanced Dual Track Model          %%
% ----------------------------------------- %
% ADTM Release: ADTM_1.4 - 2025.05.26       %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% ----------------------------------------- %
%% MATLAB path setup
path = fileparts(mfilename('fullpath'));
cd(path);
addpath(genpath(path));
ccc; % "ccc" = "clc; clear; close all;"

%% User options
% Parameter selection
opts.envConditions = 'default_mu1.0';

% Simulation options
opts.simFreq        = 200;          % Hz
opts.dataFreq       = 100;          % Hz
opts.solver         = 'Runge-Kutta';% Euler / Heun / Runge-Kutta
opts.tireModelName  = 'TMeasy';     % linear / TMeasy
opts.tireDyn        = true;         % true / false
opts.plotInputSigs  = true;         % true / false
% opts.tEnd           = 30;           % Optional override of end time in s

%% Run simulation
[TTin,TTout,road,opts] = f_runSim(opts);

%% Plot results
f_plotResults(TTin,TTout,road,opts);