close all

global position_init
global theta_init
global pend_length

addpath('simulinkfiles')
load('matfiles/params_pend_identification')

%% parameter setting

theta_init = pi + 0.1; % pendulum initial angle [radian]

%% simulation

Model = 'pend_simulator';
% Model = 'pend_simulator_2019';
open_system(Model);
set_param(Model, 'StopTime', '10');
sim(Model);
close_system(Model);
data = ans;

%% plot

figure
plot(data.t, data.angle - pi, 'b', 'LineWidth', 3)
grid on
xlabel('Time [s]')
ylabel('Displacement Angle [rad]')

set(gca, 'FontSize', 23)
