close all

global position_init
global theta_init
global pend_length

addpath('simulinkfiles')
load('matfiles/params_cart_identification')

%% parameter setting

E = 1.2; % voltage as step input 

%% simulation

Model = 'cart_simulator';
% Model = 'cart_simulator_2019';
open_system(Model);
set_param(Model, 'StopTime', '2');
sim(Model);
close_system(Model);
data = ans;

%% plot

figure
plot(data.t, data.position, 'b', 'LineWidth', 3)
grid on
xlabel('Time [s]')
ylabel('Cart Position [m]')
set(gca, 'FontSize', 23)

figure
plot(data.t, data.voltage, 'b', 'LineWidth', 3)
grid on
xlabel('Time [s]')
ylabel('Voltage [V]')
set(gca, 'FontSize', 23)

