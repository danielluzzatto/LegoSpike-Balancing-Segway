close all;clc

L = 9*10^-2; %cm, full length pendulum
g = 9.81; %m/s^2
l = L/2; % half length of the pendulum
b = 0.01; % friction coefficient

diam = 0.056;
r = diam/2;
h_motor = 0.0715;
w_motor = 0.023;
h_battery = 0.0315;
w_battery = 0.056;
m_tot = 0.292;
m_wheel =0.015; % cart (wheels)
m_motor = 0.05;
m_battery = 0.163;
m_beam = 0.003;
h_beam = 0.007;
w_beam = 0.0075;
a = 0.009; %distance from the bottom part of the battery to the axis of rotation
d_motor = h_motor/2-a;
d_battery = h_battery/2-a;
d_beam = 0.0715-a;

inertia_motor = 1/12*m_motor*(h_motor^2+w_motor^2)+m_motor*d_motor^2;
inertia_battery = 1/12*m_battery*(h_battery^2+w_battery^2)+m_battery*d_battery^2;
inertia_beam = 1/12*m_beam*(h_beam^2+w_beam^2)+m_beam*d_beam^2;
I = inertia_battery+2*inertia_motor+ 2*inertia_beam;

M = 2*m_wheel; % cart
m = m_tot-2*m_wheel; % pendulum

% State Space Representation
A = [0, 1, 0, 0;
    0, -(I + m*l^2)*b / (I*(M+m) + M*m*l^2), (m^2 * g * l^2) / (I*(M+m) + M*m*l^2), 0;
    0, 0, 0, 1;
    0, -(m*l*b) / (I*(M+m) + M*m*l^2), m*g*l*(M+m) / (I*(M+m) + M*m*l^2), 0];

B = [0;
    (I + m*l^2) / (I*(M+m) + M*m*l^2);
    0;
    m*l / (I*(M+m) + M*m*l^2)];

C = [1, 0, 0, 0;
    0, 0, 1, 0];

D = [0; 0];



%% PID Simulation
close all; clc
data = sim("segway_linearizaton_PID.slx");

theta = data.pend_angle.signals.values;
time = data.tout;
figure;
plot(time,theta)
grid minor
xlabel('Time [s]')
ylabel('Theta [deg]')
yline(1,'--','1 [deg]','LabelVerticalAlignment','top')
yline(-1,'--', '-1 [deg]','LabelVerticalAlignment','bottom')
title('Pendulum angle closed loop response')
xlim([0 0.3])
ylim([-7 7])
saveas(gcf, 'Simulation_PID_zoom.png')

figure;
plot(time,theta)
grid minor
xlabel('Time [s]')
ylabel('Theta [deg]')
title('Pendulum angle closed loop response')
xlim([0 3])
saveas(gcf, 'Simulation_PID.png')
%% LQR Simulation
clc; close all
sys = ss(A, B, C, D);
tf_sys = tf(sys);

contr = rank(ctrb(sys));
obs = rank(obsv(sys));
eigen_A = eig(A);
Q = 10^6*eye(4);
R = 0.01;
K = -lqr(sys,Q,R);

A_cl = A + B * K; 
sys_cl = ss(A_cl, B, C, D);

t = linspace(0, 20, 20*40);
x0 = [0, 0, 3*pi/180, 0];  % [position, velocity, angle, angular velocity] IC

[y_out, t_out, x_out] = initial(sys_cl, x0, t);

figure;
plot(t_out, x_out(:, 1));
title('Position plot using LQR');
xlabel('Time (s)');
ylabel('Distance [m]');
grid minor;
ylim([-0.1 0.1])
% saveas(gcf, 'pos_lqr_simulation.png')

figure;
plot(t_out, x_out(:, 2));
title('Linear velocity plot using LQR');
xlabel('Time (s)');
ylabel('Linear Velocity [m/s]');
grid minor;
% saveas(gcf, 'linvel_lqr_simulation.png')

figure;
plot(t_out, x_out(:, 3)*180/pi);
title('Angle plot using LQR');
xlabel('Time (s)');
ylabel('Angle [deg]');
grid minor;
% saveas(gcf, 'angle_lqr_simulation.png')

figure;
plot(t_out, x_out(:, 4));
title('Angular velocity plot using LQR');
xlabel('Time (s)');
ylabel('Angular Velocity [deg/s]');
grid minor;
% saveas(gcf, 'angvel_lqr_simulation.png')

%% Sensor testing
clc; close all
gyro_data = readtable('drift_gyroscope.csv');
gyro_time = gyro_data.timestamp;
gyro_drift = gyro_data.red;

comp_stat = readtable('accel_compl_stattest.csv');
comp_time = comp_stat.timestamp;
comp_accelstat= comp_stat.red;
comp_filtstat= comp_stat.orange;
avg_accelstat = mean(comp_accelstat);
avg_filtstat = mean(comp_filtstat);
maxdev_accelstat = max(comp_accelstat)-min(comp_accelstat);
maxdev_filtstat = max(comp_filtstat)-min(comp_filtstat);
RMSE_stat = sqrt(mean((comp_filtstat - comp_accelstat).^2));

dyn_test = readtable('dynamic_test.csv');
dyn_time = dyn_test.timestamp;
dyn_accel = dyn_test.red;
dyn_filt = dyn_test.orange;
RMSE_dyn = sqrt(mean((dyn_accel - dyn_filt).^2));

figure;
plot(gyro_time, gyro_drift)
grid minor;
xlabel('Time [s]')
ylabel('Angular velocity [deg/s]')
title('Stationary test on Gyroscope')
saveas(gcf, 'gyro_drift.png')

figure;
plot(comp_time, comp_accelstat)
hold on
plot(comp_time, comp_filtstat)
grid minor;
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('Accelerometer reading', 'Complementary filter reading')
title('Stationary test on Accelerometer and Complementary filter')
saveas(gcf, 'accel_filt_stat.png')

figure;
plot(dyn_time, dyn_accel)
hold on
plot(dyn_time, dyn_filt)
grid minor;
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('Accelerometer reading', 'Complementary filter reading', 'Location','northwest')
title('Dynamic test on Accelerometer and Complementary filter')
saveas(gcf, 'dyn_test.png')
%% LQR results

lqr_test = readtable('LQR_plotf.csv');
time_lqr = lqr_test.timestamp; 
angle_lqr = lqr_test.red;
pos_lqr = lqr_test.orange;

figure;
plot(time_lqr, angle_lqr)
grid minor
xlabel('Time [s]')
ylabel('Angle [deg]')
title('Angle measurement')
xlim([0, time_lqr(end)])
saveas(gcf, 'lqr_angle.png')

figure;
plot(time_lqr, pos_lqr)
grid minor
xlabel('Time [s]')
ylabel('Distance [m]')
title('Position measurement')
xlim([0, time_lqr(end)])
saveas(gcf, 'lqr_pos.png')

