close all
clc 
clear all

load('dynamics_data.mat')

figure('name', 'velocities and accelerations')

subplot(2,2,1)
plot(t0,omega_1, t0, omega_2, t0, omega_3)
grid on
xlabel('time')
ylabel('omega rad/s')
legend('link 1', 'link 2', 'link 3')
title('Angular velocity projection on local axis of rotation')

subplot(2,2,2) 
plot(t0, alpha_1, t0, alpha_2, t0, alpha_3)
grid on
xlabel('time')
ylabel('alpha rad/s^2')
legend('link 1', 'link 2', 'link 3')
title('Angular acceleration projection on local axis of rotation')

subplot(2,2,3)
plot(t0, coriollos3)
grid on
% xlim([-1 11])
% ylim([-3 1])
xlabel('time')
ylabel('C rad/s^2')
legend('link 3')
title('coriollos components projection on local axis of rotation')

subplot(2,2,4)
plot(t0, centrifugal1, t0,centrifugal2,t0,centrifugal3 )
grid on
% xlim([-1 11])
% ylim([0 0.05])
xlabel('time')
ylabel('C m/s^2')
legend('Link 1','Link 2','link 3')
title('centrifugal component projection on local axis of rotation')

figure('name', 'Linear acceleration')
subplot(2,1,1)
plot(t0,ae_1,t0,ae_2,t0,ae_3)
legend('link 1', 'link 2', 'link 3')
xlabel('time')
ylabel('Ae m/s^2')
title('Links linear acceleration in World frame')

subplot(2,1,2)
plot(t0, ac_1,t0, ac_2,t0,ac_3)
legend('Clink 1', 'Clink 2', 'Clink 3')
xlabel('time')
ylabel('Ac m/s^2')
title('Links center of gravity linear acceleration in World frame')

figure('name', 'Forces and Torques')
subplot(3,1,1)
plot(t0, T_0,t0, T_1)
legend('T link 1', 'T link 2')
xlabel('time')
ylabel('Torque N.m')
title('Torques of first two revolute joints')

subplot(3,1,2)
plot(t0, F_2)
legend('Force')
xlabel('time')
ylabel('Force N')
title('Force of Prysmatic Joint')

subplot(3,1,3)
plot(t0, g_1,t0,g_2,t0,g_3)
legend('link 1', 'link 2', 'link 3')
xlabel('time')
ylabel('Force N')
title('Gravitational Forces')
