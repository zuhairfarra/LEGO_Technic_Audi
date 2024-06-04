close all; clear;
load("Data\DriveData.mat");
load("Data\MotorParameters.mat");
load("Data\MotorInertia.mat");
% Define test run parameters
DutyCycle_pct = timeseries(drivedata.DutyCycle,drivedata.Time_sec);
ModelStopTime = 34;

battery_voltage_V = 9;
R = resistance_mean_ohm;
L = 0.001;

LinSys.A = [-R/L -r_m/L;r_m/J -B_m/J];
LinSys.B = [1/L;0];
LinSys.C = [1 0;0 1];
LinSys.D = [0;0];

% Breakaway happens at 13 pct duty cycle
breakaway_torque = 0.0073; % From best fit line of T_m = B_m * w_m + T_m_0

% Model to run
ModelName = "MotorTestHarness.slx";
% Run model
simOut = sim(ModelName);

%% Post Processing
close all;
Velocity_dps = simOut.Velocity_rpm*360/60;
figure
% plot(simOut.Velocity_dps,'-r')
grid on
hold on; plot(drivedata.Time_sec,drivedata.Velocity_dps,'-b',...
    "DisplayName","Physical Test Data")
plot(simOut.Velocity_rpm.Time,simOut.Velocity_rpm.Data*360/60,'-r',...
    "DisplayName","Model Simulation Data")
hold off
grid on
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Velocity $\omega_m$ (dps)',"Interpreter","latex")
legend('Interpreter','latex')
title('Motor PWM DC Sweep Test: Actual vs. Model',"Interpreter","latex")

Current_mA = simOut.Current_A.Data*1000;
Current_mA = timeseries(Current_mA,simOut.Current_A.Time);

figure
plot(drivedata.Time_sec,drivedata.Current_mA-current_meanBias_A*1000,'-b',...
    "DisplayName","Physical Test Data")
grid on
hold on 
plot(Current_mA,'-r',"DisplayName","Model Simulation Data")
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Current $i_m$ (mA)',"Interpreter","latex")
legend('Interpreter','latex')
title('Motor PWM DC Sweep Test: Actual vs. Model',"Interpreter","latex")
