close all; clear;

load_g = [0,128,231,369,540];

drivedata = importDriveData('awd_50dc_01.txt',3);
drivedata.Properties.VariableNames{1} = 'Time_sec';
drivedata.Properties.VariableNames{2} = 'Current_mA';
drivedata.Properties.VariableNames{3} = 'RWSpeed_dps';

load("Data\MotorParameters.mat");
load("Data\MotorInertia.mat");

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getMean = @(valvec,timevec,x1,x2) mean(valvec(inRange(timevec,x1,x2)));

% Define test run parameters
t = 0:0.02:15;
dc_pct = zeros(1,length(t));

% MF_dc_pct = dc_pct;
MR_dc_pct = dc_pct;
MR_dc_pct(all([t > 2;t <= 12])) = 0.5;
MF_dc_pct = MR_dc_pct;

ModelStopTime = 15;

% Each motor driver uses half the input voltage
MF_DutyCycle_pct = timeseries(MF_dc_pct,t);
MR_DutyCycle_pct = timeseries(MR_dc_pct,t);

% Breakaway happens at 13 pct duty cycle
breakaway_torque = 0.0073;
battery_voltage_V = 8.02;
TireRadius_m = 0.04;
m_kg = 0.893;
n_diff = 22/14;
L = 0.001;
f_r = 0.0327;
steeringRatio = 28/112;
wheelBase_m = 0.237;
K_us = 0;
steeringAngle_deg = 0;

l_f = 0.132;
l_r = 0.105;

W_f = m_kg*9.81*l_r/wheelBase_m;
W_r = m_kg*9.81-W_f;

estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;

loadCase = 1;

vehicleMass_kg = 0.893 + load_g(loadCase)/1000;
% Model to run
ModelName = "drivetrainModel.slx";
% Run model
simOut = sim(ModelName);

curvature_dpm = simOut.YawRate_dps.Data/simOut.translationalSpeed_mps.Data;

figure
plot(curvature_dpm)

figure
plot(drivedata.Time_sec,estimateVehicleSpeed(drivedata.RWSpeed_dps),'-b',...
    'DisplayName','Physical Test Data'); hold on;
plot(simOut.translationalSpeed_mps,'-r',...
    'DisplayName','Model Simulation Data'); hold off
grid on
xlabel('Time $(s)$',"Interpreter","latex")
ylabel('Vehicle Speed $(m/s)$',"Interpreter","latex")
legend('Interpreter','latex','Location','east')
title('Vehicle Drive Forward Comparison: Physical vs. Model',"Interpreter","latex")

figure
plot(simOut.Total_Current_A,'-k'); hold on;
plot(test_time_vec,ensembleCurrent_mA.(loadCaseLabels{loadCase})/1000,'-r');
grid on

ss_speed_mps = getMean(ensembleVehicleSpeedMean_mps.(loadCaseLabels{loadCase}),test_time_vec,2,12);

label = datetime;
label = string(label);
label = strrep(label," ","_");
label = strrep(label,":","_");

save(strcat("Data\Sim\Vehicle\",label,".mat"),'simOut')
