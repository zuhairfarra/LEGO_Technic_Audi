%% Rear drivetrain PWM test
close all; clear

voltage_supply_V = 8.866;

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);

drivedata = importDriveData('imuvehicledata03_60R.txt',4);
drivedata.Properties.VariableNames{1} = 'Time_sec';
drivedata.Properties.VariableNames{2} = 'RWSpeed_dps';
drivedata.Properties.VariableNames{3} = 'YawRate_dps';
drivedata.Properties.VariableNames{4} = 'SteeringAngle_deg';

figure
plot(drivedata.Time_sec,drivedata.RWSpeed_dps,'-b');
grid on

figure;
subplot(1,2,1)
plot(drivedata.Time_sec,drivedata.YawRate_dps,'-b');
grid on
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Angular Velocity $\omega$',"Interpreter","latex")
hold off
subplot(1,2,2)
plot(drivedata.Time_sec,drivedata.SteeringAngle_deg,'-b');
grid on
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Translational Acceleration $a (m/s^2)$',"Interpreter","latex")
hold off

getMean = @(valvec,timevec,x1,x2) mean(valvec(inRange(timevec,x1,x2)));

% duty_cycles_dyn = [0.30];
rangeMat = [0,2;2,8;9,14;15,20;20,24];
driveOn = logical([0;1;1;1;0]);
steerOn = logical([0;0;1;0;0]);

TireRadius_m = 0.04;
n_diff = 22/14;
estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;
wheelbase_m = 0.237;
tirewidth_m = 0.03;
treadwidth_m = 0.042*2+0.055+tirewidth_m;

for idx = 1:size(rangeMat,1)
    speedMeans_dps(idx) = getMean(...
        drivedata.RWSpeed_dps,drivedata.Time_sec,rangeMat(idx,1),rangeMat(idx,2));
    steeringAngleMeans_deg(idx) = getMean(...
        drivedata.SteeringAngle_deg,drivedata.Time_sec,rangeMat(idx,1),rangeMat(idx,2));
    yawRateMeans_dps(idx) = getMean(...
        drivedata.YawRate_dps,drivedata.Time_sec,rangeMat(idx,1),rangeMat(idx,2));
end

meanVehicleSpeed_mps = estimateVehicleSpeed(speedMeans_dps);
yawRateBias_dps = mean([yawRateMeans_dps(2),yawRateMeans_dps(4)]);
biasedYawRate_dps = yawRateMeans_dps - yawRateBias_dps;
curvature_dpm = biasedYawRate_dps(steerOn)/meanVehicleSpeed_mps(steerOn)

wheelAngle_deg = sign(curvature_dpm)*atand(abs(curvature_dpm)*pi/180*wheelbase_m)
% outerWheelAngle_deg = sign(curvature_dpm)*atand(wheelbase_m/...
%     (abs(inv(curvature_dpm*pi/180))+treadwidth_m));

% meanWheelAngle_deg = mean([innerWheelAngle_deg,outerWheelAngle_deg])

vehicleSpeed_mps.RW_grounded = estimateVehicleSpeed(drivedata.RWSpeed_dps);

% [-60 14;90 18;-60 13.65]

% save("Data\Vehicle_FWD_OneCycleIMU_50DC03.mat","drivedata","speedMeans_dps","vehicleSpeed_mps")