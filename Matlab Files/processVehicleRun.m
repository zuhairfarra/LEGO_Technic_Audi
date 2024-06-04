%% Rear drivetrain PWM test
clear; close all

voltage_supply_V = 8.02;

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);

drivedata = importDriveData('drivevehicledata_50dc100ms_01.txt',4);
drivedata.Properties.VariableNames{1} = 'Time_sec';
drivedata.Properties.VariableNames{2} = 'Current_mA';
drivedata.Properties.VariableNames{3} = 'FWSpeed_dps';
drivedata.Properties.VariableNames{4} = 'RWSpeed_dps';

plottingOn = 1;

figure;
plot(drivedata.Time_sec,drivedata.RWSpeed_dps,'-r');
grid on
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Angular Velocity $\omega$',"Interpreter","latex")
hold off

getVelocity_radpsec = @(speedvec,timevec,x1,x2) mean(speedvec(inRange(timevec,x1,x2)));
% duty_cycles_dyn = [0.50];
rangeMat = [3,12];

speedMeans_dps.FW_grounded = getVelocity_radpsec(drivedata.FWSpeed_dps,...
    drivedata.Time_sec,rangeMat(1),rangeMat(2));
speedMeans_dps.RW_grounded = getVelocity_radpsec(drivedata.RWSpeed_dps,...
    drivedata.Time_sec,rangeMat(1),rangeMat(2));

TireRadius_m = 0.04;
n_diff = 22/14;
estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;
meanVehicleSpeed_mps = structfun(@(x) estimateVehicleSpeed(x),speedMeans_dps,'UniformOutput',false);

vehicleSpeed_mps.FW_grounded = estimateVehicleSpeed(drivedata.FWSpeed_dps);
vehicleSpeed_mps.RW_grounded = estimateVehicleSpeed(drivedata.RWSpeed_dps);
