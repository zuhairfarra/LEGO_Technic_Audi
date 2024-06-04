%% Circle test
close all; clear

voltage_supply_V = 8.866;
TireRadius_m = 0.04;
n_diff = 22/14;
steeringRatio = 28/112;
wheelBase_m = 0.237;
m_kg = 0.893;
l_f = 0.132;
l_r = 0.105;

W_f = m_kg*9.81*l_r/wheelBase_m/2;
W_r = (m_kg*9.81-2*W_f)/2;

estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;
inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getMean = @(valvec,timevec,x1,x2) mean(valvec(inRange(timevec,x1,x2)));

fileList = dir("Data\SteeringTest\Turn_90Deg\");
isTxtFile = arrayfun(@(x) contains(x.name,'txt'),fileList);
fileList = fileList(isTxtFile);

steeringMotorAngle_deg = zeros(numel(fileList),1);
meanDrivingSpeed_mps = zeros(numel(fileList),1);
meanYawRate_dps = zeros(numel(fileList),1);
curvature_dpm = zeros(numel(fileList),1);
radius_m = zeros(numel(fileList),1);
K_us = zeros(numel(fileList),1);
C_a = zeros(numel(fileList),1);
lat_accel = zeros(numel(fileList),1);
slope = zeros(numel(fileList),1);

for idx = 1:numel(fileList)

    fileName = fileList(idx).name;
    drivedata = importDriveData(fileName,3);
    drivedata.Properties.VariableNames{1} = 'Time_sec';
    drivedata.Properties.VariableNames{2} = 'Heading_deg';
    drivedata.Properties.VariableNames{3} = 'RWSpeed_dps';

    vehicleSpeed_mps = estimateVehicleSpeed(drivedata.RWSpeed_dps);

    steeringMotorAngle_deg(idx) = str2double(fileName(strfind(fileName,'dc')+2:...
        strfind(fileName,'deg')-1));
    wheelAngle_deg = steeringRatio*steeringMotorAngle_deg(idx);
    approx_steer_angle = deg2rad(wheelAngle_deg);
    
    startIdx = find(drivedata.Time_sec>2,1,'first');
    if steeringMotorAngle_deg(idx) < 0
        stopIdx = find(le(drivedata.Heading_deg,-90),1,'first');
    else
        stopIdx = find(ge(drivedata.Heading_deg,90),1,'first');
    end
    meanDrivingSpeed_mps(idx) = getMean(vehicleSpeed_mps,drivedata.Time_sec,2,drivedata.Time_sec(stopIdx));
    
    timeStep_sec = diff(drivedata.Time_sec(startIdx:startIdx+1));
    
    YawRate_dps = diff(drivedata.Heading_deg(startIdx:stopIdx-1))/timeStep_sec;
    meanYawRate_dps(idx) = abs(mean(YawRate_dps));
    curvature_dpm(idx) = meanYawRate_dps(idx)/meanDrivingSpeed_mps(idx);
    radius_m(idx) = 180/pi/curvature_dpm(idx);

    C_a(idx) = pi/180*((W_f-W_r)*meanDrivingSpeed_mps(idx)^2/9.81/radius_m(idx)...
        /(approx_steer_angle-wheelBase_m/radius_m(idx)));
    
    K_us(idx) = (abs(approx_steer_angle)-wheelBase_m/radius_m(idx))*9.81*radius_m(idx)/(meanDrivingSpeed_mps(idx)^2);
    
    lat_accel(idx) = deg2rad(meanYawRate_dps(idx))*meanDrivingSpeed_mps(idx)/9.81;
    slope(idx) = -K_us(idx)/wheelBase_m;

end
steeringAngle_deg = steeringRatio*steeringMotorAngle_deg;
resultsTable = sortrows(table(steeringAngle_deg,meanDrivingSpeed_mps,...
    curvature_dpm,C_a,lat_accel,slope))

partialTable = resultsTable(resultsTable.steeringAngle_deg==-8.5,:);

figure;
plot(partialTable.lat_accel,deg2rad(partialTable.curvature_dpm),'-ob'); hold on;
plot([0:0.001:0.06],ones(size([0:0.001:0.06]))*deg2rad(28)/wheelBase_m,'-r')
grid on
xlabel('Lateral Acceleration $(m/s^2)$',"Interpreter","latex")
ylabel('Curvature $(rad/m)$',"Interpreter","latex")
ylim([0 2.1])
hold off

figure;
plot(partialTable.lat_accel,partialTable.slope,'-ob'); hold on;
grid on
xlabel('Lateral Acceleration $(m/s^2)$',"Interpreter","latex")
ylabel('$\frac{d(1/R)}{d(a_y/g)}$',"Interpreter","latex")
title(strcat('Steering Angle = 28',char(176)))
ylim([-10 0])
hold off
