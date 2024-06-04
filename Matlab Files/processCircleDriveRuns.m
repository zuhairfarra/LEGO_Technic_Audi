%% Circle test
close all; clear

voltage_supply_V = 8.866;
TireRadius_m = 0.04;
n_diff = 22/14;
steeringRatio = 28/112;
wheelBase_m = 0.237;
estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;
inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getMean = @(valvec,timevec,x1,x2) mean(valvec(inRange(timevec,x1,x2)));

fileList = dir("Data\SteeringTest\FullCircle\");
isTxtFile = arrayfun(@(x) contains(x.name,'txt'),fileList);
fileList = fileList(isTxtFile);

steeringMotorAngle_deg = zeros(numel(fileList),1);
meanDrivingSpeed_mps = zeros(numel(fileList),1);
meanYawRate_dps = zeros(numel(fileList),1);
curvature_dpm = zeros(numel(fileList),1);
radius_m = zeros(numel(fileList),1);
K_us = zeros(numel(fileList),1);
lat_accel = zeros(numel(fileList),1);
slope = zeros(numel(fileList),1);
steeringAngle_deg = zeros(numel(fileList),1);

for idx = 1:numel(fileList)

    fileName = fileList(idx).name;
    drivedata = importDriveData(fileName,3);
    drivedata.Properties.VariableNames{1} = 'Time_sec';
    drivedata.Properties.VariableNames{2} = 'Heading_deg';
    drivedata.Properties.VariableNames{3} = 'RWSpeed_dps';

    figure;
    plot(drivedata.Time_sec,drivedata.Heading_deg)
    grid on

    figure;
    plot(drivedata.Time_sec,drivedata.RWSpeed_dps)
    grid on

    vehicleSpeed_mps = estimateVehicleSpeed(drivedata.RWSpeed_dps);

    steeringMotorAngle_deg(idx) = str2double(fileName(strfind(fileName,'dc')+2:...
        strfind(fileName,'deg')-1));
    
    if steeringMotorAngle_deg(idx) == -34
        wheelAngle_deg = -6.5;
    else
        wheelAngle_deg = steeringRatio*steeringMotorAngle_deg(idx);
    end
    approx_steer_angle = deg2rad(wheelAngle_deg);
    
    startIdx = find(drivedata.Time_sec>2.5,1,'first');
    if steeringMotorAngle_deg(idx) < 0
        stopIdx = find(le(drivedata.Heading_deg,-360),1,'first');
    else
        stopIdx = find(ge(drivedata.Heading_deg,360),1,'first');
    end
    meanDrivingSpeed_mps(idx) = getMean(vehicleSpeed_mps,drivedata.Time_sec,2.5,drivedata.Time_sec(stopIdx));
    
    timeStep_sec = diff(drivedata.Time_sec(startIdx:startIdx+1));
    
    YawRate_dps = diff(drivedata.Heading_deg(startIdx:stopIdx-1))/timeStep_sec;
    meanYawRate_dps(idx) = abs(mean(YawRate_dps));
    curvature_dpm(idx) = meanYawRate_dps(idx)/meanDrivingSpeed_mps(idx);
    radius_m(idx) = 180/pi/curvature_dpm(idx);
    
    K_us(idx) = (abs(approx_steer_angle)-wheelBase_m/radius_m(idx))*9.81*radius_m(idx)/(meanDrivingSpeed_mps(idx)^2);
    
    lat_accel(idx) = deg2rad(meanYawRate_dps(idx))*meanDrivingSpeed_mps(idx)/9.81;
    slope(idx) = -K_us(idx)/wheelBase_m;

    steeringAngle_deg(idx) = wheelAngle_deg;

end

resultsTable = sortrows(table(steeringAngle_deg,meanDrivingSpeed_mps,curvature_dpm,K_us,lat_accel,slope))

partialTable = resultsTable(resultsTable.steeringAngle_deg==-28,:);

figure;
plot(partialTable.lat_accel,deg2rad(partialTable.curvature_dpm),'-ob',...
    "DisplayName","Test Results"); hold on;
plot([0:0.001:0.06],ones(size([0:0.001:0.06]))*deg2rad(28)/wheelBase_m,'-r',...
    "DisplayName","$\frac{\delta_f}{L}$")
grid on
xlabel('Lateral Acceleration $(m/s^2)$',"Interpreter","latex")
ylabel('Curvature $(rad/m)$',"Interpreter","latex")
legend("Interpreter","latex","Location","east")
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

%% Old content

% fileName = 'circledrivedata_60dc-82deg_01.txt';
% 
% drivedata = importDriveData(fileName,3);
% drivedata.Properties.VariableNames{1} = 'Time_sec';
% drivedata.Properties.VariableNames{2} = 'Heading_deg';
% drivedata.Properties.VariableNames{3} = 'RWSpeed_dps';
% 
% vehicleSpeed_mps = estimateVehicleSpeed(drivedata.RWSpeed_dps);
% 
% figure;
% subplot(1,2,1)
% plot(drivedata.Time_sec,vehicleSpeed_mps,'-b');
% grid on
% xlabel('Time $t$ (sec)',"Interpreter","latex")
% ylabel('Vehicle Speed (mps) $v$',"Interpreter","latex")
% hold off
% subplot(1,2,2)
% plot(drivedata.Time_sec,drivedata.Heading_deg,'-b');
% grid on
% xlabel('Time $t$ (sec)',"Interpreter","latex")
% ylabel('Heading $(deg)$',"Interpreter","latex")
% hold off
% 
