%% Rear drivetrain PWM test
clear
load("Data\MotorParameters.mat");
voltage_supply_V = 8.866;

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getVelocityMean = @(speedvec,timevec,x1,x2) mean(speedvec(inRange(timevec,x1,x2)));
getCurrent_mA = @(currentvec,timevec,x1,x2) mean(currentvec(inRange(timevec,x1,x2)));

floatingdata = importDriveData('floatingvehicledata_100ms.txt',4);
floatingdata.Properties.VariableNames{1} = 'Time_sec';
floatingdata.Properties.VariableNames{2} = 'Current_mA';
floatingdata.Properties.VariableNames{3} = 'FWSpeed_dps';
floatingdata.Properties.VariableNames{4} = 'RWSpeed_dps';

load_g = [0,128,231,369,540];
TireRadius_m = 0.04;
n_diff = 22/14;
estimateVehicleSpeed = @(velocity_dps) velocity_dps*pi*TireRadius_m/n_diff/180;
rangeMat = [3,12];
pwm_dc = 0.3;

floatingVehicleSpeed_FW_mps = estimateVehicleSpeed(floatingdata.FWSpeed_dps);
floatingVehicleSpeed_RW_mps= estimateVehicleSpeed(floatingdata.RWSpeed_dps);

for idx = 1:length(load_g)
    myFWData = [];
    myRWData = [];
    frontMotorMeanSpeed_radpsec = zeros(1,3);
    rearMotorMeanSpeed_radpsec = zeros(1,3);
    for jdx = 1:3
        drivedata = importDriveData(strcat('loadeddrivevehicledata_',...
            num2str(load_g(idx)),'g_0',num2str(jdx),'.txt'),4);
        drivedata.Properties.VariableNames{1} = 'Time_sec';
        drivedata.Properties.VariableNames{2} = 'Current_mA';
        drivedata.Properties.VariableNames{3} = 'FWSpeed_dps';
        drivedata.Properties.VariableNames{4} = 'RWSpeed_dps';

        current_mA(:,jdx) = drivedata.Current_mA;

        bias_1 = getCurrent_mA(drivedata.Current_mA,drivedata.Time_sec,0,2);
        bias_2 = getCurrent_mA(drivedata.Current_mA,drivedata.Time_sec,12,14);
        bias_mA = mean([bias_1 bias_2]);

        biased_current_mA(:,jdx) = drivedata.Current_mA-bias_mA;

        myFWData(:,jdx) = estimateVehicleSpeed(drivedata.FWSpeed_dps);
        myRWData(:,jdx) = estimateVehicleSpeed(drivedata.RWSpeed_dps);

        frontMotorMeanSpeed_radpsec(1,jdx) = pi/180*getVelocityMean(...
            drivedata.FWSpeed_dps,drivedata.Time_sec,rangeMat(1),rangeMat(2));

        rearMotorMeanSpeed_radpsec(1,jdx) = pi/180*getVelocityMean(...
            drivedata.RWSpeed_dps,drivedata.Time_sec,rangeMat(1),rangeMat(2));
        
    end
    labelFW = strcat('FW_Load_',num2str(load_g(idx)),'g');
    labelRW = strcat('RW_Load_',num2str(load_g(idx)),'g');
    ensembleAxleBasedSpeedMean_mps.(labelFW) = mean(myFWData,2);
    ensembleAxleBasedSpeedMean_mps.(labelRW) = mean(myRWData,2);

    label = strcat('Load_',num2str(load_g(idx)),'g');
    ensembleVehicleSpeedMean_mps.(label) = mean([...
        ensembleAxleBasedSpeedMean_mps.(labelFW),...
        ensembleAxleBasedSpeedMean_mps.(labelRW)],2);
    motorMeanSpeed_radpsec.(label) = mean([mean(frontMotorMeanSpeed_radpsec,2),...
        mean(rearMotorMeanSpeed_radpsec,2)],2);

    ensembleCurrent_mA.(label) = mean(biased_current_mA,2);
end

%% Find rolling resistance with means calculated
unloaded_vehicle_mass_kg = 0.893;
loaded_vehicle_mass_kg = unloaded_vehicle_mass_kg + load_g/1000;
loaded_vehicle_weight_N = loaded_vehicle_mass_kg*9.81;
W_f_N = loaded_vehicle_weight_N/2;
W_r_N = loaded_vehicle_weight_N/2;

pwmDC_Breakaway = 0.13;
alpha = 1.9; % Tuning factor
T_C_Nm = voltage_supply_V*pwmDC_Breakaway*r_m/resistance_mean_ohm/alpha;
brakeawaySpeed_radpsec = 0.1;

motorMeanSpeedArray_radpsec = struct2array(motorMeanSpeed_radpsec);

T_Cou_Nm = T_C_Nm*tanh(10*motorMeanSpeedArray_radpsec/brakeawaySpeed_radpsec);
T_b_Nm = B_m*motorMeanSpeedArray_radpsec;

x_vec = loaded_vehicle_weight_N*TireRadius_m/n_diff/2;
y_vec = r_m*voltage_supply_V*pwm_dc/resistance_mean_ohm...
    - (r_m^2)*motorMeanSpeedArray_radpsec/resistance_mean_ohm...
    - T_Cou_Nm - T_b_Nm;

x_vec_loadedOnly = x_vec(2:end);
y_vec_loadedOnly = y_vec(2:end);

% Includes unloaded case
lin_model = polyfit(x_vec,y_vec,1);
mu = lin_model(1); T_f_0 = lin_model(2);
T_f_fit = T_f_0 + mu*x_vec;
% Only loaded cases
lin_model_loadedOnly = polyfit(x_vec_loadedOnly,y_vec_loadedOnly,1);
mu_loaded = lin_model_loadedOnly(1); T_f_0_loaded = lin_model_loadedOnly(2);
T_f_fit_loaded = T_f_0_loaded + mu_loaded*x_vec_loadedOnly;

figure
plot(x_vec,y_vec,'-ob',"DisplayName","Data based")
grid on
% xlabel('Torque $T$ (Nm)',"Interpreter","latex")
% ylabel('Motor Voltage $v_m$ (V)',"Interpreter","latex")
hold on; plot(x_vec,T_f_fit,'-r',"DisplayName","Linear fit")
legend('Interpreter','latex')
% title(strcat('Motor constant $r_m$ = ',num2str(r_m),' $V*s/rad$'),"Interpreter","latex")
hold off;

figure
plot(x_vec_loadedOnly,y_vec_loadedOnly,'-ob',"DisplayName","Data based")
grid on
hold on; plot(x_vec_loadedOnly,T_f_fit_loaded,'-r',"DisplayName","Linear fit")
xlabel('$\frac{1}{2n_d}mgR \: (N\cdot m)$',"Interpreter","latex")
ylabel('$\frac{r_m}{R_m} v_{in} - \frac{r_m^2}{R_m} \omega_{m_{ss}} - B_m\omega_{m_{ss}} - T_{Coul}(\omega_{m_{ss}})\: (N\cdot m)$',"Interpreter","latex")
legend('Interpreter','latex','Location','northwest')
title("Loaded Vehicle Test Regression","Interpreter","latex")
hold off;

save("Data\loadedVehicleTestSpeedData.mat",...
    "drivedata","ensembleAxleBasedSpeedMean_mps","ensembleVehicleSpeedMean_mps",...
    "ensembleCurrent_mA");

figure;
plot(floatingdata.Time_sec,floatingdata.FWSpeed_dps,'-b'); hold on;
plot(floatingdata.Time_sec,floatingdata.RWSpeed_dps,'-r');
plot(drivedata.Time_sec,drivedata.FWSpeed_dps,'--b');
plot(drivedata.Time_sec,drivedata.RWSpeed_dps,'--r');
grid on
xlabel('Time $t$ (sec)',"Interpreter","latex")
ylabel('Angular Velocity $\omega$',"Interpreter","latex")
hold off