%% Rear drivetrain PWM test
close all; clear
load("Data\MotorParametersM3.mat","B_m","r_m","resistance_mean_ohm")
load("Data\DriveDataM3.mat")

rundata = drivedata;

voltage_supply_V = 8.866;

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getCurrent_A = @(currentvec,timevec,x1,x2) mean(currentvec(inRange(timevec,x1,x2)))/1000;

drivedata = importDriveData('faxledata_M3_50ms.txt',3);
drivedata.Properties.VariableNames{1} = 'Time_sec';
drivedata.Properties.VariableNames{2} = 'Velocity_dps';
drivedata.Properties.VariableNames{3} = 'Current_mA';

transientTestFlag = 0;
transientTestDC = 0.3;
t_PWM_On = 2;

for idx = 1:length(drivedata.Time_sec)
    if drivedata.Time_sec(idx) >= 2 && drivedata.Time_sec(idx) < 8 
        drivedata.DutyCycle(idx) = 0.15;
    elseif drivedata.Time_sec(idx) >= 10 && drivedata.Time_sec(idx) < 16
        drivedata.DutyCycle(idx) = 0.3;
    elseif drivedata.Time_sec(idx) >= 18 && drivedata.Time_sec(idx) < 24
        drivedata.DutyCycle(idx) = 0.45;
    elseif drivedata.Time_sec(idx) >= 26 && drivedata.Time_sec(idx) < 32
        drivedata.DutyCycle(idx) = 0.6;
    else
        drivedata.DutyCycle(idx) = 0;
    end
end

plottingOn = 1;
if plottingOn
    figure;
    subplot(3,1,1)
    plot(drivedata.Time_sec,drivedata.DutyCycle,'-b'); hold on;
    plot(rundata.Time_sec,rundata.DutyCycle,'-r');
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('PWM Duty Cycle $u(t)$',"Interpreter","latex")
    ylim([0 1])
    subplot(3,1,2)
    plot(drivedata.Time_sec,drivedata.Current_mA,'-b'); hold on;
    plot(rundata.Time_sec,rundata.Current_mA,'-r');
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('Current $i_m$ (mA)',"Interpreter","latex")
    subplot(3,1,3)
    plot(rundata.Time_sec,rundata.Velocity_dps,'-r',...
        "DisplayName","Motor only"); hold on;
    plot(drivedata.Time_sec,drivedata.Velocity_dps,'-b',...
        "DisplayName","Drivetrain");
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('Velocity $\omega_m$ (dps)',"Interpreter","latex")
    legend("Interpreter","latex",'Location','northwest')
    title("PWM DC Sweep Test: Motor Velocity, Motor Load vs. Drivetrain Load",...
        "Interpreter","latex")
    sgtitle('PWM Duty Cycle Motor Test', 'Interpreter', 'latex', 'FontSize', 16);
end

w_radpsec = drivedata.Velocity_dps*pi/180;
getVelocity_radpsec = @(timevec,x1,x2) mean(w_radpsec(inRange(timevec,x1,x2)));
% To calculate r_m, use expression for v_m at steady state
if transientTestFlag
    duty_cycles_dyn = transientTestDC;
    rangeMat = [2.1,8];
else
    duty_cycles_dyn = [0.15,0.30,0.45,0.60];
    rangeMat = [2.25,8;10.25,16;18.25,24;26.25,32];
end

for idx = 1:size(rangeMat,1)
    i_m(idx) = getCurrent_A(drivedata.Current_mA,drivedata.Time_sec,...
        rangeMat(idx,1),rangeMat(idx,2))-current_meanBias_A;
    w_m(idx) = getVelocity_radpsec(drivedata.Time_sec,rangeMat(idx,1),rangeMat(idx,2));
end

if ~transientTestFlag
    % Find B_load at steady state
    figure
    lin_model_mech = polyfit(w_m,r_m*i_m,1);
    B_load = lin_model_mech(1); T_L_0 = lin_model_mech(2);
    T_L_fit = T_L_0 + B_load*w_m;
    plot(w_m,r_m*i_m,'-ob',"DisplayName","Data based")
    grid on
    xlabel('Rotor Speed $\omega_m$ (rad/sec)',"Interpreter","latex")
    ylabel('Motor Torque $T_m$ (Nm)',"Interpreter","latex")
    hold on; plot(w_m,T_L_fit,'-r',"DisplayName","Linear fit")
    legend('Interpreter','latex')
    title(strcat('Motor damping constant $B_m$ = ',num2str(B_load),' $Nm*s/rad$'),"Interpreter","latex")
    hold off;
    BLValid = 1;
else
    load("Data\MotorParameters.mat");
end

% Find J of motor from transient response of omega curve
if transientTestFlag && BLValid
    w_tau = 0.632*w_m;
    jdx = find(w_radpsec>=w_tau(1),1,"first");
    tau = interp1([w_radpsec(jdx-1),w_radpsec(jdx)],...
        [drivedata.Time_sec(jdx-1),drivedata.Time_sec(jdx)],...
        w_tau) - t_PWM_On;
    % tau = time @ w_tau;
    J = B_m*tau;
    save("Data\MotorInertia.mat","J")
end

%% Export parameters to .mat file
% if ~transientTestFlag
%     save("Data\MotorParameters.mat","B_m","r_m","resistance_mean_ohm","rmValid","BmValid")
%     save("Data\DriveData.mat","drivedata","current_meanBias_A")
% end
